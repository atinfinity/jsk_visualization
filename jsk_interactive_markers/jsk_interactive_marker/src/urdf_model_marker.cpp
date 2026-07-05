#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <filesystem>
#include <interactive_markers/tools.hpp>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <Eigen/Geometry>

#include <kdl/frames_io.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using namespace urdf;
using namespace std;
using namespace im_utils;

namespace {
// tf::poseMsgToKDL / tf::poseKDLToMsg replacements (tf_conversions is not
// available in ROS 2; use KDL directly)
void poseMsgToKDL(const geometry_msgs::msg::Pose &p, KDL::Frame &f)
{
  f.p = KDL::Vector(p.position.x, p.position.y, p.position.z);
  f.M = KDL::Rotation::Quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w);
}

void poseKDLToMsg(const KDL::Frame &f, geometry_msgs::msg::Pose &p)
{
  p.position.x = f.p.x();
  p.position.y = f.p.y();
  p.position.z = f.p.z();
  f.M.GetQuaternion(p.orientation.x, p.orientation.y,
                    p.orientation.z, p.orientation.w);
}

// tf2 does not accept frame_ids with a leading slash
std::string stripSlash(const std::string &frame)
{
  if (!frame.empty() && frame[0] == '/') {
    return frame.substr(1);
  }
  return frame;
}
}

void UrdfModelMarker::addMoveMarkerControl(visualization_msgs::msg::InteractiveMarker &int_marker, LinkConstSharedPtr link, bool root) {
  visualization_msgs::msg::InteractiveMarkerControl control;
  if (root) {
    im_helpers::add6DofControl(int_marker,false);
    for(size_t i=0; i<int_marker.controls.size(); i++) {
      int_marker.controls[i].always_visible = true;
    }
  }
  else {
    JointSharedPtr parent_joint = link->parent_joint;
    Eigen::Vector3f origin_x(1,0,0);
    Eigen::Vector3f dest_x(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);
    Eigen::Quaternionf qua;

    qua.setFromTwoVectors(origin_x, dest_x);
    control.orientation.x = qua.x();
    control.orientation.y = qua.y();
    control.orientation.z = qua.z();
    control.orientation.w = qua.w();

    int_marker.scale = 0.5;

    switch(parent_joint->type) {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      break;
    case Joint::PRISMATIC:
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
      break;
    default:
      break;
    }
  }
}

void UrdfModelMarker::addInvisibleMeshMarkerControl(visualization_msgs::msg::InteractiveMarker &int_marker, LinkConstSharedPtr link, const std_msgs::msg::ColorRGBA &color) {
  visualization_msgs::msg::InteractiveMarkerControl control;
  visualization_msgs::msg::Marker marker;

  //if (use_color) marker.color = color;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  double scale=0.01;
  marker.scale.x = scale;
  marker.scale.y = scale * 1;
  marker.scale.z = scale * 40;
  marker.color = color;
  JointSharedPtr parent_joint = link->parent_joint;
  Eigen::Vector3f origin_x(0,0,1);
  Eigen::Vector3f dest_x(parent_joint->axis.x, parent_joint->axis.y, parent_joint->axis.z);
  Eigen::Quaternionf qua;

  qua.setFromTwoVectors(origin_x, dest_x);
  marker.pose.orientation.x = qua.x();
  marker.pose.orientation.y = qua.y();
  marker.pose.orientation.z = qua.z();
  marker.pose.orientation.w = qua.w();

  control.markers.push_back(marker);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  int_marker.controls.push_back(control);
  return;
}


void UrdfModelMarker::addGraspPointControl(visualization_msgs::msg::InteractiveMarker &int_marker, std::string link_frame_name_) {
  //yellow sphere
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  if (linkMarkerMap[link_frame_name_].gp.displayMoveMarker) {
    im_helpers::add6DofControl(int_marker,false);
  }
}


// dynamic_tf_publisher SetDynamicTF service replacement:
// register/update the transform in a local map which is broadcast
// periodically (20 Hz, same freq as the ROS 1 service request).
void UrdfModelMarker::callSetDynamicTf(string parent_frame_id, string frame_id, geometry_msgs::msg::Transform transform) {
  if (use_dynamic_tf_ || parent_frame_id == frame_id_) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.stamp = node_->now();
    tf_stamped.header.frame_id = stripSlash(parent_frame_id);
    tf_stamped.child_frame_id = stripSlash(frame_id);
    tf_stamped.transform = transform;
    std::lock_guard<std::mutex> lock(dynamic_tf_mutex_);
    dynamic_tf_map_[tf_stamped.child_frame_id] = tf_stamped;
  }
}

// dynamic_tf_publisher publish_tf service replacement:
// flush all registered transforms immediately.
void UrdfModelMarker::callPublishTf() {
  if (use_dynamic_tf_) {
    publishDynamicTf();
  }
}

void UrdfModelMarker::publishDynamicTf() {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  {
    std::lock_guard<std::mutex> lock(dynamic_tf_mutex_);
    rclcpp::Time now = node_->now();
    for (auto &it : dynamic_tf_map_) {
      it.second.header.stamp = now;
      transforms.push_back(it.second);
    }
  }
  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

void UrdfModelMarker::publishBasePose(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  publishBasePose(feedback->pose, feedback->header);
}

void UrdfModelMarker::publishBasePose(geometry_msgs::msg::Pose pose, std_msgs::msg::Header header) {
  geometry_msgs::msg::PoseStamped ps;
  ps.pose = pose;
  ps.header = header;
  pub_base_pose_->publish(ps);
}



void UrdfModelMarker::publishMarkerPose(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  publishMarkerPose(feedback->pose, feedback->header, feedback->marker_name);
}

void UrdfModelMarker::publishMarkerPose(geometry_msgs::msg::Pose pose, std_msgs::msg::Header header, std::string marker_name) {
  jsk_interactive_marker_msgs::msg::MarkerPose mp;
  mp.pose.header = header;
  mp.pose.pose = pose;
  mp.marker_name = marker_name;
  mp.type = jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_GENERAL;
  pub_->publish(mp);
}


void UrdfModelMarker::publishMarkerMenu(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int menu) {
  jsk_interactive_marker_msgs::msg::MarkerMenu m;
  m.marker_name = feedback->marker_name;
  m.menu=menu;
  pub_move_->publish(m);
}

void UrdfModelMarker::publishMoveObject(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  (void)feedback;
}

void UrdfModelMarker::publishJointState(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  (void)feedback;
  publishJointState();
}

void UrdfModelMarker::republishJointState(sensor_msgs::msg::JointState js) {
  js.header.stamp = node_->now();
  pub_joint_state_->publish(js);
}

void UrdfModelMarker::setRootPoseCB(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  setRootPose(*msg);
}
void UrdfModelMarker::setRootPose (geometry_msgs::msg::PoseStamped ps) {
  try {
    init_stamp_ = ps.header.stamp;
    ps.header.frame_id = stripSlash(ps.header.frame_id);
    ps = tf_buffer_->transform(ps, stripSlash(frame_id_));

    geometry_msgs::msg::Pose pose = getRootPose(ps.pose);

    string root_frame = tf_prefix_ + model->getRoot()->name;
    linkMarkerMap[frame_id_].pose = pose;
    callSetDynamicTf(frame_id_, root_frame, Pose2Transform(pose));
    root_pose_ = pose;
    addChildLinkNames(model->getRoot(), true, false);
    publishMarkerPose(pose, ps.header, root_frame);

  }
  catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s",ex.what());
  }

}




void UrdfModelMarker::resetJointStatesCB(const sensor_msgs::msg::JointState::ConstSharedPtr &msg, bool update_root) {
  std::lock_guard<std::mutex> lock(joint_states_mutex_);
  if (is_joint_states_locked_) {
    return;
  }
  setJointState(model->getRoot(), msg);
  republishJointState(*msg);

  //update root correctly
  //this may take long time
  if (update_root) {
    callPublishTf();
  }
  resetRootForVisualization();
  server_->applyChanges();
}


void UrdfModelMarker::proc_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, string parent_frame_id, string frame_id) {
  RCLCPP_INFO(node_->get_logger(), "proc_feedback");
  switch (feedback->event_type) {
  case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
  case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
    linkMarkerMap[frame_id].pose = feedback->pose;
    //root link
    if (parent_frame_id == frame_id_) {
      root_pose_ = feedback->pose;
      publishBasePose(feedback);
    }
    callSetDynamicTf(parent_frame_id, frame_id, Pose2Transform(feedback->pose));
    publishMarkerPose(feedback);
    publishJointState(feedback);
    break;
  case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
    cout << "clicked" << " frame:" << frame_id << mode_ << endl;
    if (mode_ != "visualization") {
      linkMarkerMap[linkMarkerMap[frame_id].movable_link].displayMoveMarker ^= true;
      addChildLinkNames(model->getRoot(), true, false);
    }
    else {
      geometry_msgs::msg::PoseStamped ps = getOriginPoseStamped();
      pub_selected_->publish(ps);
      jsk_recognition_msgs::msg::Int32Stamped index_msg;
      index_msg.data = index_;
      index_msg.header.stamp = init_stamp_;
      pub_selected_index_->publish(index_msg);
    }
    break;

  }
}



void UrdfModelMarker::graspPointCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  KDL::Vector graspVec(feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z);
  KDL::Frame parentFrame;
  poseMsgToKDL (linkMarkerMap[feedback->marker_name].pose, parentFrame);

  graspVec = parentFrame.Inverse(graspVec);

  geometry_msgs::msg::Pose p;
  p.position.x = graspVec.x();
  p.position.y = graspVec.y();
  p.position.z = graspVec.z();
  p.orientation = linkMarkerMap[feedback->marker_name].gp.pose.orientation;
  linkMarkerMap[feedback->marker_name].gp.pose = p;

  linkMarkerMap[feedback->marker_name].gp.displayGraspPoint = true;
  addChildLinkNames(model->getRoot(), true, false);
}


void UrdfModelMarker::jointMoveCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  publishJointState(feedback);
  publishMarkerMenu(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::JOINT_MOVE);
}

void UrdfModelMarker::resetMarkerCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {

  publishJointState(feedback);
  publishMarkerMenu(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::RESET_JOINT);
}

void UrdfModelMarker::resetBaseMsgCB(const std_msgs::msg::Empty::ConstSharedPtr &msg) {
  (void)msg;
  resetBaseCB();
}

void UrdfModelMarker::resetBaseMarkerCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  (void)feedback;
  resetBaseCB();
}
void UrdfModelMarker::resetBaseCB() {
  resetRobotBase();

  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame_id_;
  ps.pose = root_pose_;
  setRootPose(ps);

  // to update root link marker
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::resetRobotBase() {
  //set root_pose_ to robot base pose
  try {
    geometry_msgs::msg::TransformStamped ts_msg;
    ts_msg = tf_buffer_->lookupTransform(stripSlash(frame_id_), stripSlash(model->getRoot()->name),
                                         tf2::TimePointZero);

    root_pose_ = Transform2Pose(ts_msg.transform);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s",ex.what());
  }
}

void UrdfModelMarker::resetRootForVisualization() {
  if (fixed_link_.size() > 0 && (mode_ == "visualization" || mode_ == "robot")) {
    tf2::Transform st_offset;
    st_offset.setIdentity();
    bool first_offset = true;
    for(size_t i=0; i<fixed_link_.size(); i++) {
      std::string link = fixed_link_[i];
      if (!link.empty()) {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "fixed_link:" << tf_prefix_ + model->getRoot()->name << tf_prefix_ + link);
        const std::string source_frame = stripSlash(tf_prefix_ + link);
        const std::string target_frame = stripSlash(tf_prefix_ + model->getRoot()->name);
        try {
          // ROS 1 used a blocking waitForTransform(5.0s); use canTransform
          // with timeout instead.
          if (!tf_buffer_->canTransform(target_frame, source_frame,
                                        tf2::TimePointZero,
                                        tf2::durationFromSec(5.0))) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to lookup transformation from %s to %s: timeout",
                         source_frame.c_str(), target_frame.c_str());
            continue;
          }
          geometry_msgs::msg::TransformStamped ts_link_offset =
            tf_buffer_->lookupTransform(target_frame, source_frame,
                                        tf2::TimePointZero);
          tf2::Transform st_link_offset;
          tf2::fromMsg(ts_link_offset.transform, st_link_offset);

          if (first_offset) {
            st_offset.setRotation(st_link_offset.getRotation());
            st_offset.setOrigin(st_link_offset.getOrigin());
            first_offset = false;
          }
          else {
            st_offset.setRotation(st_link_offset.getRotation().slerp(st_offset.getRotation(), (i * 1.0)/(i + 1)));
            st_offset.setOrigin(st_link_offset.getOrigin().lerp(st_offset.getOrigin(), (i* 1.0)/(i+1)));
          }
        }
        catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(node_->get_logger(), "Failed to lookup transformation from %s to %s: %s",
                       source_frame.c_str(), target_frame.c_str(),
                       ex.what());
        }
      }
    }

    //multiply fixed_link_offset_
    tf2::Transform st_fixed_link_offset;
    st_fixed_link_offset.setOrigin(tf2::Vector3(fixed_link_offset_.position.x,
                                                fixed_link_offset_.position.y,
                                                fixed_link_offset_.position.z));
    st_fixed_link_offset.setRotation(tf2::Quaternion(fixed_link_offset_.orientation.x,
                                                     fixed_link_offset_.orientation.y,
                                                     fixed_link_offset_.orientation.z,
                                                     fixed_link_offset_.orientation.w));

    tf2::Transform transform;
    transform = st_offset * st_fixed_link_offset;

    //convert to root_offset_
    geometry_msgs::msg::Transform tf_msg = tf2::toMsg(transform);

    root_offset_.position.x = tf_msg.translation.x;
    root_offset_.position.y = tf_msg.translation.y;
    root_offset_.position.z = tf_msg.translation.z;
    root_offset_.orientation = tf_msg.rotation;

    //reset root_pose_
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = node_->now();
    ps.header.frame_id = frame_id_;
    ps.pose.orientation.w = 1.0;
    root_pose_ = ps.pose;

    geometry_msgs::msg::Pose pose = getRootPose(ps.pose);

    string root_frame = tf_prefix_ + model->getRoot()->name;
    linkMarkerMap[frame_id_].pose = pose;
    callSetDynamicTf(frame_id_, root_frame, Pose2Transform(pose));

    addChildLinkNames(model->getRoot(), true, false);
  }
}




void UrdfModelMarker::registrationCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {

  publishJointState(feedback);
}

void UrdfModelMarker::moveCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  /* publish jsk_interactive_marker_msgs::msg::MoveModel */
  jsk_interactive_marker_msgs::msg::MoveModel mm;
  mm.header = feedback->header;
  mm.name = model_name_;
  mm.description = model_description_;
  mm.joint_state_origin = joint_state_origin_;
  mm.joint_state_goal = joint_state_;
  mm.pose_origin.header.frame_id = frame_id_;
  mm.pose_origin.pose = root_pose_origin_;
  mm.pose_goal.header.frame_id = frame_id_;
  mm.pose_goal.pose = root_pose_;
  pub_move_model_->publish(mm);


  /* publish jsk_interactive_marker_msgs::msg::MoveObject */
  jsk_interactive_marker_msgs::msg::MoveObject mo;
  mo.origin.header = feedback->header;
  mo.origin.pose = linkMarkerMap[feedback->marker_name].origin;

  mo.goal.header = feedback->header;
  mo.goal.pose = feedback->pose;

  mo.grasp_pose = linkMarkerMap[feedback->marker_name].gp.pose;
  pub_move_object_->publish(mo);

}

void UrdfModelMarker::setPoseCB() {
  cout << "setPose" <<endl;
  joint_state_origin_ = joint_state_;
  root_pose_origin_ = root_pose_;
  setOriginalPose(model->getRoot());
}

void UrdfModelMarker::setPoseCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  (void)feedback;
  setPoseCB();
}

void UrdfModelMarker::hideMarkerCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  linkMarkerMap[linkMarkerMap[feedback->marker_name].movable_link].displayMoveMarker = false;
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::hideAllMarkerCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  (void)feedback;
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while (it != linkMarkerMap.end())
  {
    (*it).second.displayMoveMarker = false;
    ++it;
  }
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::hideModelMarkerCB(const std_msgs::msg::Empty::ConstSharedPtr &msg) {
  (void)msg;
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while (it != linkMarkerMap.end())
  {
    (*it).second.displayModelMarker = false;
    ++it;
  }
  addChildLinkNames(model->getRoot(), true, false);
}

void UrdfModelMarker::showModelMarkerCB(const std_msgs::msg::Empty::ConstSharedPtr &msg) {
  (void)msg;
  map<string, linkProperty>::iterator it = linkMarkerMap.begin();
  while (it != linkMarkerMap.end())
  {
    (*it).second.displayModelMarker = true;
    ++it;
  }
  addChildLinkNames(model->getRoot(), true, false);

}

void UrdfModelMarker::setUrdfCB(const std_msgs::msg::String::ConstSharedPtr &msg) {
  //clear
  server_->clear();
  linkMarkerMap.clear();

  model = parseURDF(msg->data);
  if (!model) {
    RCLCPP_ERROR(node_->get_logger(), "Model Parsing the xml failed");
    return;
  }
  addChildLinkNames(model->getRoot(), true, true);

  // start JointState
  publishJointState();
  return;
}


void UrdfModelMarker::graspPoint_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, string link_name) {
  switch (feedback->event_type) {
  case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    linkMarkerMap[link_name].gp.pose = feedback->pose;
    publishMarkerPose(feedback);
    break;
  case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
    cout << "clicked" << " frame:" << feedback->marker_name << endl;
    linkMarkerMap[link_name].gp.displayMoveMarker ^= true;
    addChildLinkNames(model->getRoot(), true, false);
    break;
  }
}

visualization_msgs::msg::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA &color, bool use_color) {
  visualization_msgs::msg::Marker meshMarker;

  if (use_color) meshMarker.color = color;
  meshMarker.mesh_resource = mesh_resource;
  meshMarker.mesh_use_embedded_materials = !use_color;
  meshMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

  meshMarker.scale = scale;
  meshMarker.pose = stamped.pose;
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.markers.push_back(meshMarker);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  return control;
}

visualization_msgs::msg::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale)
{
  std_msgs::msg::ColorRGBA color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = 0;
  return makeMeshMarkerControl(mesh_resource, stamped, scale, color, false);
}

visualization_msgs::msg::InteractiveMarkerControl UrdfModelMarker::makeMeshMarkerControl(const std::string &mesh_resource,
                                                                                         const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA &color)
{
  return makeMeshMarkerControl(mesh_resource, stamped, scale, color, true);
}

visualization_msgs::msg::InteractiveMarkerControl UrdfModelMarker::makeCylinderMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, double length,  double radius, const std_msgs::msg::ColorRGBA &color, bool use_color) {
  visualization_msgs::msg::Marker cylinderMarker;

  if (use_color) cylinderMarker.color = color;
  cylinderMarker.type = visualization_msgs::msg::Marker::CYLINDER;
  cylinderMarker.scale.x = radius * 2;
  cylinderMarker.scale.y = radius * 2;
  cylinderMarker.scale.z = length;
  cylinderMarker.pose = stamped.pose;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.markers.push_back(cylinderMarker);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  return control;
}

visualization_msgs::msg::InteractiveMarkerControl UrdfModelMarker::makeBoxMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, Vector3 dim, const std_msgs::msg::ColorRGBA &color, bool use_color) {
  visualization_msgs::msg::Marker boxMarker;

  fprintf(stderr, "urdfModelMarker = %f %f %f\n", dim.x, dim.y, dim.z);
  if (use_color) boxMarker.color = color;
  boxMarker.type = visualization_msgs::msg::Marker::CUBE;
  boxMarker.scale.x = dim.x;
  boxMarker.scale.y = dim.y;
  boxMarker.scale.z = dim.z;
  boxMarker.pose = stamped.pose;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.markers.push_back(boxMarker);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  return control;
}

visualization_msgs::msg::InteractiveMarkerControl UrdfModelMarker::makeSphereMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, double rad, const std_msgs::msg::ColorRGBA &color, bool use_color) {
  visualization_msgs::msg::Marker sphereMarker;

  if (use_color) sphereMarker.color = color;
  sphereMarker.type = visualization_msgs::msg::Marker::SPHERE;
  sphereMarker.scale.x = rad * 2;
  sphereMarker.scale.y = rad * 2;
  sphereMarker.scale.z = rad * 2;
  sphereMarker.pose = stamped.pose;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.markers.push_back(sphereMarker);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  return control;
}

void UrdfModelMarker::publishJointState() {
  getJointState();
  pub_joint_state_->publish(joint_state_);
}

void UrdfModelMarker::getJointState() {
  sensor_msgs::msg::JointState new_joint_state;
  joint_state_ = new_joint_state;
  joint_state_.header.stamp = node_->now();
  getJointState(model->getRoot());
}

void UrdfModelMarker::getJointState(LinkConstSharedPtr link)
{
  string link_frame_name_ =  tf_prefix_ + link->name;
  JointSharedPtr parent_joint = link->parent_joint;
  if (parent_joint != NULL) {
    KDL::Frame initialFrame;
    KDL::Frame presentFrame;
    KDL::Rotation rot;
    KDL::Vector rotVec;
    KDL::Vector jointVec;
    double jointAngle;
    double jointAngleAllRange;
    switch(parent_joint->type) {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
    {
      linkProperty *link_property = &linkMarkerMap[link_frame_name_];
      poseMsgToKDL (link_property->initial_pose, initialFrame);
      poseMsgToKDL (link_property->pose, presentFrame);
      rot = initialFrame.M.Inverse() * presentFrame.M;
      jointAngle = rot.GetRotAngle(rotVec);
      jointVec = KDL::Vector(link_property->joint_axis.x,
                             link_property->joint_axis.y,
                             link_property->joint_axis.z);
      if (KDL::dot(rotVec,jointVec) < 0) {
        jointAngle = - jointAngle;
      }
      if (link_property->joint_angle > M_PI/2 && jointAngle < -M_PI/2) {
        link_property->rotation_count += 1;
      }
      else if (link_property->joint_angle < -M_PI/2 && jointAngle > M_PI/2) {
        link_property->rotation_count -= 1;
      }
      link_property->joint_angle = jointAngle;
      jointAngleAllRange = jointAngle + link_property->rotation_count * M_PI * 2;

      if (parent_joint->type == Joint::REVOLUTE && parent_joint->limits != NULL) {
        bool changeMarkerAngle = false;
        if (jointAngleAllRange < parent_joint->limits->lower) {
          jointAngleAllRange = parent_joint->limits->lower + 0.001;
          changeMarkerAngle = true;
        }
        if (jointAngleAllRange > parent_joint->limits->upper) {
          jointAngleAllRange = parent_joint->limits->upper - 0.001;
          changeMarkerAngle = true;
        }

        if (changeMarkerAngle) {
          setJointAngle(link, jointAngleAllRange);
        }
      }

      joint_state_.position.push_back(jointAngleAllRange);
      joint_state_.name.push_back(parent_joint->name);
      break;
    }
    case Joint::PRISMATIC:
    {
      KDL::Vector pos;
      linkProperty *link_property = &linkMarkerMap[link_frame_name_];
      poseMsgToKDL (link_property->initial_pose, initialFrame);
      poseMsgToKDL (link_property->pose, presentFrame);

      pos = presentFrame.p - initialFrame.p;

      jointVec = KDL::Vector(link_property->joint_axis.x,
                             link_property->joint_axis.y,
                             link_property->joint_axis.z);
      jointVec = jointVec / jointVec.Norm(); // normalize vector
      jointAngle = KDL::dot(jointVec, pos);

      link_property->joint_angle = jointAngle;
      jointAngleAllRange = jointAngle;

      if (parent_joint->type == Joint::PRISMATIC && parent_joint->limits != NULL) {
        bool changeMarkerAngle = false;
        if (jointAngleAllRange < parent_joint->limits->lower) {
          jointAngleAllRange = parent_joint->limits->lower + 0.003;
          changeMarkerAngle = true;
        }
        if (jointAngleAllRange > parent_joint->limits->upper) {
          jointAngleAllRange = parent_joint->limits->upper - 0.003;
          changeMarkerAngle = true;
        }
        if (changeMarkerAngle) {
          setJointAngle(link, jointAngleAllRange);
        }
      }

      joint_state_.position.push_back(jointAngleAllRange);
      joint_state_.name.push_back(parent_joint->name);
      break;
    }
    case Joint::FIXED:
      break;
    default:
      break;
    }
    server_->applyChanges();
  }

  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    getJointState(*child);
  }
  return;
}

void UrdfModelMarker::setJointAngle(LinkConstSharedPtr link, double joint_angle) {
  string link_frame_name_ =  tf_prefix_ + link->name;
  JointSharedPtr parent_joint = link->parent_joint;

  if (parent_joint == NULL) {
    return;
  }

  KDL::Frame initialFrame;
  KDL::Frame presentFrame;
  KDL::Rotation rot;
  KDL::Vector rotVec;
  KDL::Vector jointVec;

  std_msgs::msg::Header link_header;

  int rotation_count = 0;

  switch(parent_joint->type) {
  case Joint::REVOLUTE:
  case Joint::CONTINUOUS:
  {
    if (joint_angle > M_PI) {
      rotation_count = (int)((joint_angle + M_PI) / (M_PI * 2));
      joint_angle -= rotation_count * M_PI * 2;
    }
    else if (joint_angle < -M_PI) {
      rotation_count = (int)((- joint_angle + M_PI) / (M_PI * 2));
      joint_angle -= rotation_count * M_PI * 2;
    }
    linkProperty *link_property = &linkMarkerMap[link_frame_name_];
    link_property->joint_angle = joint_angle;
    link_property->rotation_count = rotation_count;

    poseMsgToKDL (link_property->initial_pose, initialFrame);
    poseMsgToKDL (link_property->initial_pose, presentFrame);
    jointVec = KDL::Vector(link_property->joint_axis.x,
                           link_property->joint_axis.y,
                           link_property->joint_axis.z);

    presentFrame.M = KDL::Rotation::Rot(jointVec, joint_angle) * initialFrame.M;
    poseKDLToMsg(presentFrame, link_property->pose);

    break;
  }
  case Joint::PRISMATIC:
  {
    linkProperty *link_property = &linkMarkerMap[link_frame_name_];
    link_property->joint_angle = joint_angle;
    link_property->rotation_count = rotation_count;
    poseMsgToKDL (link_property->initial_pose, initialFrame);
    poseMsgToKDL (link_property->initial_pose, presentFrame);
    jointVec = KDL::Vector(link_property->joint_axis.x,
                           link_property->joint_axis.y,
                           link_property->joint_axis.z);
    jointVec = jointVec / jointVec.Norm(); // normalize vector
    presentFrame.p = joint_angle * jointVec + initialFrame.p;
    poseKDLToMsg(presentFrame, link_property->pose);
    break;
  }
  default:
    break;
  }

  link_header.stamp = builtin_interfaces::msg::Time();
  link_header.frame_id = linkMarkerMap[link_frame_name_].frame_id;

  server_->setPose(link_frame_name_, linkMarkerMap[link_frame_name_].pose, link_header);
  callSetDynamicTf(linkMarkerMap[link_frame_name_].frame_id, link_frame_name_, Pose2Transform(linkMarkerMap[link_frame_name_].pose));
}

void UrdfModelMarker::setJointState(LinkConstSharedPtr link, const sensor_msgs::msg::JointState::ConstSharedPtr &js)
{
  JointSharedPtr parent_joint = link->parent_joint;
  if (parent_joint != NULL) {
    double jointAngle;
    bool changeAngle = false;
    switch(parent_joint->type) {
    case Joint::REVOLUTE:
    case Joint::CONTINUOUS:
      for(size_t i=0; i< js->name.size(); i++) {
        if (js->name[i] == parent_joint->name) {
          jointAngle = js->position[i];
          changeAngle = true;
          break;
        }
      }
      if (!changeAngle) {
        break;
      }
      setJointAngle(link, jointAngle);
      break;
    case Joint::PRISMATIC:
      for(size_t i=0; i< js->name.size(); i++) {
        if (js->name[i] == parent_joint->name) {
          jointAngle = js->position[i];
          changeAngle = true;
          break;
        }
      }
      if (!changeAngle) {
        break;
      }
      setJointAngle(link, jointAngle);
      break;
    default:
      break;
    }
  }
  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    setJointState(*child, js);
  }
  return;
}

geometry_msgs::msg::Pose UrdfModelMarker::getRootPose(geometry_msgs::msg::Pose pose) {
  KDL::Frame pose_frame, offset_frame;
  poseMsgToKDL(pose, pose_frame);
  poseMsgToKDL(root_offset_, offset_frame);
  pose_frame = pose_frame * offset_frame.Inverse();
  poseKDLToMsg(pose_frame, pose);
  return pose;
}

geometry_msgs::msg::PoseStamped UrdfModelMarker::getOriginPoseStamped() {
  geometry_msgs::msg::PoseStamped ps;
  geometry_msgs::msg::Pose pose;
  pose = root_pose_;
  KDL::Frame pose_frame, offset_frame;
  poseMsgToKDL(pose, pose_frame);
  poseMsgToKDL(root_offset_, offset_frame);
  pose_frame = pose_frame * offset_frame;
  poseKDLToMsg(pose_frame, pose);
  ps.pose = pose;
  ps.header.frame_id = frame_id_;
  ps.header.stamp = init_stamp_;
  return ps;
}


void UrdfModelMarker::setOriginalPose(LinkConstSharedPtr link)
{
  string link_frame_name_ =  tf_prefix_ + link->name;
  linkMarkerMap[link_frame_name_].origin =  linkMarkerMap[link_frame_name_].pose;

  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    setOriginalPose(*child);
  }
}

void UrdfModelMarker::addChildLinkNames(LinkConstSharedPtr link, bool root, bool init) {
  addChildLinkNames(link, root, init, use_visible_color_, 0);
}

void UrdfModelMarker::addChildLinkNames(LinkConstSharedPtr link, bool root, bool init, bool use_color, int color_index)
{
  geometry_msgs::msg::PoseStamped ps;

  string link_frame_name_ =  tf_prefix_ + link->name;
  string parent_link_frame_name_;
  RCLCPP_INFO(node_->get_logger(), "link_frame_name: %s", link_frame_name_.c_str());
  if (root) {
    parent_link_frame_name_ = frame_id_;
    ps.pose = getRootPose(root_pose_);
  }
  else {
    parent_link_frame_name_ = link->parent_joint->parent_link_name;
    parent_link_frame_name_ = tf_prefix_ + parent_link_frame_name_;
    ps.pose = UrdfPose2Pose(link->parent_joint->parent_to_joint_origin_transform);
  }
  ps.header.frame_id =  stripSlash(parent_link_frame_name_);
  ps.header.stamp = builtin_interfaces::msg::Time();

  //initialize linkProperty
  if (init) {
    callSetDynamicTf(parent_link_frame_name_, link_frame_name_, Pose2Transform(ps.pose));
    linkProperty lp;
    lp.pose = ps.pose;
    lp.origin = ps.pose;
    lp.initial_pose = ps.pose;
    if (link->parent_joint !=NULL) {
      lp.joint_axis = link->parent_joint->axis;
    }
    lp.joint_angle = 0;
    lp.rotation_count=0;
    if (link->parent_joint !=NULL && link->parent_joint->type == Joint::FIXED) {
      lp.movable_link = linkMarkerMap[parent_link_frame_name_].movable_link;
    }
    else {
      lp.movable_link = link_frame_name_;
    }

    linkMarkerMap.insert(map<string, linkProperty>::value_type(link_frame_name_, lp));
  }

  linkMarkerMap[link_frame_name_].frame_id = parent_link_frame_name_;

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header = ps.header;

  int_marker.name = link_frame_name_;
  if (root) {
    int_marker.description = model_description_;
  }
  int_marker.scale = 1.0;
  int_marker.pose = ps.pose;


  if (!init && !root) {
    visualization_msgs::msg::InteractiveMarker old_marker;
    if (server_->get(link_frame_name_, old_marker)) {
      int_marker.pose = old_marker.pose;
    }
  }


  //hide marker
  if (!linkMarkerMap[link_frame_name_].displayModelMarker) {
    server_->erase(link_frame_name_);
    server_->erase(tf_prefix_ + link->name + "/grasp"); //grasp marker
  }
  else {

    //move Marker
    if (linkMarkerMap[link_frame_name_].displayMoveMarker) {
      addMoveMarkerControl(int_marker, link, root);
    }
    //model Mesh Marker
    std_msgs::msg::ColorRGBA color;
    if (mode_ == "visualization") {
      color.r = (double)0xFF / 0xFF;
      color.g = (double)0xFF / 0xFF;
      color.b = (double)0x00 / 0xFF;
      color.a = 0.5;
    }
    else {
      switch(color_index %3) {
      case 0:
        color.r = (double)0xFF / 0xFF;
        color.g = (double)0xC3 / 0xFF;
        color.b = (double)0x00 / 0xFF;
        break;
      case 1:
        color.r = (double)0x58 / 0xFF;
        color.g = (double)0x0E / 0xFF;
        color.b = (double)0xAD / 0xFF;
        break;
      case 2:
        color.r = (double)0x0C / 0xFF;
        color.g = (double)0x5A / 0xFF;
        color.b = (double)0xA6 / 0xFF;
        break;
      }
      color.a = 1.0;
    }

    //link_array
    std::vector<VisualSharedPtr > visual_array;
    if (link->visual_array.size() != 0) {
      visual_array = link->visual_array;
    }
    else if (link->visual.get() != NULL) {
      visual_array.push_back(link->visual);
    }
    for(size_t i=0; i<visual_array.size(); i++) {
      VisualSharedPtr link_visual = visual_array[i];
      if (link_visual.get() != NULL && link_visual->geometry.get() != NULL) {
        visualization_msgs::msg::InteractiveMarkerControl meshControl;
        if (link_visual->geometry->type == Geometry::MESH) {
          MeshConstSharedPtr mesh = std::static_pointer_cast<const Mesh>(link_visual->geometry);
          string model_mesh_ = mesh->filename;
          if (linkMarkerMap[link_frame_name_].mesh_file == "") {
            model_mesh_ = getRosPathFromModelPath(model_mesh_);
            linkMarkerMap[link_frame_name_].mesh_file = model_mesh_;
          }
          else {
            model_mesh_ = linkMarkerMap[link_frame_name_].mesh_file;
          }
          ps.pose = UrdfPose2Pose(link_visual->origin);
          RCLCPP_DEBUG_STREAM(node_->get_logger(), "mesh_file:" << model_mesh_);
          geometry_msgs::msg::Vector3 mesh_scale;
          mesh_scale.x = mesh->scale.x;
          mesh_scale.y = mesh->scale.y;
          mesh_scale.z = mesh->scale.z;
          RCLCPP_INFO(node_->get_logger(), "make mesh marker from %s", model_mesh_.c_str());
          if (use_color) {
            meshControl = makeMeshMarkerControl(model_mesh_, ps, mesh_scale, color);
          }
          else {
            meshControl = makeMeshMarkerControl(model_mesh_, ps, mesh_scale);
          }
        }
        else if (link_visual->geometry->type == Geometry::CYLINDER) {
          CylinderConstSharedPtr cylinder = std::static_pointer_cast<const Cylinder>(link_visual->geometry);
          ps.pose = UrdfPose2Pose(link_visual->origin);
          double length = cylinder->length;
          double radius = cylinder->radius;
          RCLCPP_INFO_STREAM(node_->get_logger(), "cylinder " << link->name << ", length =  " << length << ", radius " << radius);
          if (use_color) {
            meshControl = makeCylinderMarkerControl(ps, length, radius, color, true);
          }
          else {
            meshControl = makeCylinderMarkerControl(ps, length, radius, color, true);
          }
        }
        else if (link_visual->geometry->type == Geometry::BOX) {
          BoxConstSharedPtr box = std::static_pointer_cast<const Box>(link_visual->geometry);
          ps.pose = UrdfPose2Pose(link_visual->origin);
          Vector3 dim = box->dim;
          RCLCPP_INFO_STREAM(node_->get_logger(), "box " << link->name << ", dim =  " << dim.x << ", " << dim.y << ", " << dim.z);
          if (use_color) {
            meshControl = makeBoxMarkerControl(ps, dim, color, true);
          }
          else {
            meshControl = makeBoxMarkerControl(ps, dim, color, true);
          }
        }
        else if (link_visual->geometry->type == Geometry::SPHERE) {
          SphereConstSharedPtr sphere = std::static_pointer_cast<const Sphere>(link_visual->geometry);
          ps.pose = UrdfPose2Pose(link_visual->origin);
          double rad = sphere->radius;
          if (use_color) {
            meshControl = makeSphereMarkerControl(ps, rad, color, true);
          }
          else {
            meshControl = makeSphereMarkerControl(ps, rad, color, true);
          }
        }
        int_marker.controls.push_back(meshControl);

        server_->insert(int_marker);
        server_->setCallback(int_marker.name,
                             [this, parent_link_frame_name_, link_frame_name_](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
                               proc_feedback(feedback, parent_link_frame_name_, link_frame_name_);
                             });

        model_menu_.apply(*server_, link_frame_name_);

      }
      else {
        JointSharedPtr parent_joint = link->parent_joint;
        if (parent_joint != NULL) {
          if (parent_joint->type==Joint::REVOLUTE || parent_joint->type==Joint::REVOLUTE) {
            addInvisibleMeshMarkerControl(int_marker, link, color);
            server_->insert(int_marker);
            server_->setCallback(int_marker.name,
                                 [this, parent_link_frame_name_, link_frame_name_](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
                                   proc_feedback(feedback, parent_link_frame_name_, link_frame_name_);
                                 });
            model_menu_.apply(*server_, link_frame_name_);
          }
        }
      }
    }
    if (!robot_mode_) {
      //add Grasp Point Marker
      if (linkMarkerMap[link_frame_name_].gp.displayGraspPoint) {
        visualization_msgs::msg::InteractiveMarker grasp_int_marker;
        double grasp_scale_factor = 1.02;
        string grasp_link_frame_name_ = tf_prefix_ + link->name + "/grasp";
        string grasp_parent_link_frame_name_ = tf_prefix_ + link->name;

        geometry_msgs::msg::PoseStamped grasp_ps;
        grasp_ps.pose = linkMarkerMap[link_frame_name_].gp.pose;
        grasp_ps.header.frame_id =  stripSlash(grasp_parent_link_frame_name_);

        grasp_int_marker.header = grasp_ps.header;
        grasp_int_marker.name = grasp_link_frame_name_;
        grasp_int_marker.scale = grasp_scale_factor;
        grasp_int_marker.pose = grasp_ps.pose;

        addGraspPointControl(grasp_int_marker, link_frame_name_);

        server_->insert(grasp_int_marker);
        server_->setCallback(grasp_int_marker.name,
                             [this, link_frame_name_](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
                               graspPoint_feedback(feedback, link_frame_name_);
                             });

      }
    }
  }

  //initialize JointState
  if (init) {
    if (!root && initial_pose_map_.count(link->parent_joint->name) != 0) {
      setJointAngle(link, initial_pose_map_[link->parent_joint->name]);
    }
  }

  for (std::vector<LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
    addChildLinkNames(*child, false, init, use_color, color_index + 1);
  }
  if (root) {
    server_->applyChanges();
  }
}



UrdfModelMarker::UrdfModelMarker ()
{}

UrdfModelMarker::UrdfModelMarker (string model_name, string model_description, string model_file, string frame_id, geometry_msgs::msg::PoseStamped root_pose, geometry_msgs::msg::Pose root_offset, double scale_factor, string mode, bool robot_mode, bool registration, vector<string> fixed_link, bool use_robot_description, bool use_visible_color, map<string, double> initial_pose_map, int index, rclcpp::Node::SharedPtr node, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : node_(node), use_dynamic_tf_(true), is_joint_states_locked_(false) {
  tf_buffer_.reset(new tf2_ros::Buffer(node_->get_clock()));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(node_));

  if (!node_->has_parameter("server_name")) {
    node_->declare_parameter("server_name", std::string(""));
  }
  node_->get_parameter("server_name", server_name);

  if (server_name == "") {
    server_name = node_->get_name();
  }

  if (!node_->has_parameter("use_dynamic_tf")) {
    node_->declare_parameter("use_dynamic_tf", true);
  }
  node_->get_parameter("use_dynamic_tf", use_dynamic_tf_);
  // dynamic_tf_publisher replacement: broadcast the registered transforms
  // periodically (the ROS 1 code requested freq=20 in SetDynamicTF)
  dynamic_tf_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / 20.0),
    std::bind(&UrdfModelMarker::publishDynamicTf, this));
  RCLCPP_INFO_STREAM(node_->get_logger(), "use_dynamic_tf_ is " << use_dynamic_tf_);

  if (index != -1) {
    stringstream ss;
    ss << model_name << index;
    model_name_ = ss.str();
  }
  else {
    model_name_ = model_name;
  }

  model_description_ = model_description;
  server_ = server;
  model_file_ = model_file;
  frame_id_ = frame_id;
  root_offset_ = root_offset;
  fixed_link_offset_ = root_offset;
  root_pose_ = root_pose.pose;
  init_stamp_ = root_pose.header.stamp;
  scale_factor_ = scale_factor;
  robot_mode_ = robot_mode;
  registration_ = registration;
  mode_ = mode;
  fixed_link_ = fixed_link;
  use_robot_description_ = use_robot_description;
  use_visible_color_ = use_visible_color;
  tf_prefix_ = server_name + "/" + model_name_ + "/";
  initial_pose_map_ = initial_pose_map;
  index_ = index;

  pub_ = node_->create_publisher<jsk_interactive_marker_msgs::msg::MarkerPose>("~/pose", 1);
  pub_move_ = node_->create_publisher<jsk_interactive_marker_msgs::msg::MarkerMenu>("~/marker_menu", 1);
  pub_move_object_ = node_->create_publisher<jsk_interactive_marker_msgs::msg::MoveObject>("~/move_object", 1);
  pub_move_model_ = node_->create_publisher<jsk_interactive_marker_msgs::msg::MoveModel>("~/move_model", 1);
  pub_selected_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("~/" + model_name + "/selected", 1);
  pub_selected_index_ = node_->create_publisher<jsk_recognition_msgs::msg::Int32Stamped>("~/" + model_name + "/selected_index", 1);
  pub_joint_state_ = node_->create_publisher<sensor_msgs::msg::JointState>("~/" + model_name_ + "/joint_states", 1);

  sub_set_root_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/" + model_name_ + "/set_pose", 1,
    std::bind(&UrdfModelMarker::setRootPoseCB, this, std::placeholders::_1));
  sub_reset_joints_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "~/" + model_name_ + "/reset_joint_states", 1,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { resetJointStatesCB(msg, false); });
  sub_reset_joints_and_root_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "~/" + model_name_ + "/reset_joint_states_and_root", 1,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { resetJointStatesCB(msg, true); });

  hide_marker_ = node_->create_subscription<std_msgs::msg::Empty>(
    "~/" + model_name_ + "/hide_marker", 1,
    std::bind(&UrdfModelMarker::hideModelMarkerCB, this, std::placeholders::_1));
  show_marker_ = node_->create_subscription<std_msgs::msg::Empty>(
    "~/" + model_name_ + "/show_marker", 1,
    std::bind(&UrdfModelMarker::showModelMarkerCB, this, std::placeholders::_1));
  sub_set_urdf_ = node_->create_subscription<std_msgs::msg::String>(
    "~/" + model_name_ + "/set_urdf", 1,
    std::bind(&UrdfModelMarker::setUrdfCB, this, std::placeholders::_1));

  sub_reset_root_pose_ = node_->create_subscription<std_msgs::msg::Empty>(
    "~/" + model_name_ + "/reset_root_pose", 1,
    std::bind(&UrdfModelMarker::resetBaseMsgCB, this, std::placeholders::_1));

  pub_base_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("~/" + model_name_ + "/base_pose", 1);

  if (mode_ == "") {
    if (registration_) {
      mode_ = "registration";
    }
    else if (robot_mode_) {
      mode_ = "robot";
    }
    else {
      mode_ = "model";
    }
  }
  serv_lock_joint_states_ = node_->create_service<std_srvs::srv::Empty>(
    "~/lock_joint_states",
    std::bind(&UrdfModelMarker::lockJointStates, this,
              std::placeholders::_1, std::placeholders::_2));
  serv_unlock_joint_states_ = node_->create_service<std_srvs::srv::Empty>(
    "~/unlock_joint_states",
    std::bind(&UrdfModelMarker::unlockJointStates, this,
              std::placeholders::_1, std::placeholders::_2));

  if (mode_ == "registration") {
    model_menu_.insert("Registration",
                       std::bind(&UrdfModelMarker::registrationCB, this, std::placeholders::_1));
  }
  else if (mode_ == "visualization") {

  }
  else if (mode_ == "robot") {
    interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
    sub_menu_move_ = model_menu_.insert("Move");
    model_menu_.insert(sub_menu_move_, "Joint",
                       std::bind(&UrdfModelMarker::jointMoveCB, this, std::placeholders::_1));
    model_menu_.insert(sub_menu_move_, "Base",
                       [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
                         publishMarkerMenu(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE);
                       });

    interactive_markers::MenuHandler::EntryHandle sub_menu_reset_;
    sub_menu_reset_ = model_menu_.insert("Reset Marker Pose");
    model_menu_.insert(sub_menu_reset_, "Joint",
                       std::bind(&UrdfModelMarker::resetMarkerCB, this, std::placeholders::_1));
    model_menu_.insert(sub_menu_reset_, "Base",
                       std::bind(&UrdfModelMarker::resetBaseMarkerCB, this, std::placeholders::_1));


    interactive_markers::MenuHandler::EntryHandle sub_menu_pose_;
    sub_menu_pose_ = model_menu_.insert("Special Pose");

    model_menu_.insert(sub_menu_pose_, "Stand Pose",
                       [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
                         publishMarkerMenu(feedback, 100);
                       });

    model_menu_.insert(sub_menu_pose_, "Manip Pose",
                       [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
                         publishMarkerMenu(feedback, 101);
                       });

    model_menu_.insert("Hide Marker" ,
                       std::bind(&UrdfModelMarker::hideMarkerCB, this, std::placeholders::_1));
    model_menu_.insert("Hide All Marker" ,
                       std::bind(&UrdfModelMarker::hideAllMarkerCB, this, std::placeholders::_1));


  }
  else if (mode_ == "model") {
    model_menu_.insert("Grasp Point",
                       std::bind(&UrdfModelMarker::graspPointCB, this, std::placeholders::_1));
    model_menu_.insert("Move",
                       std::bind(&UrdfModelMarker::moveCB, this, std::placeholders::_1));
    model_menu_.insert("Set as present pose",
                       [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
                         setPoseCB(feedback);
                       });
    model_menu_.insert("Hide Marker" ,
                       std::bind(&UrdfModelMarker::hideMarkerCB, this, std::placeholders::_1));
    model_menu_.insert("Hide All Marker" ,
                       std::bind(&UrdfModelMarker::hideAllMarkerCB, this, std::placeholders::_1));
  }

  // get the entire file
  std::string xml_string;

  if (use_robot_description_) {
    // read the URDF from a node parameter (rviz2/robot_state_publisher convention)
    std::string param_name = stripSlash(model_file_);
    RCLCPP_INFO(node_->get_logger(), "loading robot_description from parameter %s", param_name.c_str());
    if (!node_->has_parameter(param_name)) {
      node_->declare_parameter(param_name, std::string(""));
    }
    node_->get_parameter(param_name, xml_string);

  }
  else {
    RCLCPP_INFO_STREAM(node_->get_logger(), "loading model_file: " << model_file_);
    model_file_ = getFilePathFromRosPath(model_file_);
    model_file_ = getFullPathFromModelPath(model_file_);
    try {
      if (!std::filesystem::exists(model_file_.c_str())) {
        RCLCPP_ERROR(node_->get_logger(), "%s does not exists", model_file_.c_str());
      }
      else {
        std::fstream xml_file(model_file_.c_str(), std::fstream::in);
        while (xml_file.good())
        {
          std::string line;
          std::getline(xml_file, line);
          xml_string += (line + "\n");
        }
        xml_file.close();
        RCLCPP_INFO_STREAM(node_->get_logger(), "finish loading model_file: " << model_file_);
      }
    }
    catch (...) {
      RCLCPP_ERROR(node_->get_logger(), "model or mesh not found: %s", model_file_.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Please check GAZEBO_MODEL_PATH");
    }
  }
  RCLCPP_INFO(node_->get_logger(), "xml_string is %lu size", xml_string.length());
  model = parseURDF(xml_string);

  if (!model) {
    RCLCPP_ERROR(node_->get_logger(), "ERROR: Model Parsing the xml failed");
    return;
  }
  else {
    RCLCPP_INFO(node_->get_logger(), "model name is %s", model->getName().c_str());
  }
  if (mode_ == "robot") {
    resetRobotBase();
  }
  addChildLinkNames(model->getRoot(), true, true);

  publishJointState();
  setPoseCB(); //init joint_state_origin

  resetRootForVisualization();
  return;
}

void UrdfModelMarker::lockJointStates(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                      std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  std::lock_guard<std::mutex> lock(joint_states_mutex_);
  is_joint_states_locked_ = true;
}

void UrdfModelMarker::unlockJointStates(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                        std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  std::lock_guard<std::mutex> lock(joint_states_mutex_);
  is_joint_states_locked_ = false;
}
