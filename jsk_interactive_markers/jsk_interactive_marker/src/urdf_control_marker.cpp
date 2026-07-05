#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>

#include <map>
#include <mutex>
#include <string>

using namespace visualization_msgs;

namespace {
std::string stripSlash(const std::string &frame)
{
  if (!frame.empty() && frame[0] == '/') {
    return frame.substr(1);
  }
  return frame;
}
}

class UrdfControlMarker : public rclcpp::Node {
public:
  UrdfControlMarker();
  void processFeedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void makeControlMarker( bool fixed );
  void set_pose_cb( const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg );
  void show_marker_cb ( const std_msgs::msg::Bool::ConstSharedPtr msg);
  void markerUpdate ( std_msgs::msg::Header header, geometry_msgs::msg::Pose pose);
  void publish_pose_cb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  // dynamic_tf_publisher replacement
  void setDynamicTf(const std_msgs::msg::Header& header,
                    const std::string& child_frame,
                    const geometry_msgs::msg::Transform& transform);
  void publishDynamicTf();
private:
  bool use_dynamic_tf_, move_2d_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> dynamic_tf_map_;
  std::mutex dynamic_tf_mutex_;
  rclcpp::TimerBase::SharedPtr dynamic_tf_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_set_pose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_show_marker_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_, pub_selected_pose_;
  std::string frame_id_, marker_frame_id_, fixed_frame_id_;
  std::string center_marker_;
  std_msgs::msg::ColorRGBA color_;
  bool mesh_use_embedded_materials_;
  double marker_scale_, center_marker_scale_;
  interactive_markers::MenuHandler marker_menu_;
  geometry_msgs::msg::Pose center_marker_pose_;
};

UrdfControlMarker::UrdfControlMarker() : rclcpp::Node("urdf_control_marker") {
  server_.reset(new interactive_markers::InteractiveMarkerServer("urdf_control_marker", this));

  move_2d_ = this->declare_parameter("move_2d", false);
  use_dynamic_tf_ = this->declare_parameter("use_dynamic_tf", false);
  frame_id_ = stripSlash(this->declare_parameter("frame_id", std::string("map")));
  fixed_frame_id_ = stripSlash(this->declare_parameter("fixed_frame_id", std::string("odom_on_ground")));
  marker_frame_id_ = stripSlash(this->declare_parameter("marker_frame_id", std::string("urdf_control_marker")));
  center_marker_ = this->declare_parameter("center_marker", std::string(""));
  marker_scale_ = this->declare_parameter("marker_scale", 1.0);
  center_marker_scale_ = this->declare_parameter("center_marker_scale", 1.0);
  //set color
  bool use_center_marker_color =
    this->declare_parameter("center_marker_color", false);
  if (use_center_marker_color) {
    mesh_use_embedded_materials_ = false;
    color_.r = this->declare_parameter("color.r", 0.0);
    color_.g = this->declare_parameter("color.g", 0.0);
    color_.b = this->declare_parameter("color.b", 0.0);
    color_.a = this->declare_parameter("color.a", 0.0);
  }else{
    mesh_use_embedded_materials_ = true;
  }

  //set pose
  // center_marker_pose is passed as a YAML string, e.g.
  //   center_marker_pose: "{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}"
  std::string center_marker_pose_yaml =
    this->declare_parameter("center_marker_pose", std::string(""));
  center_marker_pose_.orientation.w = 1.0;
  if (!center_marker_pose_yaml.empty()) {
    try {
      YAML::Node pose_v = YAML::Load(center_marker_pose_yaml);
      center_marker_pose_ = im_utils::getPose(pose_v);
    }
    catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "failed to parse ~center_marker_pose: %s", e.what());
    }
  }

  tf_buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  //dynamic_tf_publisher replacement: broadcast registered transforms
  //periodically (the ROS 1 code requested freq=20 in SetDynamicTF)
  if (use_dynamic_tf_) {
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(this));
    dynamic_tf_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 20.0),
      std::bind(&UrdfControlMarker::publishDynamicTf, this));
  }
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 1);
  pub_selected_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/selected_pose", 1);
  sub_set_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/set_pose", 1,
    std::bind(&UrdfControlMarker::set_pose_cb, this, std::placeholders::_1));
  sub_show_marker_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/show_marker", 1,
    std::bind(&UrdfControlMarker::show_marker_cb, this, std::placeholders::_1));

  marker_menu_.insert("Publish Pose",
                      std::bind(&UrdfControlMarker::publish_pose_cb, this, std::placeholders::_1));

  makeControlMarker( false );
}

void UrdfControlMarker::publish_pose_cb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  geometry_msgs::msg::PoseStamped ps;
  ps.header = feedback->header;
  ps.pose = feedback->pose;
  pub_selected_pose_->publish(ps);
}

void UrdfControlMarker::set_pose_cb ( const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
  // Convert PoseStamped frame_id to fixed_frame_id_
  geometry_msgs::msg::PoseStamped in_pose(*msg);
  geometry_msgs::msg::PoseStamped out_pose;
  in_pose.header.stamp = builtin_interfaces::msg::Time();
  in_pose.header.frame_id = stripSlash(in_pose.header.frame_id);
  try {
    out_pose = tf_buffer_->transform(in_pose, fixed_frame_id_);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform pose: %s", ex.what());
    return;
  }
  out_pose.header.stamp = msg->header.stamp;
  server_->setPose("urdf_control_marker", out_pose.pose, out_pose.header);
  server_->applyChanges();
  markerUpdate(out_pose.header, out_pose.pose);
}

void UrdfControlMarker::show_marker_cb ( const std_msgs::msg::Bool::ConstSharedPtr msg){
  if(msg->data){
    makeControlMarker( false );
  }else{
    server_->clear();
    server_->applyChanges();
  }
}


void UrdfControlMarker::processFeedback( visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback )
{
  markerUpdate( feedback->header, feedback->pose);
}

// replacement of the dynamic_tf_publisher SetDynamicTF service call:
// register/update the transform which is broadcast by dynamic_tf_timer_.
void UrdfControlMarker::setDynamicTf(
  const std_msgs::msg::Header& header,
  const std::string& child_frame,
  const geometry_msgs::msg::Transform& transform)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header = header;
  tf_stamped.header.frame_id = stripSlash(header.frame_id);
  tf_stamped.child_frame_id = stripSlash(child_frame);
  tf_stamped.transform = transform;
  std::lock_guard<std::mutex> lock(dynamic_tf_mutex_);
  dynamic_tf_map_[tf_stamped.child_frame_id] = tf_stamped;
}

void UrdfControlMarker::publishDynamicTf()
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  {
    std::lock_guard<std::mutex> lock(dynamic_tf_mutex_);
    rclcpp::Time now = this->now();
    for (auto &it : dynamic_tf_map_) {
      it.second.header.stamp = now;
      transforms.push_back(it.second);
    }
  }
  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

void UrdfControlMarker::markerUpdate ( std_msgs::msg::Header header, geometry_msgs::msg::Pose pose){
  if (use_dynamic_tf_){
    geometry_msgs::msg::Transform transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation = pose.orientation;
    setDynamicTf(header, marker_frame_id_, transform);
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  ps.pose = pose;
  pub_pose_->publish(ps);
}




void UrdfControlMarker::makeControlMarker( bool fixed )
{
  msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.scale = marker_scale_;

  int_marker.name = "urdf_control_marker";

  //add center marker
  if(center_marker_ != ""){
    msg::InteractiveMarkerControl center_marker_control;
    center_marker_control.name = "center_marker";
    center_marker_control.always_visible = true;
    center_marker_control.orientation.w = 1.0;
    center_marker_control.orientation.y = 1.0;

    if(move_2d_){
      center_marker_control.interaction_mode = msg::InteractiveMarkerControl::MOVE_PLANE;
    }else{
      center_marker_control.interaction_mode = msg::InteractiveMarkerControl::MOVE_3D;
    }
    msg::Marker center_marker;
    center_marker.type = msg::Marker::MESH_RESOURCE;
    center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = center_marker_scale_;
    center_marker.mesh_use_embedded_materials = mesh_use_embedded_materials_;
    center_marker.mesh_resource = center_marker_;
    center_marker.pose = center_marker_pose_;
    center_marker.color = color_;

    center_marker_control.markers.push_back(center_marker);
    int_marker.controls.push_back(center_marker_control);
  }

  msg::InteractiveMarkerControl control;

  if ( fixed )
    {
      int_marker.name += "_fixed";
      control.orientation_mode = msg::InteractiveMarkerControl::FIXED;
    }
  control.always_visible = true;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = msg::InteractiveMarkerControl::ROTATE_AXIS;
  if(!move_2d_){
    int_marker.controls.push_back(control);
  }
  control.name = "move_x";
  control.interaction_mode = msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = msg::InteractiveMarkerControl::MOVE_AXIS;
  if(!move_2d_){
    int_marker.controls.push_back(control);
  }

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = msg::InteractiveMarkerControl::ROTATE_AXIS;
  if(!move_2d_){
    int_marker.controls.push_back(control);
  }
  control.name = "move_y";
  control.interaction_mode = msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&UrdfControlMarker::processFeedback, this, std::placeholders::_1));
  marker_menu_.apply(*server_, int_marker.name);
  server_->applyChanges();
  if (use_dynamic_tf_) {
    /* First initialize dynamic tf as identity */
    std_msgs::msg::Header header;
    header.frame_id = fixed_frame_id_;
    header.stamp = this->now();
    geometry_msgs::msg::Transform transform;
    transform.rotation.w = 1.0;
    setDynamicTf(header, marker_frame_id_, transform);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UrdfControlMarker>());
  rclcpp::shutdown();
  return 0;
}
