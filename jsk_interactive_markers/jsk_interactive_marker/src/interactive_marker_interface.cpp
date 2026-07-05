#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.hpp>
#include <jsk_interactive_marker_msgs/srv/set_pose.hpp>
#include <jsk_interactive_marker_msgs/srv/marker_set_pose.hpp>

#include <math.h>
#include <jsk_interactive_marker_msgs/msg/marker_menu.hpp>
#include <jsk_interactive_marker_msgs/msg/marker_pose.hpp>

#include <std_msgs/msg/int8.hpp>

#include <jsk_interactive_marker/interactive_marker_interface.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>

#include <kdl/frames_io.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <fstream>

using namespace im_utils;

namespace {
// tf2 does not accept frame_ids with a leading slash
std::string stripSlash(const std::string &frame)
{
  if (!frame.empty() && frame[0] == '/') {
    return frame.substr(1);
  }
  return frame;
}
}

visualization_msgs::msg::InteractiveMarker InteractiveMarkerInterface::make6DofControlMarker( std::string name, geometry_msgs::msg::PoseStamped &stamped, float scale, bool fixed_position, bool fixed_rotation){

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header =  stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;

  visualization_msgs::msg::InteractiveMarkerControl control;

  //x axis
  if(fixed_rotation){
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  if(fixed_position){
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  }

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);


  //y axis
  if(fixed_rotation){
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  }
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  if(fixed_position){
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  }

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  //z axis
  if(fixed_rotation){
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  }

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  if(fixed_position){
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }else{
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
  }
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  return int_marker;
}



void InteractiveMarkerInterface::proc_feedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ) {
  if(feedback->control_name.find("center_sphere") != std::string::npos){
    proc_feedback(feedback, jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_SPHERE_MARKER);
  }else{
    proc_feedback(feedback, jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_GENERAL);
  }

}

void InteractiveMarkerInterface::proc_feedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int type ) {

  jsk_interactive_marker_msgs::msg::MarkerPose mp;
  mp.pose.header = feedback->header;
  mp.pose.pose = feedback->pose;
  mp.marker_name = feedback->marker_name;
  mp.type = type;
  pub_->publish( mp );

  //update Marker Pose Status
  control_state_.marker_pose_.pose = feedback->pose;
  control_state_.marker_pose_.header = feedback->header;

  pub_marker_tf(feedback->header, feedback->pose);

}

// dynamic_tf_publisher SetDynamicTF replacement: register/update the
// "moving_marker" transform, broadcast periodically at 10 Hz (the freq the
// ROS 1 code passed in the service request).
void InteractiveMarkerInterface::pub_marker_tf ( std_msgs::msg::Header header, geometry_msgs::msg::Pose pose){
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.header.stamp = this->now();
  tf_stamped.header.frame_id = stripSlash(header.frame_id);
  tf_stamped.child_frame_id = "moving_marker";
  tf_stamped.transform.translation.x = pose.position.x;
  tf_stamped.transform.translation.y = pose.position.y;
  tf_stamped.transform.translation.z = pose.position.z;
  tf_stamped.transform.rotation = pose.orientation;

  std::lock_guard<std::mutex> lock(dynamic_tf_mutex_);
  dynamic_tf_map_[tf_stamped.child_frame_id] = tf_stamped;
}

void InteractiveMarkerInterface::publishDynamicTf(){
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

void InteractiveMarkerInterface::pub_marker_pose ( std_msgs::msg::Header header, geometry_msgs::msg::Pose pose, std::string name, int type ) {
  jsk_interactive_marker_msgs::msg::MarkerPose mp;
  mp.pose.header = header;
  mp.pose.pose = pose;
  mp.marker_name = name;
  mp.type = type;
  pub_->publish( mp );
}




void InteractiveMarkerInterface::pub_marker_menuCb(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int menu){
  jsk_interactive_marker_msgs::msg::MarkerMenu m;
  m.marker_name = feedback->marker_name;
  m.menu=menu;
  pub_move_->publish(m);
}

void InteractiveMarkerInterface::pub_marker_menuCb(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int menu, int type){
  jsk_interactive_marker_msgs::msg::MarkerMenu m;
  m.marker_name = feedback->marker_name;
  m.menu = menu;
  m.type = type;
  pub_move_->publish(m);
}


void InteractiveMarkerInterface::pub_marker_menu(std::string marker, int menu, int type){
  jsk_interactive_marker_msgs::msg::MarkerMenu m;
  m.marker_name = marker;
  m.menu=menu;
  m.type = type;
  pub_move_->publish(m);
}

void InteractiveMarkerInterface::pub_marker_menu(std::string marker, int menu){
  pub_marker_menu(marker, menu, 0);
}

void InteractiveMarkerInterface::IMSizeLargeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  geometry_msgs::msg::PoseStamped pose;
  pose.header = feedback->header;
  pose.pose = feedback->pose;
  changeMarkerMoveMode(feedback->marker_name, 0, 0.5, pose);
}

void InteractiveMarkerInterface::IMSizeMiddleCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  geometry_msgs::msg::PoseStamped pose;
  pose.header = feedback->header;
  pose.pose = feedback->pose;
  changeMarkerMoveMode(feedback->marker_name, 0, 0.3, pose);
}

void InteractiveMarkerInterface::IMSizeSmallCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  geometry_msgs::msg::PoseStamped pose;
  pose.header = feedback->header;
  pose.pose = feedback->pose;
  changeMarkerMoveMode(feedback->marker_name, 0, 0.1, pose);
}

void InteractiveMarkerInterface::changeMoveModeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  changeMarkerMoveMode(feedback->marker_name,0);
  pub_marker_menu(feedback->marker_name,13);

}
void InteractiveMarkerInterface::changeMoveModeCb1( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  changeMarkerMoveMode(feedback->marker_name,1);
  pub_marker_menu(feedback->marker_name,13);

}
void InteractiveMarkerInterface::changeMoveModeCb2( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  changeMarkerMoveMode(feedback->marker_name,2);
  pub_marker_menu(feedback->marker_name,13);
}

void InteractiveMarkerInterface::changeForceModeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  RCLCPP_INFO(this->get_logger(), "%s changeForceMode",feedback->marker_name.c_str());
  changeMarkerForceMode(feedback->marker_name,0);
  pub_marker_menu(feedback->marker_name,12);

}
void InteractiveMarkerInterface::changeForceModeCb1( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  RCLCPP_INFO(this->get_logger(), "%s changeForceMode1",feedback->marker_name.c_str());
  changeMarkerForceMode(feedback->marker_name,1);
  pub_marker_menu(feedback->marker_name,12);

}
void InteractiveMarkerInterface::changeForceModeCb2( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  RCLCPP_INFO(this->get_logger(), "%s changeForceMode2",feedback->marker_name.c_str());
  changeMarkerForceMode(feedback->marker_name,2);
  pub_marker_menu(feedback->marker_name,12);
}

void InteractiveMarkerInterface::targetPointMenuCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  RCLCPP_INFO(this->get_logger(), "targetPointMenu callback");

  control_state_.head_on_ ^= true;
  control_state_.init_head_goal_ = true;

  if(control_state_.head_on_){
    menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::CHECKED);
    control_state_.look_auto_on_ = false;
    menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::UNCHECKED);
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::HEAD_TARGET_POINT, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_HEAD_MARKER);
  }else{
    menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::UNCHECKED);
  }
  menu_head_.reApply(*server_);

  initControlMarkers();
}

void InteractiveMarkerInterface::lookAutomaticallyMenuCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  RCLCPP_INFO(this->get_logger(), "targetPointMenu callback");

  control_state_.head_on_ = false;
  control_state_.look_auto_on_ ^= true;
  control_state_.init_head_goal_ = true;

  if(control_state_.look_auto_on_){
    menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::CHECKED);
    menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::UNCHECKED);
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::PUBLISH_MARKER, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_HEAD_MARKER);
  }else{
    menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::UNCHECKED);
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::HEAD_TARGET_POINT, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_HEAD_MARKER);
  }
  menu_head_.reApply(*server_);
  initControlMarkers();
}


void InteractiveMarkerInterface::ConstraintCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  menu_handler.setCheckState( h_mode_last2, interactive_markers::MenuHandler::UNCHECKED );
  h_mode_last2 = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last2, interactive_markers::MenuHandler::CHECKED );

  switch(h_mode_last2-h_mode_constrained){
  case 0:
    pub_marker_menu(feedback->marker_name,jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE_CONSTRAINT_T);
    RCLCPP_INFO(this->get_logger(), "send 23");
    break;
  case 1:
    pub_marker_menu(feedback->marker_name,jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE_CONSTRAINT_NIL);
    RCLCPP_INFO(this->get_logger(), "send 24");
    break;
  default:
    RCLCPP_INFO(this->get_logger(), "Switching Arm Error");
    break;
  }

  menu_handler.reApply( *server_ );
  server_->applyChanges();

}

void InteractiveMarkerInterface::useTorsoCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if(feedback->menu_entry_id == use_torso_t_menu_){
    menu_handler.setCheckState( use_torso_t_menu_ , interactive_markers::MenuHandler::CHECKED );
    menu_handler.setCheckState( use_torso_nil_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_fullbody_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::USE_TORSO_T);
  }else if(feedback->menu_entry_id == use_torso_nil_menu_){
    menu_handler.setCheckState( use_torso_t_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_torso_nil_menu_ , interactive_markers::MenuHandler::CHECKED );
    menu_handler.setCheckState( use_fullbody_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::USE_TORSO_NIL);
  }else{
    menu_handler.setCheckState( use_torso_t_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_torso_nil_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( use_fullbody_menu_ , interactive_markers::MenuHandler::CHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::USE_FULLBODY);
  }
  menu_handler.reApply( *server_ );
  server_->applyChanges();

}

void InteractiveMarkerInterface::usingIKCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if(feedback->menu_entry_id == start_ik_menu_){
    menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
    menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::PLAN);
  }else if(feedback->menu_entry_id == stop_ik_menu_){
    menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
    menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::CANCEL_PLAN);
  }
  menu_handler.reApply( *server_ );
  server_->applyChanges();

}

void InteractiveMarkerInterface::toggleStartIKCb( const std_msgs::msg::Empty::ConstSharedPtr &msg)
{
  (void)msg;
  interactive_markers::MenuHandler::CheckState check_state;
  if(menu_handler.getCheckState( start_ik_menu_ , check_state)){

    if(check_state == interactive_markers::MenuHandler::CHECKED){
      //stop ik
      menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
      pub_marker_menu("", jsk_interactive_marker_msgs::msg::MarkerMenu::CANCEL_PLAN);

    }else{
      //start ik
      menu_handler.setCheckState( start_ik_menu_ , interactive_markers::MenuHandler::CHECKED );
      menu_handler.setCheckState( stop_ik_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      pub_marker_menu("" , jsk_interactive_marker_msgs::msg::MarkerMenu::PLAN);
    }

    menu_handler.reApply( *server_ );
    server_->applyChanges();
  }
}


void InteractiveMarkerInterface::modeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
  h_mode_last = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::CHECKED );

  switch(h_mode_last - h_mode_rightarm){
  case 0:
    changeMoveArm( feedback->marker_name, jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_RARM);
    break;
  case 1:
    changeMoveArm( feedback->marker_name, jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_LARM);
    break;
  case 2:
    changeMoveArm( feedback->marker_name, jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_ARMS);
    break;
  default:
    RCLCPP_INFO(this->get_logger(), "Switching Arm Error");
    break;
  }
  menu_handler.reApply( *server_ );
  server_->applyChanges();
}

void InteractiveMarkerInterface::changeMoveArm( std::string m_name, int menu ){
  switch(menu){
  case jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_RARM:
    pub_marker_menu(m_name,jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_RARM);
    control_state_.move_arm_ = ControlState::RARM;
    RCLCPP_INFO(this->get_logger(), "move Rarm");
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    break;
  case jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_LARM:
    pub_marker_menu(m_name,jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_LARM);
    control_state_.move_arm_ = ControlState::LARM;
    RCLCPP_INFO(this->get_logger(), "move Larm");
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    break;
  case jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_ARMS:
    pub_marker_menu(m_name,jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_ARMS);
    control_state_.move_arm_ = ControlState::ARMS;
    RCLCPP_INFO(this->get_logger(), "move Arms");
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    break;
  default:
    RCLCPP_INFO(this->get_logger(), "Switching Arm Error");
    break;
  }
}

void InteractiveMarkerInterface::setOriginCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback,  bool origin_hand){
  if(origin_hand){
    control_state_.move_origin_state_ = ControlState::HAND_ORIGIN;
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    if(control_state_.move_arm_ == ControlState::RARM){
      pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::SET_ORIGIN_RHAND);}else{
      pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::SET_ORIGIN_LHAND);}
  }else{
    control_state_.move_origin_state_ = ControlState::DESIGNATED_ORIGIN;
    changeMarkerMoveMode( marker_name.c_str(), 0, 0.5, control_state_.marker_pose_);
    pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::SET_ORIGIN);
  }


}


void InteractiveMarkerInterface::ikmodeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if(feedback->menu_entry_id == rotation_t_menu_){
    menu_handler.setCheckState( rotation_nil_menu_, interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menu(feedback->marker_name,jsk_interactive_marker_msgs::msg::MarkerMenu::IK_ROTATION_AXIS_T);
    RCLCPP_INFO(this->get_logger(), "Rotation Axis T");
  }else{
    menu_handler.setCheckState( rotation_t_menu_, interactive_markers::MenuHandler::UNCHECKED );
    pub_marker_menu(feedback->marker_name ,jsk_interactive_marker_msgs::msg::MarkerMenu::IK_ROTATION_AXIS_NIL);
    RCLCPP_INFO(this->get_logger(), "Rotation Axis NIL");
  }


  menu_handler.setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );

  menu_handler.reApply( *server_ );
  server_->applyChanges();
}


void InteractiveMarkerInterface::toggleIKModeCb( const std_msgs::msg::Empty::ConstSharedPtr &msg)
{
  (void)msg;
  interactive_markers::MenuHandler::CheckState check_state;
  if(menu_handler.getCheckState( rotation_t_menu_ , check_state)){
    if(check_state == interactive_markers::MenuHandler::CHECKED){
      //rotation axis nil
      menu_handler.setCheckState( rotation_t_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      menu_handler.setCheckState( rotation_nil_menu_ , interactive_markers::MenuHandler::CHECKED );
      pub_marker_menu("", jsk_interactive_marker_msgs::msg::MarkerMenu::IK_ROTATION_AXIS_NIL);

    }else{
      //rotation_axis t
      menu_handler.setCheckState( rotation_t_menu_ , interactive_markers::MenuHandler::CHECKED );
      menu_handler.setCheckState( rotation_nil_menu_ , interactive_markers::MenuHandler::UNCHECKED );
      pub_marker_menu("" , jsk_interactive_marker_msgs::msg::MarkerMenu::IK_ROTATION_AXIS_T);
    }

    menu_handler.reApply( *server_ );
    server_->applyChanges();
  }
}

void InteractiveMarkerInterface::marker_menu_cb( const jsk_interactive_marker_msgs::msg::MarkerMenu::ConstSharedPtr msg){
  switch (msg->menu){
  case jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_RARM:
  case jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_LARM:
  case jsk_interactive_marker_msgs::msg::MarkerMenu::SET_MOVE_ARMS:
    changeMoveArm(msg->marker_name, msg->menu);
    break;
  case jsk_interactive_marker_msgs::msg::MarkerMenu::IK_ROTATION_AXIS_NIL:
  case jsk_interactive_marker_msgs::msg::MarkerMenu::IK_ROTATION_AXIS_T:
    {
      std_msgs::msg::Empty::ConstSharedPtr empty;
      toggleIKModeCb(empty);
    }
    break;
  case jsk_interactive_marker_msgs::msg::MarkerMenu::PLAN:
  case jsk_interactive_marker_msgs::msg::MarkerMenu::CANCEL_PLAN:
    {
      std_msgs::msg::Empty::ConstSharedPtr empty;
      toggleStartIKCb(empty);
    }
    break;
  default:
    pub_marker_menu(msg->marker_name , msg->menu, msg->type);
    break;
  }
}


void InteractiveMarkerInterface::updateHeadGoal( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      RCLCPP_INFO_STREAM(this->get_logger(), feedback->marker_name << " was clicked on." );
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      RCLCPP_INFO_STREAM(this->get_logger(), "Marker " << feedback->marker_name
                         << " control " << feedback->control_name
                         << " menu_entry_id " << feedback->menu_entry_id);
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      break;
    }
}

void InteractiveMarkerInterface::updateBase( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      proc_feedback(feedback, jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_BASE_MARKER);
      break;
    }
}

void InteractiveMarkerInterface::updateFinger( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string hand)
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
      if(hand == "rhand"){
        control_state_.r_finger_on_ ^= true;
      }else if(hand == "lhand"){
        control_state_.l_finger_on_ ^= true;
      }
      initControlMarkers();
      RCLCPP_INFO_STREAM(this->get_logger(), hand << feedback->marker_name << " was clicked on." );
      break;
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      RCLCPP_INFO_STREAM(this->get_logger(), hand << feedback->marker_name << " was clicked on." );
      if(hand == "rhand"){
        proc_feedback(feedback, jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_RFINGER_MARKER);
      }
      if(hand == "lhand"){
        proc_feedback(feedback, jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_LFINGER_MARKER);
      }

      break;
    }
}


//im_mode
//0:normal move  1:operationModel 2:operationalModelFirst
void InteractiveMarkerInterface::changeMarkerForceMode( std::string mk_name , int im_mode){
  RCLCPP_INFO(this->get_logger(), "changeMarkerForceMode  marker:%s  mode:%d\n",mk_name.c_str(),im_mode);
  interactive_markers::MenuHandler reset_handler;
  menu_handler_force = reset_handler;
  menu_handler_force1 = reset_handler;
  menu_handler_force2 = reset_handler;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = base_frame;
  if ( target_frame != "" ) {
  }
  visualization_msgs::msg::InteractiveMarker mk;
  mk.name = mk_name.c_str();
  mk.scale = 0.5;
  mk.header = pose.header;
  mk.pose = pose.pose;

  visualization_msgs::msg::InteractiveMarkerControl control;

  if ( false )
    {
      control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);

  switch(im_mode){
  case 0:
    menu_handler_force.insert("MoveMode",std::bind( &InteractiveMarkerInterface::changeMoveModeCb, this, std::placeholders::_1));
    menu_handler_force.insert("Delete Force",
                              [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                                pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::DELETE_FORCE);
                              });
    server_->insert( mk );
    server_->setCallback( mk.name,
                          [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });
    menu_handler_force.apply(*server_,mk.name);

    server_->applyChanges();
    break;
  case 1:
    mk.scale = 0.5;
    menu_handler_force1.insert("MoveMode",std::bind( &InteractiveMarkerInterface::changeMoveModeCb1, this, std::placeholders::_1));
    menu_handler_force1.insert("Delete Force",
                               [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                                 pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::DELETE_FORCE);
                               });
    server_->insert( mk );
    server_->setCallback( mk.name,
                          [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });
    menu_handler_force1.apply(*server_,mk.name);

    server_->applyChanges();
    break;
  case 2:
    mk.scale = 0.5;
    menu_handler_force2.insert("MoveMode",std::bind( &InteractiveMarkerInterface::changeMoveModeCb2, this, std::placeholders::_1));
    menu_handler_force2.insert("Delete Force",
                               [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                                 pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::DELETE_FORCE);
                               });
    server_->insert( mk );
    server_->setCallback( mk.name,
                          [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });
    menu_handler_force2.apply(*server_,mk.name);

    server_->applyChanges();
    break;
  default:
    break;
  }

  std::list<visualization_msgs::msg::InteractiveMarker>::iterator it = imlist.begin();

  while( it != imlist.end() )
    {
      if(it->name == mk_name.c_str()){
        imlist.erase(it);
        break;
      }
      it++;
    }
  imlist.push_back( mk );

  RCLCPP_INFO(this->get_logger(), "add mk");
  /* add mk */

}

void InteractiveMarkerInterface::initBodyMarkers(void){
  geometry_msgs::msg::PoseStamped ps;

  double scale_factor = 1.02;

  //for head
  ps.header.frame_id = head_link_frame_;
  visualization_msgs::msg::InteractiveMarker im =
    im_helpers::makeMeshMarker(head_link_frame_, head_mesh_, ps, scale_factor);
  makeIMVisible(im);
  server_->insert(im);
  menu_head_.apply(*server_, head_link_frame_);


  if(hand_type_ == "sandia_hand"){
    geometry_msgs::msg::PoseStamped ps;

    ps.header.frame_id = "right_f0_base";
    for(int i=0;i<4;i++){
      for(int j=0;j<3;j++){
        visualization_msgs::msg::InteractiveMarker fingerIm =
          makeSandiaHandInteractiveMarker(ps, "right", i, j);
        makeIMVisible(fingerIm);
        server_->insert(fingerIm,
                        [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ updateFinger(feedback, "rhand"); });
      }
    }

    ps.header.frame_id = "left_f0_base";
    for(int i=0;i<4;i++){
      for(int j=0;j<3;j++){
        visualization_msgs::msg::InteractiveMarker fingerIm =
          makeSandiaHandInteractiveMarker(ps, "left", i, j);
        makeIMVisible(fingerIm);
        server_->insert(fingerIm,
                        [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ updateFinger(feedback, "lhand"); });
      }
    }

  }else{
    //for right hand
    for(size_t i=0; i<rhand_mesh_.size(); i++){
      ps.header.frame_id = rhand_mesh_[i].link_name;
      ps.pose.orientation = rhand_mesh_[i].orientation;
      visualization_msgs::msg::InteractiveMarker handIm =
        im_helpers::makeMeshMarker( rhand_mesh_[i].link_name, rhand_mesh_[i].mesh_file, ps, scale_factor);
      makeIMVisible(handIm);
      server_->insert(handIm);
    }
  }
}


void InteractiveMarkerInterface::initControlMarkers(void){
  //Head Marker
  if(control_state_.head_on_ && control_state_.init_head_goal_){
    control_state_.init_head_goal_ = false;
    head_goal_pose_.header.stamp = builtin_interfaces::msg::Time();

    visualization_msgs::msg::InteractiveMarker HeadGoalIm =
      im_helpers::makeHeadGoalMarker( "head_point_goal", head_goal_pose_, 0.1);
    makeIMVisible(HeadGoalIm);
    server_->insert(HeadGoalIm,
                    std::bind( &InteractiveMarkerInterface::updateHeadGoal, this, std::placeholders::_1));
    menu_head_target_.apply(*server_,"head_point_goal");
  }
  if(!control_state_.head_on_){
    server_->erase("head_point_goal");
  }

  //Base Marker
  if(control_state_.base_on_ ){
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.orientation.w = 1;
    ps.header.frame_id = move_base_frame;
    visualization_msgs::msg::InteractiveMarker baseIm =
      InteractiveMarkerInterface::makeBaseMarker( "base_control", ps, 0.75, false);
    makeIMVisible(baseIm);
    server_->insert(baseIm,
                    std::bind( &InteractiveMarkerInterface::updateBase, this, std::placeholders::_1));

    menu_base_.apply(*server_,"base_control");
  }else{
    server_->erase("base_control");
  }

  //finger Control Marker
  if(use_finger_marker_ && control_state_.r_finger_on_){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "right_f0_base";

    server_->insert(makeFingerControlMarker("right_finger", ps),
                    [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ updateFinger(feedback, "rhand"); });
    menu_finger_r_.apply(*server_,"right_finger");
  }else{
    server_->erase("right_finger");
  }

  if(use_finger_marker_ && control_state_.l_finger_on_){
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "left_f0_base";

    server_->insert(makeFingerControlMarker("left_finger", ps),
                    [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ updateFinger(feedback, "lhand"); });
    menu_finger_l_.apply(*server_,"left_finger");
  }else{
    server_->erase("left_finger");
  }

  server_->applyChanges();
}


visualization_msgs::msg::InteractiveMarker InteractiveMarkerInterface::makeBaseMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped, float scale, bool fixed)
{
  (void)fixed;
  visualization_msgs::msg::InteractiveMarker mk;
  mk.header =  stamped.header;
  mk.name = name;
  mk.scale = scale;
  mk.pose = stamped.pose;

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;

  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  mk.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  mk.controls.push_back(control);
  return mk;

}



void InteractiveMarkerInterface::initHandler(void){
  bool use_menu;
  use_menu = this->declare_parameter("force_mode_menu", false);
  if(use_menu){
    menu_handler.insert("ForceMode",std::bind( &InteractiveMarkerInterface::changeForceModeCb, this, std::placeholders::_1));
  }

  use_menu = this->declare_parameter("move_menu", false);
  if(use_menu){
    use_menu = this->declare_parameter("move_safety_menu", false);
    if(use_menu){
      interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
      sub_menu_move_ = menu_handler.insert( "Move" );
      menu_handler.insert( sub_menu_move_,"Plan",
                           [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                             pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::PLAN); });
      menu_handler.insert( sub_menu_move_,"Execute",
                           [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                             pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::EXECUTE); });
      menu_handler.insert( sub_menu_move_,"Plan And Execute",
                           [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                             pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::PLAN_EXECUTE); });
      menu_handler.insert( sub_menu_move_,"Cancel",
                           [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                             pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::CANCEL_PLAN); });
    }else{
      menu_handler.insert("Move",
                          [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                            pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE); });
    }
  }

  use_menu = this->declare_parameter("change_using_ik_menu", false);
  if(use_menu){
    interactive_markers::MenuHandler::EntryHandle sub_menu_move_;
    sub_menu_move_ = menu_handler.insert( "Whether To Use IK" );
    start_ik_menu_ = menu_handler.insert( sub_menu_move_,"Start IK",std::bind( &InteractiveMarkerInterface::usingIKCb, this, std::placeholders::_1));
    menu_handler.setCheckState( start_ik_menu_, interactive_markers::MenuHandler::CHECKED );

    stop_ik_menu_ = menu_handler.insert( sub_menu_move_,"Stop IK",std::bind( &InteractiveMarkerInterface::usingIKCb, this, std::placeholders::_1));
    menu_handler.setCheckState( stop_ik_menu_, interactive_markers::MenuHandler::UNCHECKED );
  }

  use_menu = this->declare_parameter("touch_it_menu", false);
  if(use_menu){

    interactive_markers::MenuHandler::EntryHandle sub_menu_handle_touch_it;
    sub_menu_handle_touch_it = menu_handler.insert( "Touch It" );

    menu_handler.insert( sub_menu_handle_touch_it, "Execute",
                         [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                           pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::TOUCHIT_EXEC); });
    menu_handler.insert( sub_menu_handle_touch_it, "Cancel",
                         [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                           pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::TOUCHIT_CANCEL); });
  }
  use_menu = this->declare_parameter("look_hand_menu", false);
  if(use_menu){


    interactive_markers::MenuHandler::EntryHandle sub_menu_handle_look_hand;
    sub_menu_handle_look_hand = menu_handler.insert( "Look hand" );

    menu_handler.insert( sub_menu_handle_look_hand, "rarm",
                         [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                           pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::LOOK_RARM); });
    menu_handler.insert( sub_menu_handle_look_hand, "larm",
                         [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                           pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::LOOK_LARM); });
  }

  use_menu = this->declare_parameter("force_move_menu", false);
  if(use_menu){
    menu_handler.insert("Force Move",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::FORCE_MOVE); });
  }

  use_menu = this->declare_parameter("pick_menu", false);
  if(use_menu){
    menu_handler.insert("Pick",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::PICK); });
  }

  use_menu = this->declare_parameter("grasp_menu", false);
  if(use_menu){
    menu_handler.insert("Grasp",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::START_GRASP); });
  }

  use_menu = this->declare_parameter("harf_grasp_menu", false);
  if(use_menu){
    menu_handler.insert("Harf Grasp",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::HARF_GRASP); });
  }


  use_menu = this->declare_parameter("stop_grasp_menu", false);
  if(use_menu){
    menu_handler.insert("Stop Grasp",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::STOP_GRASP); });
  }

  use_menu = this->declare_parameter("set_origin_menu", false);
  if(use_menu){
    menu_handler.insert("Set Origin To Hand",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          setOriginCb(feedback, true); });

    menu_handler.insert("Set Origin",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          setOriginCb(feedback, false); });
  }

  use_menu = this->declare_parameter("reset_marker_pos_menu", false);
  if(use_menu){
    menu_handler.insert("Reset Marker Position",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::RESET_COORDS); });
  }

  use_menu = this->declare_parameter("manipulation_mode_menu", false);
  if(use_menu){
    menu_handler.insert("Manipulation Mode",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::MANIP_MODE); });
  }

  use_menu = this->declare_parameter("select_arm_menu", false);
  if(use_menu){
    sub_menu_handle = menu_handler.insert( "SelectArm" );
    h_mode_last = menu_handler.insert( sub_menu_handle, "Right Arm", std::bind( &InteractiveMarkerInterface::modeCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::CHECKED );
    h_mode_rightarm = h_mode_last;
    h_mode_last = menu_handler.insert( sub_menu_handle, "Left Arm", std::bind( &InteractiveMarkerInterface::modeCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
    h_mode_last = menu_handler.insert( sub_menu_handle, "Both Arms", std::bind( &InteractiveMarkerInterface::modeCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( h_mode_last, interactive_markers::MenuHandler::UNCHECKED );
    h_mode_last = h_mode_rightarm;
  }

  use_menu = this->declare_parameter("ik_mode_menu", false);
  if(use_menu){
    sub_menu_handle_ik = menu_handler.insert( "IK mode" );

    rotation_t_menu_ = menu_handler.insert( sub_menu_handle_ik, "6D (Position + Rotation)", std::bind( &InteractiveMarkerInterface::ikmodeCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( rotation_t_menu_ , interactive_markers::MenuHandler::CHECKED );
    rotation_nil_menu_ = menu_handler.insert( sub_menu_handle_ik, "3D (Position)", std::bind( &InteractiveMarkerInterface::ikmodeCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( rotation_nil_menu_, interactive_markers::MenuHandler::UNCHECKED );
  }

  use_menu = this->declare_parameter("use_torso_menu", false);
  if(use_menu){
    use_torso_menu_ = menu_handler.insert( "Links To Use" );

    use_torso_nil_menu_ = menu_handler.insert( use_torso_menu_, "Arm", std::bind( &InteractiveMarkerInterface::useTorsoCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( use_torso_nil_menu_, interactive_markers::MenuHandler::UNCHECKED );
    use_torso_t_menu_ = menu_handler.insert( use_torso_menu_, "Arm and Torso", std::bind( &InteractiveMarkerInterface::useTorsoCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( use_torso_t_menu_, interactive_markers::MenuHandler::UNCHECKED );
    use_fullbody_menu_ = menu_handler.insert( use_torso_menu_, "Fullbody", std::bind( &InteractiveMarkerInterface::useTorsoCb,this, std::placeholders::_1 ));
    menu_handler.setCheckState( use_fullbody_menu_, interactive_markers::MenuHandler::CHECKED );

  }



  interactive_markers::MenuHandler::EntryHandle sub_menu_handle_im_size;
  sub_menu_handle_im_size = menu_handler.insert( "IMsize" );
  menu_handler.insert( sub_menu_handle_im_size, "Large", std::bind( &InteractiveMarkerInterface::IMSizeLargeCb, this, std::placeholders::_1));
  menu_handler.insert( sub_menu_handle_im_size, "Middle", std::bind( &InteractiveMarkerInterface::IMSizeMiddleCb, this, std::placeholders::_1));
  menu_handler.insert( sub_menu_handle_im_size, "Small", std::bind( &InteractiveMarkerInterface::IMSizeSmallCb, this, std::placeholders::_1));

  use_menu = this->declare_parameter("publish_marker_menu", false);
  if(use_menu){
    menu_handler.insert("Publish Marker",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::PUBLISH_MARKER); });

  }




  //--------- menu_handler 1 ---------------
  menu_handler1.insert("ForceMode",std::bind( &InteractiveMarkerInterface::changeForceModeCb1, this, std::placeholders::_1));

  //--------- menu_handler 2 ---------------
  menu_handler2.insert("ForceMode",std::bind( &InteractiveMarkerInterface::changeForceModeCb2, this, std::placeholders::_1));
  menu_handler2.insert("Move",
                       [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                         pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE); });


  /* porting from PR2 marker control */
  /* head marker */

  head_target_handle_ = menu_head_.insert( "Target Point",
                                           std::bind( &InteractiveMarkerInterface::targetPointMenuCB, this, std::placeholders::_1 ) );
  menu_head_.setCheckState(head_target_handle_, interactive_markers::MenuHandler::UNCHECKED);

  head_auto_look_handle_ = menu_head_.insert( "Look Automatically", std::bind( &InteractiveMarkerInterface::lookAutomaticallyMenuCB,
                                                                               this, std::placeholders::_1 ) );
  menu_head_.setCheckState(head_auto_look_handle_, interactive_markers::MenuHandler::CHECKED);

  menu_head_target_.insert( "Look At",
                            [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                              proc_feedback(feedback, jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_HEAD_MARKER); });


  /* base move menu*/
  use_menu = this->declare_parameter("use_base_marker", false);
  control_state_.base_on_ = use_menu;

  menu_base_.insert("Base Move",
                    [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                      pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_BASE_MARKER); });
  menu_base_.insert("Reset Marker Position",
                    [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                      pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::RESET_COORDS, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_BASE_MARKER); });

  /*finger move menu*/
  use_finger_marker_ = this->declare_parameter("use_finger_marker", false);

  menu_finger_r_.insert("Move Finger",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_RFINGER_MARKER); });
  menu_finger_r_.insert("Reset Marker",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::RESET_COORDS, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_RFINGER_MARKER); });

  menu_finger_l_.insert("Move Finger",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::MOVE, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_LFINGER_MARKER); });
  menu_finger_l_.insert("Reset Marker",
                        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
                          pub_marker_menuCb(feedback, jsk_interactive_marker_msgs::msg::MarkerMenu::RESET_COORDS, jsk_interactive_marker_msgs::msg::MarkerMenu::TYPE_LFINGER_MARKER); });

}

void InteractiveMarkerInterface::addHandMarker(visualization_msgs::msg::InteractiveMarker &im,std::vector < UrdfProperty > urdf_vec){
  if(urdf_vec.size() > 0){
    for(size_t i=0; i<urdf_vec.size(); i++){
      UrdfProperty up = urdf_vec[i];
      if(up.model){
        KDL::Frame origin_frame;
        tf2::fromMsg(up.pose, origin_frame);

        LinkConstSharedPtr hand_root_link;
        hand_root_link = up.model->getLink(up.root_link_name);
        if(!hand_root_link){
          hand_root_link = up.model->getRoot();
        }
        im_utils::addMeshLinksControl(im, hand_root_link, origin_frame, !up.use_original_color, up.color, up.scale);
        for(size_t j=0; j<im.controls.size(); j++){
          if(im.controls[j].interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::BUTTON){
            im.controls[j].interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
            im.controls[j].name = "center_sphere";
          }
        }
      }else{
        addSphereMarker(im, up.scale, up.color);
      }
    }
  }else{
    double center_marker_size = 0.2;
    //gray
    std_msgs::msg::ColorRGBA color;
    color.r = color.g = color.b = 0.7;
    color.a = 0.5;
    addSphereMarker(im, center_marker_size, color);
  }
}

void InteractiveMarkerInterface::addSphereMarker(visualization_msgs::msg::InteractiveMarker &im, double scale, std_msgs::msg::ColorRGBA color){
    visualization_msgs::msg::Marker sphereMarker;
    sphereMarker.type = visualization_msgs::msg::Marker::SPHERE;

    sphereMarker.scale.x = scale;
    sphereMarker.scale.y = scale;
    sphereMarker.scale.z = scale;

    sphereMarker.color = color;

    visualization_msgs::msg::InteractiveMarkerControl sphereControl;
    sphereControl.name = "center_sphere";

    sphereControl.markers.push_back(sphereMarker);
    sphereControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    im.controls.push_back(sphereControl);
}


void InteractiveMarkerInterface::makeCenterSphere(visualization_msgs::msg::InteractiveMarker &mk, double mk_size){
  (void)mk_size;
  std::vector < UrdfProperty > null_urdf;
  if(control_state_.move_origin_state_ == ControlState::HAND_ORIGIN){
    if(control_state_.move_arm_ == ControlState::RARM){
      addHandMarker(mk, rhand_urdf_);
    }else if(control_state_.move_arm_ == ControlState::LARM){
      addHandMarker(mk, lhand_urdf_);
    }else{
      addHandMarker(mk, null_urdf);
    }
  }else{
    addHandMarker(mk, null_urdf);
  }
}

//im_mode
//0:normal move  1:operationModel 2:operationalModelFirst
void InteractiveMarkerInterface::changeMarkerMoveMode( std::string mk_name , int im_mode){
  switch(im_mode){
  case 0:
    changeMarkerMoveMode( mk_name, im_mode , 0.5);
    break;
  case 1:
  case 2:
    changeMarkerMoveMode( mk_name, im_mode , 0.3);
    break;
  default:
    changeMarkerMoveMode( mk_name, im_mode , 0.3);
    break;
  }
}

void InteractiveMarkerInterface::changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size){
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = base_frame;
  pose.pose.orientation.w = 1.0;
  changeMarkerMoveMode( mk_name, im_mode , mk_size, pose);
}

void InteractiveMarkerInterface::changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size, geometry_msgs::msg::PoseStamped dist_pose){
  RCLCPP_INFO(this->get_logger(), "changeMarkerMoveMode  marker:%s  mode:%d\n",mk_name.c_str(),im_mode);

  control_state_.marker_pose_ = dist_pose;

  interactive_markers::MenuHandler reset_handler;

  geometry_msgs::msg::PoseStamped pose;

  if ( target_frame != "" ) {
  }else{
    pose = dist_pose;
  }

  visualization_msgs::msg::InteractiveMarker mk;
  //0:normal move  1:operationModel 2:operationalModelFirst

  switch(im_mode){
  case 0:
    pose.header.stamp = builtin_interfaces::msg::Time();

    mk = make6DofControlMarker(mk_name.c_str(), pose, mk_size,
                               true, false );

    if(use_center_sphere_){
      makeCenterSphere(mk, mk_size);
    }

    makeIMVisible(mk);

    server_->insert( mk );
    server_->setCallback( mk.name,
                          [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });
    menu_handler.apply(*server_,mk.name);
    server_->applyChanges();
    break;
  case 1:
    mk = im_helpers::make6DofMarker(mk_name.c_str(), pose, mk_size,
                                    true, false );
    mk.description = mk_name.c_str();
    makeIMVisible(mk);
    server_->insert( mk );
    server_->setCallback( mk.name,
                          [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });
    menu_handler1.apply(*server_,mk.name);
    server_->applyChanges();
    break;

  case 2:
    mk = im_helpers::make6DofMarker(mk_name.c_str(), pose, mk_size,
                                    true, false );
    mk.description = mk_name.c_str();
    makeIMVisible(mk);

    server_->insert( mk );
    server_->setCallback( mk.name,
                          [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });
    menu_handler2.apply(*server_,mk.name);
    server_->applyChanges();
    break;
  default:
    mk = im_helpers::make6DofMarker(mk_name.c_str(), pose, mk_size,
                                    true, false );
    mk.description = mk_name.c_str();

    server_->insert( mk );
    server_->setCallback( mk.name,
                          [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });
    server_->applyChanges();
    break;
  }

  std::list<visualization_msgs::msg::InteractiveMarker>::iterator it = imlist.begin();

  while( it != imlist.end() )
    {
      if(it->name == mk_name.c_str()){
        imlist.erase(it);
        break;
      }
      it++;
    }
  imlist.push_back( mk );
}

void InteractiveMarkerInterface::changeMarkerOperationModelMode( std::string mk_name ){
  interactive_markers::MenuHandler reset_handler;
  menu_handler = reset_handler;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = base_frame;

  visualization_msgs::msg::InteractiveMarker mk =

    im_helpers::make6DofMarker(mk_name.c_str(), pose, 0.5,
                               true, false );
  mk.description = mk_name.c_str();
  menu_handler.insert("ForceMode",std::bind( &InteractiveMarkerInterface::changeForceModeCb, this, std::placeholders::_1));

  std::list<visualization_msgs::msg::InteractiveMarker>::iterator it = imlist.begin();

  while( it != imlist.end() )
    {
      if(it->name == mk_name.c_str()){
        imlist.erase(it);
        break;
      }
      it++;
    }
  imlist.push_back( mk );
  server_->insert( mk );

  server_->setCallback( mk.name,
                        [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){ proc_feedback(feedback); });

  menu_handler.apply(*server_,mk.name);
  server_->applyChanges();
}


InteractiveMarkerInterface::InteractiveMarkerInterface () : rclcpp::Node("jsk_marker_interface") {
  marker_name = this->declare_parameter("marker_name", std::string("100"));
  server_name = this->declare_parameter("server_name", std::string(""));
  base_frame = stripSlash(this->declare_parameter("base_frame", std::string("base_link")));
  move_base_frame = stripSlash(this->declare_parameter("move_base_frame", std::string("base_link")));
  target_frame = this->declare_parameter("target_frame", std::string(""));

  if ( server_name == "" ) {
    server_name = this->get_name();
  }

  pub_ = this->create_publisher<jsk_interactive_marker_msgs::msg::MarkerPose>("~/pose", 1);
  pub_update_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose_update", 1);
  pub_move_ = this->create_publisher<jsk_interactive_marker_msgs::msg::MarkerMenu>("~/marker_menu", 1);

  serv_set_ = this->create_service<jsk_interactive_marker_msgs::srv::MarkerSetPose>(
    "~/set_pose",
    std::bind(&InteractiveMarkerInterface::set_cb, this,
              std::placeholders::_1, std::placeholders::_2));
  serv_markers_set_ = this->create_service<jsk_interactive_marker_msgs::srv::MarkerSetPose>(
    "~/set_markers",
    std::bind(&InteractiveMarkerInterface::markers_set_cb, this,
              std::placeholders::_1, std::placeholders::_2));
  serv_markers_del_ = this->create_service<jsk_interactive_marker_msgs::srv::MarkerSetPose>(
    "~/del_markers",
    std::bind(&InteractiveMarkerInterface::markers_del_cb, this,
              std::placeholders::_1, std::placeholders::_2));
  serv_reset_ = this->create_service<jsk_interactive_marker_msgs::srv::SetPose>(
    "~/reset_pose",
    std::bind(&InteractiveMarkerInterface::reset_cb, this,
              std::placeholders::_1, std::placeholders::_2));

  sub_marker_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/move_marker", 1,
    std::bind(&InteractiveMarkerInterface::move_marker_cb, this, std::placeholders::_1));
  sub_marker_menu_ = this->create_subscription<jsk_interactive_marker_msgs::msg::MarkerMenu>(
    "~/select_marker_menu", 1,
    std::bind(&InteractiveMarkerInterface::marker_menu_cb, this, std::placeholders::_1));

  sub_toggle_start_ik_ = this->create_subscription<std_msgs::msg::Empty>(
    "~/toggle_start_ik", 1,
    std::bind(&InteractiveMarkerInterface::toggleStartIKCb, this, std::placeholders::_1));

  sub_toggle_ik_mode_ = this->create_subscription<std_msgs::msg::Empty>(
    "~/toggle_ik_mode", 1,
    std::bind(&InteractiveMarkerInterface::toggleIKModeCb, this, std::placeholders::_1));

  // dynamic_tf_publisher replacement: broadcast the registered transforms
  // periodically (the ROS 1 code requested freq=10 in SetDynamicTF)
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(this));
  dynamic_tf_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / 10.0),
    std::bind(&InteractiveMarkerInterface::publishDynamicTf, this));

  server_.reset(new interactive_markers::InteractiveMarkerServer(server_name, this));

  head_link_frame_ = this->declare_parameter("head_link_frame", std::string("head_tilt_link"));
  head_mesh_ = this->declare_parameter("head_mesh", std::string("package://pr2_description/meshes/head_v0/head_tilt.dae"));

  hand_type_ = this->declare_parameter("hand_type", std::string("GENERIC"));

  use_body_marker_ = this->declare_parameter("use_head_marker", false);
  use_center_sphere_ = this->declare_parameter("use_center_sphere", false);

  // ROS 1 read the structured rosparam "~mesh_config"; in ROS 2 this is a
  // YAML file specified by the string parameter "mesh_config_file"
  std::string mesh_config_file =
    this->declare_parameter("mesh_config_file", std::string(""));
  if (!mesh_config_file.empty()) {
    try {
      YAML::Node v = YAML::LoadFile(mesh_config_file);
      loadMeshes(v);
    }
    catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "failed to load %s: %s",
                   mesh_config_file.c_str(), e.what());
    }
  }

  head_goal_pose_.pose.position.x = 1.0;
  head_goal_pose_.pose.position.z = 1.0;
  head_goal_pose_.header.frame_id = base_frame;

  initHandler();
  if(use_body_marker_){
    initBodyMarkers();
  }
  initControlMarkers();
  changeMarkerMoveMode(marker_name.c_str(),0);
}

void InteractiveMarkerInterface::loadMeshes(const YAML::Node &val){
  loadUrdfFromYaml(val, "r_hand", rhand_urdf_);
  loadUrdfFromYaml(val, "l_hand", lhand_urdf_);
}

void InteractiveMarkerInterface::loadUrdfFromYaml(const YAML::Node &val, std::string name, std::vector<UrdfProperty>& mesh){
  if(val[name]){
    for(size_t i=0; i< val[name].size(); i++){
      YAML::Node nval = val[name][i];
      UrdfProperty up;
      //urdf file
      if(nval["urdf_file"]){
        std::string urdf_file = nval["urdf_file"].as<std::string>();
        std::cerr << "load urdf file: " << urdf_file << std::endl;
        up.model = im_utils::getModelInterface(urdf_file);
      }else if(nval["urdf_param"]){
        std::string urdf_param = stripSlash(nval["urdf_param"].as<std::string>());
        std::string urdf_model;
        if (!this->has_parameter(urdf_param)) {
          this->declare_parameter(urdf_param, std::string(""));
        }
        this->get_parameter(urdf_param, urdf_model);
        up.model = parseURDF(urdf_model);
      }

      if(nval["root_link"]){
        std::string root_link_name = nval["root_link"].as<std::string>();
        std::cerr << "root link name: " << root_link_name << std::endl;
        up.root_link_name = root_link_name;
      }else{
        up.root_link_name = "";
      }

      up.use_original_color = false;
      up.pose.orientation.w = 1.0;
      //pose
      if(nval["pose"]){
        YAML::Node pose = nval["pose"];
        if(pose["position"]){
          YAML::Node position = pose["position"];
          up.pose.position.x = position["x"].as<double>();
          up.pose.position.y = position["y"].as<double>();
          up.pose.position.z = position["z"].as<double>();
        }

        if(pose["orientation"]){
          YAML::Node orient = pose["orientation"];
          up.pose.orientation.x = orient["x"].as<double>();
          up.pose.orientation.y = orient["y"].as<double>();
          up.pose.orientation.z = orient["z"].as<double>();
          up.pose.orientation.w = orient["w"].as<double>();
        }
      }

      if(nval["color"]){
        YAML::Node color = nval["color"];
        up.color.r = color["r"].as<double>();
        up.color.g = color["g"].as<double>();
        up.color.b = color["b"].as<double>();
        up.color.a = color["a"].as<double>();
      }else{
        up.color.r = 1.0;
        up.color.g = 1.0;
        up.color.b = 0.0;
        up.color.a = 0.7;
      }
      if(nval["scale"]){
        up.scale = nval["scale"].as<double>();
      }else{
        up.scale = 1.05; //make bigger a bit
      }
      mesh.push_back(up);
    }
  }
}


void InteractiveMarkerInterface::markers_set_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Request> req,
                                                  std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Response> res ) {
  (void)res;
  bool setalready = false;

  std::list<visualization_msgs::msg::InteractiveMarker>::iterator it = imlist.begin();
  while( it != imlist.end() )
    {
      if( it->name == req->marker_name){
        setalready = true;
        break;
      }
      it++;
    }

  if(setalready){
    server_->setPose(req->marker_name, req->pose.pose, req->pose.header);
    server_->applyChanges();
  }
}

void InteractiveMarkerInterface::markers_del_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Request> req,
                                                  std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Response> res ) {
  (void)res;
  server_->erase(req->marker_name);
  server_->applyChanges();
  std::list<visualization_msgs::msg::InteractiveMarker>::iterator it = imlist.begin();
  while( it != imlist.end() )
    {
      if( it->name == req->marker_name){
        imlist.erase(it);
        break;
      }
      it++;
    }
}

void InteractiveMarkerInterface::move_marker_cb ( const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
  pub_marker_tf(msg->header, msg->pose);

  pub_marker_pose( msg->header, msg->pose, marker_name, jsk_interactive_marker_msgs::msg::MarkerPose::TYPE_GENERAL);

  server_->setPose(marker_name, msg->pose, msg->header);
  server_->applyChanges();
}


void InteractiveMarkerInterface::set_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Request> req,
                                          std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Response> res ) {
  (void)res;
  if ( req->markers.size() > 0 ) {
    visualization_msgs::msg::InteractiveMarker mk;
    if ( server_->get(req->marker_name, mk) ) {
      visualization_msgs::msg::InteractiveMarkerControl mkc;
      mkc.name = "additional_marker";
      mkc.always_visible = true;
      mkc.markers = req->markers;
      // delete added marker
      for ( std::vector<visualization_msgs::msg::InteractiveMarkerControl>::iterator it
              =  mk.controls.begin();
            it != mk.controls.end(); it++ ) {
        if ( it->name == mkc.name ){
          mk.controls.erase( it );
          break;
        }
      }
      mk.controls.push_back( mkc );
    }
  }
  std::string mName = req->marker_name;
  if(mName == ""){
    mName = marker_name;
  }
  pub_marker_tf(req->pose.header, req->pose.pose);

  server_->setPose(mName, req->pose.pose, req->pose.header);
  server_->applyChanges();
  pub_update_->publish(req->pose);
}

void InteractiveMarkerInterface::reset_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::SetPose::Request> req,
                                            std::shared_ptr<jsk_interactive_marker_msgs::srv::SetPose::Response> res ) {
  (void)req;
  (void)res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = base_frame;
  if ( target_frame != "" ) {
  } else {
    server_->setPose(marker_name, pose.pose);
  }
  server_->applyChanges();
}


void InteractiveMarkerInterface::makeIMVisible(visualization_msgs::msg::InteractiveMarker &im){
  for(size_t i=0; i<im.controls.size(); i++){
    im.controls[i].always_visible = true;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InteractiveMarkerInterface>());
  rclcpp::shutdown();

  return 0;
}
