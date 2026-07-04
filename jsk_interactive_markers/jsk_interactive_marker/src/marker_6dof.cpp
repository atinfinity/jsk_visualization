// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2016, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/tools.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <mutex>

class Marker6DOF : public rclcpp::Node {
public:
  Marker6DOF(): rclcpp::Node("marker_6dof"), show_6dof_circle_(true) {
    publish_tf_ = this->declare_parameter("publish_tf", false);
    publish_pose_periodically_ = this->declare_parameter("publish_pose_periodically", false);
    tf_frame_ = this->declare_parameter("tf_frame", std::string("object"));
    double tf_duration = this->declare_parameter("tf_duration", 0.1);
    object_type_ = this->declare_parameter("object_type", std::string("sphere"));
    object_x_ = this->declare_parameter("object_x", 1.0);
    object_y_ = this->declare_parameter("object_y", 1.0);
    object_z_ = this->declare_parameter("object_z", 1.0);
    object_r_ = this->declare_parameter("object_r", 1.0);
    object_g_ = this->declare_parameter("object_g", 1.0);
    object_b_ = this->declare_parameter("object_b", 1.0);
    object_a_ = this->declare_parameter("object_a", 1.0);
    frame_id_ = this->declare_parameter("frame_id", std::string("map"));
    latest_pose_.header.frame_id = frame_id_;
    double initial_x, initial_y, initial_z;
    initial_x = this->declare_parameter("initial_x", 0.0);
    initial_y = this->declare_parameter("initial_y", 0.0);
    initial_z = this->declare_parameter("initial_z", 0.0);
    latest_pose_.pose.position.x = initial_x;
    latest_pose_.pose.position.y = initial_y;
    latest_pose_.pose.position.z = initial_z;
    std::vector<double> initial_orientation
      = this->declare_parameter("initial_orientation", std::vector<double>());
    if (initial_orientation.size() == 4) {
      latest_pose_.pose.orientation.x = initial_orientation[0];
      latest_pose_.pose.orientation.y = initial_orientation[1];
      latest_pose_.pose.orientation.z = initial_orientation[2];
      latest_pose_.pose.orientation.w = initial_orientation[3];
    }
    else {
      latest_pose_.pose.orientation.w = 1.0;
    }
    line_width_ = this->declare_parameter("line_width", 0.007);
    mesh_file_ = this->declare_parameter("mesh_file", std::string(""));
    // if the parameter is not set, use the size of the object to compute the scale
    int_marker_scale_ = this->declare_parameter(
      "interactive_marker_scale",
      std::max(object_x_, std::max(object_y_, object_z_)) + 0.5);
    if (publish_tf_) {
      tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(this));
      tf_buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
      tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 1);
    pose_stamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/move_marker", 1,
      std::bind(&Marker6DOF::moveMarkerCB, this, std::placeholders::_1));

    circle_menu_entry_
      = menu_handler_.insert("Toggle 6DOF Circle",
                             std::bind(&Marker6DOF::menuFeedbackCB, this, std::placeholders::_1));
    menu_handler_.setCheckState(circle_menu_entry_,
                                interactive_markers::MenuHandler::CHECKED);
    server_.reset( new interactive_markers::InteractiveMarkerServer(this->get_name(), this));
    initializeInteractiveMarker();
    // Timer to update current pose on Rviz in the case which user re-enabled the plugin
    timer_pose_ = this->create_wall_timer(
      std::chrono::duration<double>(0.1),
      std::bind(&Marker6DOF::timerPoseCallback, this));
    if (publish_tf_) {
      timer_tf_ = this->create_wall_timer(
        std::chrono::duration<double>(tf_duration),
        std::bind(&Marker6DOF::timerTFCallback, this));
    }
  }

protected:
  void moveMarkerCB(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!publish_pose_periodically_) {
      pose_pub_->publish(*msg);
    }
    server_->setPose("marker", msg->pose, msg->header);
    latest_pose_ = geometry_msgs::msg::PoseStamped(*msg);
    server_->applyChanges();
  }


  void calculateBoundingBox( visualization_msgs::msg::Marker& object_marker){
    geometry_msgs::msg::Point top[5];
    top[0].x = object_x_/2;
    top[0].y = object_y_/2;
    top[1].x = -object_x_/2;
    top[1].y = object_y_/2;
    top[2].x = -object_x_/2;
    top[2].y = -object_y_/2;
    top[3].x = object_x_/2;
    top[3].y = -object_y_/2;
    top[4].x = object_x_/2;
    top[4].y = object_y_/2;

    geometry_msgs::msg::Point bottom[5];
    bottom[0].x = object_x_/2;
    bottom[0].y = object_y_/2;
    bottom[1].x = -object_x_/2;
    bottom[1].y = object_y_/2;
    bottom[2].x = -object_x_/2;
    bottom[2].y = -object_y_/2;
    bottom[3].x = object_x_/2;
    bottom[3].y = -object_y_/2;
    bottom[4].x = object_x_/2;
    bottom[4].y = object_y_/2;

    for(int i = 0; i< 5; i++){
      top[i].z = object_z_/2;
      bottom[i].z = -object_z_/2;
    }

    for(int i = 0; i< 4; i++){
      object_marker.points.push_back(top[i]);
      object_marker.points.push_back(top[i+1]);
      object_marker.points.push_back(bottom[i]);
      object_marker.points.push_back(bottom[i+1]);
      object_marker.points.push_back(top[i]);
      object_marker.points.push_back(bottom[i]);
    }
  }

  void initializeInteractiveMarker() {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = latest_pose_.header.frame_id;
    int_marker.name = "marker";
    int_marker.pose = geometry_msgs::msg::Pose(latest_pose_.pose);

    visualization_msgs::msg::Marker object_marker;
    if(object_type_ == std::string("cube")){
      object_marker.type = visualization_msgs::msg::Marker::CUBE;
      object_marker.scale.x = object_x_;
      object_marker.scale.y = object_y_;
      object_marker.scale.z = object_z_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
    }
    else if( object_type_ == std::string("sphere") ){
      object_marker.type = visualization_msgs::msg::Marker::SPHERE;
      object_marker.scale.x = object_x_;
      object_marker.scale.y = object_y_;
      object_marker.scale.z = object_z_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
    }
    else if(object_type_ == std::string("line")){
      object_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      object_marker.scale.x = line_width_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
      calculateBoundingBox(object_marker);
    }
    else if(object_type_ == std::string("mesh")){
      object_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      object_marker.scale.x = object_x_;
      object_marker.scale.y = object_y_;
      object_marker.scale.z = object_z_;
      object_marker.color.r = object_r_;
      object_marker.color.g = object_g_;
      object_marker.color.b = object_b_;
      object_marker.color.a = object_a_;
      object_marker.pose.orientation.w = 1.0;
      object_marker.mesh_resource = mesh_file_;
    }


    visualization_msgs::msg::InteractiveMarkerControl object_marker_control;
    object_marker_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    object_marker_control.always_visible = true;
    object_marker_control.markers.push_back(object_marker);
    int_marker.controls.push_back(object_marker_control);

    visualization_msgs::msg::InteractiveMarkerControl control;
    if (show_6dof_circle_) {
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;

      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }

    int_marker.scale = int_marker_scale_;

    server_->insert(int_marker,
                    std::bind(&Marker6DOF::processFeedbackCB, this, std::placeholders::_1));

    menu_handler_.apply(*server_, "marker");
    server_->applyChanges();
  }

  void publishTF(const geometry_msgs::msg::PoseStamped& pose) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = pose.header.stamp;
    transform.header.frame_id = pose.header.frame_id;
    transform.child_frame_id = tf_frame_;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void processFeedbackCB(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = feedback->header;
    pose.pose = feedback->pose;

    if (publish_tf_) {
      // feedback->header.frame_id equals to fixed frame of rviz.
      // Pose should be transformed respect to frame_id_ to publish correct tf frames.
      try {
        latest_pose_ = tf_buffer_->transform(pose, frame_id_);
      }
      catch (tf2::TransformException& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to transform " << pose.header.frame_id << " to " << frame_id_ << ": " << e.what());
        return;
      }
    }
    else {
      latest_pose_ = pose;
    }
    if (!publish_pose_periodically_) {
      pose_pub_->publish(pose);
    }
  }

  void menuFeedbackCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
    (void)feedback;
    show_6dof_circle_ = !show_6dof_circle_;
    if (show_6dof_circle_) {
      menu_handler_.setCheckState(circle_menu_entry_,
                                  interactive_markers::MenuHandler::CHECKED);
    }
    else {
      menu_handler_.setCheckState(circle_menu_entry_,
                                  interactive_markers::MenuHandler::UNCHECKED);
    }
    initializeInteractiveMarker(); // ok...?
  }

  void timerPoseCallback() {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PoseStamped pose = latest_pose_;
    pose.header.stamp = this->now();
    server_->setPose("marker", pose.pose, pose.header);
    server_->applyChanges();
    if (publish_pose_periodically_) {
      pose_pub_->publish(pose);
    }
  }

  void timerTFCallback() {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PoseStamped pose = latest_pose_;
    pose.header.stamp = this->now();
    publishTF(pose);
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::string object_type_;
  double object_x_;
  double object_y_;
  double object_z_;
  double object_r_;
  double object_g_;
  double object_b_;
  double object_a_;
  std::string frame_id_;
  double line_width_;
  double int_marker_scale_;
  std::string mesh_file_;
  bool show_6dof_circle_;
  bool publish_tf_;
  bool publish_pose_periodically_;
  std::string tf_frame_;
  rclcpp::TimerBase::SharedPtr timer_pose_;
  rclcpp::TimerBase::SharedPtr timer_tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex mutex_;
  interactive_markers::MenuHandler::EntryHandle circle_menu_entry_;
  geometry_msgs::msg::PoseStamped latest_pose_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Marker6DOF>());
  rclcpp::shutdown();
  return 0;
}
