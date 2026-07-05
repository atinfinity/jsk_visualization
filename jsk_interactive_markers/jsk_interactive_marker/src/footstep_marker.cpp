// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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

#include <iostream>
#include <chrono>
#include <interactive_markers/tools.hpp>
#include <jsk_interactive_marker/footstep_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <jsk_footstep_msgs/msg/footstep_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/StdVector>

using namespace std::chrono_literals;

// tf2 does not accept frame_ids with a leading slash
static inline std::string stripSlash(const std::string& in)
{
  if (!in.empty() && in[0] == '/') {
    return in.substr(1);
  }
  return in;
}

FootstepMarker::FootstepMarker():
  rclcpp::Node("footstep_marker"),
  plan_run_(false), exec_run_(false), lleg_first_(true) {
  // read parameters
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // dynamic_reconfigure -> declared parameters + parameter callback
  use_projection_service_ = this->declare_parameter("use_projection_service", false);
  use_projection_topic_ = this->declare_parameter("use_projection_topic", false);
  use_plane_snap_ = this->declare_parameter("use_plane_snap", false);
  use_2d_ = this->declare_parameter("use_2d", false);
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&FootstepMarker::parametersCallback, this, std::placeholders::_1));
  foot_size_x_ = this->declare_parameter("foot_size_x", 0.247);
  foot_size_y_ = this->declare_parameter("foot_size_y", 0.135);
  foot_size_z_ = this->declare_parameter("foot_size_z", 0.01);
  lfoot_frame_id_ = stripSlash(this->declare_parameter("lfoot_frame_id", std::string("lfsensor")));
  rfoot_frame_id_ = stripSlash(this->declare_parameter("rfoot_frame_id", std::string("rfsensor")));
  show_6dof_control_ = this->declare_parameter("show_6dof_control", true);
  always_planning_ = this->declare_parameter("always_planning", true);
  project_footprint_pub_ = this->create_publisher<jsk_interactive_marker_msgs::msg::SnapFootPrintInput>(
    "~/project_footprint", 1);
  // read lfoot_offset
  readPoseParam("lfoot_offset", lleg_offset_);
  readPoseParam("rfoot_offset", rleg_offset_);

  footstep_margin_ = this->declare_parameter("footstep_margin", 0.2);
  use_footstep_planner_ = this->declare_parameter("use_footstep_planner", true);

  use_footstep_controller_ = this->declare_parameter("use_footstep_controller", true);
  use_initial_footstep_tf_ = this->declare_parameter("use_initial_footstep_tf", true);
  wait_snapit_server_ = this->declare_parameter("wait_snapit_server", false);
  bool nowait = this->declare_parameter("no_wait", true);
  marker_frame_id_ = stripSlash(this->declare_parameter("frame_id", std::string("map")));
  footstep_pub_ = this->create_publisher<jsk_footstep_msgs::msg::FootstepArray>("footstep_from_marker", 1);
  snapit_client_ = this->create_client<jsk_recognition_msgs::srv::CallSnapIt>("snapit");
  snapped_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/snapped_pose", 1);
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 1);
  estimate_occlusion_client_ = this->create_client<std_srvs::srv::Empty>("require_estimation");
  look_ground_client_ = this->create_client<std_srvs::srv::Empty>("/lookaround_ground");
  set_heuristic_client_ = this->create_client<jsk_interactive_marker_msgs::srv::SetHeuristic>(
    "/footstep_planner/set_heuristic");
  project_footprint_client_ = this->create_client<jsk_interactive_marker_msgs::srv::SnapFootPrint>(
    "project_footprint");
  if (!nowait && wait_snapit_server_) {
    if (!snapit_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "snapit service is not available");
    }
  }

  initial_reference_frame_ = stripSlash(
    this->declare_parameter("initial_reference_frame", std::string("")));
  if (!initial_reference_frame_.empty()) {
    use_initial_reference_ = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "initial_reference_frame: " << initial_reference_frame_);
  }
  else {
    use_initial_reference_ = false;
    RCLCPP_INFO(this->get_logger(), "initial_reference_frame is not specified ");
  }

  server_.reset(new interactive_markers::InteractiveMarkerServer(this->get_name(), this));
  menu_handler_.insert( "Look Ground",
                        std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert( "Execute the Plan",
                        std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert( "Force to replan",
                        std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert( "Cancel Walk",
                        std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert( "Toggle 6dof marker",
                        std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert("Straight Heuristic",
                       std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert("Stepcost Heuristic**",
                       std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert("LLeg First",
                       std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  menu_handler_.insert("RLeg First",
                       std::bind(&FootstepMarker::menuFeedbackCB, this, std::placeholders::_1));
  marker_pose_.header.frame_id = marker_frame_id_;
  marker_pose_.header.stamp = this->now();
  marker_pose_.pose.orientation.w = 1.0;

  resetLegPoses();

  // initialize lleg_initial_pose, rleg_initial_pose
  lleg_initial_pose_.position.y = footstep_margin_ / 2.0;
  lleg_initial_pose_.orientation.w = 1.0;
  rleg_initial_pose_.position.y = - footstep_margin_ / 2.0;
  rleg_initial_pose_.orientation.w = 1.0;

  if (use_initial_reference_) {
    // bounded wait: do not block forever if the transform never becomes
    // available, so that the marker stays operable.
    rclcpp::Time start_time = this->now();
    bool resolved = false;
    while (rclcpp::ok() && (this->now() - start_time).seconds() < 3.0) {
      if (tf_buffer_->_frameExists(marker_frame_id_) &&
          tf_buffer_->_frameExists(initial_reference_frame_) &&
          tf_buffer_->canTransform(marker_frame_id_, initial_reference_frame_,
                                   tf2::TimePointZero)) {
        resolved = true;
        break;
      }
      rclcpp::sleep_for(100ms);
    }
    if (resolved) {
      try {
        RCLCPP_INFO(this->get_logger(), "resolved transform %s => %s", marker_frame_id_.c_str(),
                    initial_reference_frame_.c_str());
        geometry_msgs::msg::TransformStamped transform
          = tf_buffer_->lookupTransform(marker_frame_id_, initial_reference_frame_,
                                        tf2::TimePointZero);
        marker_pose_.pose.position.x = transform.transform.translation.x;
        marker_pose_.pose.position.y = transform.transform.translation.y;
        marker_pose_.pose.position.z = transform.transform.translation.z;
        marker_pose_.pose.orientation = transform.transform.rotation;
      }
      catch (tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to lookup transformation: %s", e.what());
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(),
                  "could not resolve transform %s => %s, use the default marker pose",
                  marker_frame_id_.c_str(), initial_reference_frame_.c_str());
    }
  }

  if (!rclcpp::ok()) {          // interrupted while waiting for TF
    return;
  }

  initializeInteractiveMarker();

  ac_ = rclcpp_action::create_client<PlanFootsteps>(this, "footstep_planner");
  ac_exec_ = rclcpp_action::create_client<ExecFootsteps>(this, "footstep_controller");
  if (use_footstep_planner_) {
    RCLCPP_INFO(this->get_logger(), "waiting planner server...");
    if (ac_->wait_for_action_server(2s)) {
      RCLCPP_INFO(this->get_logger(), "found planner server...");
    }
    else {
      RCLCPP_WARN(this->get_logger(),
                  "footstep_planner action server is not available; "
                  "planning requests will be skipped until the server appears");
    }
  }
  if (use_footstep_controller_) {
    RCLCPP_INFO(this->get_logger(), "waiting controller server...");
    if (ac_exec_->wait_for_action_server(2s)) {
      RCLCPP_INFO(this->get_logger(), "found controller server...");
    }
    else {
      RCLCPP_WARN(this->get_logger(),
                  "footstep_controller action server is not available; "
                  "execution requests will be skipped until the server appears");
    }
  }

  if (!rclcpp::ok()) {          // interrupted while waiting for the servers
    return;
  }

  move_marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "move_marker", 1, std::bind(&FootstepMarker::moveMarkerCB, this, std::placeholders::_1));
  menu_command_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "menu_command", 1, std::bind(&FootstepMarker::menuCommandCB, this, std::placeholders::_1));
  exec_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "~/execute", 1, std::bind(&FootstepMarker::executeCB, this, std::placeholders::_1));
  resume_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "~/resume", 1, std::bind(&FootstepMarker::resumeCB, this, std::placeholders::_1));
  plan_if_possible_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/force_to_replan",
    std::bind(&FootstepMarker::forceToReplan, this,
              std::placeholders::_1, std::placeholders::_2));
  if (use_initial_footstep_tf_) {
    // bounded wait for the initial footstep TF; updateInitialFootstep keeps
    // trying periodically even if it is not available yet.
    rclcpp::Time start_time = this->now();
    bool resolved = false;
    while (rclcpp::ok() && (this->now() - start_time).seconds() < 3.0) {
      if (tf_buffer_->_frameExists(marker_frame_id_) &&
          tf_buffer_->_frameExists(lfoot_frame_id_) &&
          tf_buffer_->_frameExists(rfoot_frame_id_) &&
          tf_buffer_->canTransform(marker_frame_id_, lfoot_frame_id_, tf2::TimePointZero) &&
          tf_buffer_->canTransform(marker_frame_id_, rfoot_frame_id_, tf2::TimePointZero)) {
        resolved = true;
        break;
      }
      rclcpp::sleep_for(100ms);
    }
    if (resolved) {
      RCLCPP_INFO(this->get_logger(), "resolved transform {%s, %s} => %s", lfoot_frame_id_.c_str(),
                  rfoot_frame_id_.c_str(), marker_frame_id_.c_str());
    }
    else {
      RCLCPP_WARN(this->get_logger(), "could not yet resolve transform {%s, %s} => %s",
                  lfoot_frame_id_.c_str(), rfoot_frame_id_.c_str(), marker_frame_id_.c_str());
    }
  }
  if (!rclcpp::ok()) {          // interrupted while waiting for TF
    return;
  }
  projection_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/projected_pose", 1,
    std::bind(&FootstepMarker::projectionCallback, this, std::placeholders::_1));
  // ROS 1 version updated the initial footstep from the main loop at 10Hz
  initial_footstep_timer_ = this->create_wall_timer(
    100ms, std::bind(&FootstepMarker::updateInitialFootstep, this));
}

rcl_interfaces::msg::SetParametersResult FootstepMarker::parametersCallback(
  const std::vector<rclcpp::Parameter>& parameters)
{
  std::lock_guard<std::mutex> lock(plane_mutex_);
  for (const rclcpp::Parameter& parameter : parameters) {
    if (parameter.get_name() == "use_projection_topic") {
      use_projection_topic_ = parameter.as_bool();
    }
    else if (parameter.get_name() == "use_projection_service") {
      use_projection_service_ = parameter.as_bool();
    }
    else if (parameter.get_name() == "use_plane_snap") {
      use_plane_snap_ = parameter.as_bool();
    }
    else if (parameter.get_name() == "use_2d") {
      use_2d_ = parameter.as_bool();
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void FootstepMarker::readPoseParam(const std::string& param, Eigen::Affine3d& offset) {
  offset = Eigen::Affine3d::Identity();
  std::vector<double> v = this->declare_parameter(param, std::vector<double>());
  if (!v.empty()) {
    // check if v is 7 length Array
    if (v.size() == 7) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = v[0];
      pose.position.y = v[1];
      pose.position.z = v[2];
      pose.orientation.x = v[3];
      pose.orientation.y = v[4];
      pose.orientation.z = v[5];
      pose.orientation.w = v[6];
      tf2::fromMsg(pose, offset); // msg -> eigen
    }
    else {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          param << " is malformed, which should be 7 length array");
    }
  }
  else {
    RCLCPP_WARN_STREAM(this->get_logger(), "there is no parameter on " << param);
  }
}

void FootstepMarker::resetLegPoses() {
  lleg_pose_.orientation.x = 0.0;
  lleg_pose_.orientation.y = 0.0;
  lleg_pose_.orientation.z = 0.0;
  lleg_pose_.orientation.w = 1.0;
  lleg_pose_.position.x = 0.0;
  lleg_pose_.position.y = footstep_margin_ / 2.0;
  lleg_pose_.position.z = 0.0;

  rleg_pose_.orientation.x = 0.0;
  rleg_pose_.orientation.y = 0.0;
  rleg_pose_.orientation.z = 0.0;
  rleg_pose_.orientation.w = 1.0;
  rleg_pose_.position.x = 0.0;
  rleg_pose_.position.y = - footstep_margin_ / 2.0;
  rleg_pose_.position.z = 0.0;
}

void FootstepMarker::computeLegTransformation(uint8_t leg) {
  if (!snapit_client_->service_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "failed to call snapit: service is not available");
    return;
  }
  auto req = std::make_shared<jsk_recognition_msgs::srv::CallSnapIt::Request>();
  req->request.header.stamp = this->now();
  req->request.header.frame_id = marker_frame_id_;
  req->request.target_plane.header.stamp = this->now();
  req->request.target_plane.header.frame_id = marker_frame_id_;
  req->request.target_plane.polygon = computePolygon(leg);
  // ROS 1 version called the service synchronously; use an asynchronous
  // request and update the leg pose in the continuation.
  snapit_client_->async_send_request(
    req,
    [this, leg](rclcpp::Client<jsk_recognition_msgs::srv::CallSnapIt>::SharedFuture future) {
      auto response = future.get();
      geometry_msgs::msg::Pose new_pose;
      Eigen::Affine3d A, T, B, B_prime;
      tf2::fromMsg(response->transformation, T);
      tf2::fromMsg(marker_pose_.pose, A);
      if (leg == jsk_footstep_msgs::msg::Footstep::LEFT) {
        tf2::fromMsg(lleg_pose_, B);
      }
      else if (leg == jsk_footstep_msgs::msg::Footstep::RIGHT) {
        tf2::fromMsg(rleg_pose_, B);
      }
      B_prime = A.inverse() * T * A * B;
      new_pose = tf2::toMsg(B_prime);
      if (leg == jsk_footstep_msgs::msg::Footstep::LEFT) {
        lleg_pose_ = new_pose;
      }
      else if (leg == jsk_footstep_msgs::msg::Footstep::RIGHT) {
        rleg_pose_ = new_pose;
      }
    });
}

void FootstepMarker::snapLegs() {
  computeLegTransformation(jsk_footstep_msgs::msg::Footstep::LEFT);
  computeLegTransformation(jsk_footstep_msgs::msg::Footstep::RIGHT);
}

geometry_msgs::msg::Polygon FootstepMarker::computePolygon(uint8_t leg) {
  geometry_msgs::msg::Polygon polygon;
  // tree
  // marker_frame_id_ ---[marker_pose_]---> [leg_pose_] --> points
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;
  points.push_back(Eigen::Vector3d(foot_size_x_ / 2.0, foot_size_y_ / 2.0, 0.0));
  points.push_back(Eigen::Vector3d(-foot_size_x_ / 2.0, foot_size_y_ / 2.0, 0.0));
  points.push_back(Eigen::Vector3d(-foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0));
  points.push_back(Eigen::Vector3d(foot_size_x_ / 2.0, -foot_size_y_ / 2.0, 0.0));

  Eigen::Affine3d marker_pose_eigen;
  Eigen::Affine3d leg_pose_eigen = Eigen::Affine3d::Identity();
  tf2::fromMsg(marker_pose_.pose, marker_pose_eigen);
  if (leg == jsk_footstep_msgs::msg::Footstep::LEFT) {
    tf2::fromMsg(lleg_pose_, leg_pose_eigen);
  }
  else if (leg == jsk_footstep_msgs::msg::Footstep::RIGHT) {
    tf2::fromMsg(rleg_pose_, leg_pose_eigen);
  }

  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector3d point = points[i];
    Eigen::Vector3d new_point = marker_pose_eigen * leg_pose_eigen * point;
    geometry_msgs::msg::Point32 point_msg;
    point_msg.x = new_point[0];
    point_msg.y = new_point[1];
    point_msg.z = new_point[2];
    polygon.points.push_back(point_msg);
  }

  return polygon;
}

void FootstepMarker::executeCB(const std_msgs::msg::Empty::ConstSharedPtr msg) {
  (void)msg;
  executeFootstep();
}

void FootstepMarker::resumeCB(const std_msgs::msg::Empty::ConstSharedPtr msg) {
  (void)msg;
  resumeFootstep();
}

void FootstepMarker::menuCommandCB(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
  processMenuFeedback(msg->data);
}

void FootstepMarker::updateInitialFootstep() {
  if (!use_initial_footstep_tf_) {
    return;
  }
  if (!tf_buffer_->_frameExists(marker_frame_id_) ||
      !tf_buffer_->_frameExists(lfoot_frame_id_) ||
      !tf_buffer_->_frameExists(rfoot_frame_id_) ||
      !tf_buffer_->canTransform(marker_frame_id_, lfoot_frame_id_, tf2::TimePointZero) ||
      !tf_buffer_->canTransform(marker_frame_id_, rfoot_frame_id_, tf2::TimePointZero)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "waiting for transform {%s, %s} => %s", lfoot_frame_id_.c_str(),
                         rfoot_frame_id_.c_str(), marker_frame_id_.c_str());
    return;
  }
  try {
    geometry_msgs::msg::TransformStamped lfoot_transform
      = tf_buffer_->lookupTransform(marker_frame_id_, lfoot_frame_id_, tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped rfoot_transform
      = tf_buffer_->lookupTransform(marker_frame_id_, rfoot_frame_id_, tf2::TimePointZero);

    // apply offset
    // convert like tf -> eigen -> msg
    Eigen::Affine3d le = tf2::transformToEigen(lfoot_transform) * lleg_offset_;
    lleg_initial_pose_ = tf2::toMsg(le);  // eigen -> msg
    Eigen::Affine3d re = tf2::transformToEigen(rfoot_transform) * rleg_offset_;
    rleg_initial_pose_ = tf2::toMsg(re);  // eigen -> msg

    // we need to move the marker
    initializeInteractiveMarker();
  }
  catch (tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Failed to lookup transformation: %s", e.what());
  }
}

void FootstepMarker::lookGround()
{
  if (!look_ground_client_->service_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to look ground: service is not available");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  // asynchronous call instead of the ROS 1 blocking ros::service::call
  look_ground_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
      (void)future;
      RCLCPP_INFO(this->get_logger(), "Finished to look ground");
    });
}

void FootstepMarker::forceToReplan(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  planIfPossible();
}

void FootstepMarker::processMenuFeedback(uint8_t menu_entry_id) {
  switch (menu_entry_id) {
  case 1: {                     // look ground
    lookGround();
    break;
  }
  case 2: {                     // execute
    executeFootstep();
    break;
  }
  case 3: {                     // replan
    planIfPossible();
    break;
  }
  case 4: {                     // cancel walk
    cancelWalk();
    break;
  }
  case 5: {                     // toggle 6dof marker
    show_6dof_control_ = !show_6dof_control_;
    break;
  }
  case 6: {                     // toggle 6dof marker
    changePlannerHeuristic(":straight-heuristic");
    break;
  }
  case 7: {                     // toggle 6dof marker
    changePlannerHeuristic(":stepcost-heuristic**");
    break;
  }
  case 8: {                     // toggle 6dof marker
    lleg_first_ = true;
    break;
  }
  case 9: {                     // toggle 6dof marker
    lleg_first_ = false;
    break;
  }

  default: {
    break;
  }
  }
}

void FootstepMarker::changePlannerHeuristic(const std::string& heuristic)
{
  if (!set_heuristic_client_->service_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "failed to set heuristic: service is not available");
    return;
  }
  auto req = std::make_shared<jsk_interactive_marker_msgs::srv::SetHeuristic::Request>();
  req->heuristic = heuristic;
  // asynchronous call instead of the ROS 1 blocking ros::service::call
  set_heuristic_client_->async_send_request(
    req,
    [this, heuristic](rclcpp::Client<jsk_interactive_marker_msgs::srv::SetHeuristic>::SharedFuture future) {
      (void)future;
      RCLCPP_INFO(this->get_logger(), "Success to set heuristic: %s", heuristic.c_str());
    });
}

void FootstepMarker::cancelWalk()
{
  RCLCPP_WARN(this->get_logger(), "canceling walking");
  if (ac_exec_->action_server_is_ready()) {
    ac_exec_->async_cancel_all_goals();
    RCLCPP_WARN(this->get_logger(), "canceled walking");
  }
  else {
    RCLCPP_WARN(this->get_logger(),
                "footstep_controller action server is not available; nothing to cancel");
  }
}

void FootstepMarker::callEstimateOcclusion()
{
  if (!estimate_occlusion_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "require_estimation service is not available");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  // asynchronous call instead of the ROS 1 blocking call
  estimate_occlusion_client_->async_send_request(req);
}

bool FootstepMarker::projectMarkerToPlane()
{
  if (use_projection_service_) {
    if (!project_footprint_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Failed to snap footprint: service is not available");
      return false;
    }
    auto req = std::make_shared<jsk_interactive_marker_msgs::srv::SnapFootPrint::Request>();
    req->input_pose = marker_pose_;
    req->lleg_pose.orientation.w = 1.0;
    req->rleg_pose.orientation.w = 1.0;
    req->lleg_pose.position.y = footstep_margin_ / 2.0;
    req->rleg_pose.position.y = - footstep_margin_ / 2.0;
    // ROS 1 version called the service synchronously and planned afterwards;
    // send the request asynchronously and continue in the response callback.
    project_footprint_client_->async_send_request(
      req,
      [this](rclcpp::Client<jsk_interactive_marker_msgs::srv::SnapFootPrint>::SharedFuture future) {
        auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(this->get_logger(), "Failed to snap footprint");
          return;
        }
        try {
          // Resolve tf
          geometry_msgs::msg::PoseStamped resolved_pose
            = tf_buffer_->transform(response->snapped_pose,
                                    stripSlash(marker_pose_.header.frame_id));
          // Check distance to project
          Eigen::Vector3d projected_point, marker_point;
          tf2::fromMsg(marker_pose_.pose.position, marker_point);
          tf2::fromMsg(resolved_pose.pose.position, projected_point);
          if ((projected_point - marker_point).norm() < 0.3) {
            server_->setPose("footstep_marker", resolved_pose.pose);
            snapped_pose_pub_->publish(resolved_pose);
            current_pose_pub_->publish(resolved_pose);
            server_->applyChanges();
            marker_pose_.pose = resolved_pose.pose;
            // plan after a successful projection
            planIfPossible();
          }
        }
        catch (tf2::TransformException& e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to lookup transformation: %s", e.what());
        }
      });
    return false;               // the plan is triggered from the continuation
  }
  else if (use_projection_topic_) {
    jsk_interactive_marker_msgs::msg::SnapFootPrintInput msg;
    msg.input_pose = marker_pose_;
    msg.lleg_pose.orientation.w = 1.0;
    msg.rleg_pose.orientation.w = 1.0;
    msg.lleg_pose.position.y = footstep_margin_ / 2.0;
    msg.rleg_pose.position.y = - footstep_margin_ / 2.0;
    project_footprint_pub_->publish(msg);
    return true;                // true...?
  }
  return false;
}

void FootstepMarker::menuFeedbackCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  processMenuFeedback(feedback->menu_entry_id);
}

void FootstepMarker::processFeedbackCB(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
  std::lock_guard<std::mutex> lock(plane_mutex_);

  marker_pose_.header = feedback->header;
  marker_pose_.pose = feedback->pose;
  marker_frame_id_ = feedback->header.frame_id;
  bool skip_plan = false;
  current_pose_pub_->publish(marker_pose_);
  try {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      if (use_plane_snap_) {
        skip_plan = !projectMarkerToPlane();
      }
    }
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP && !skip_plan) {
      if (always_planning_) planIfPossible();
    }
  }
  catch (tf2::TransformException& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lookup transformation: %s", e.what());
  }
}

void FootstepMarker::resumeFootstep() {
  if (!use_footstep_controller_) {
    return;
  }
  if (!ac_exec_->action_server_is_ready()) {
    RCLCPP_WARN(this->get_logger(),
                "footstep_controller action server is not available; cannot resume");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    if (exec_run_) {
      RCLCPP_ERROR(this->get_logger(), "still executing footstep");
      return;
    }
    exec_run_ = true;
  }
  ExecFootsteps::Goal goal;
  goal.strategy = ExecFootsteps::Goal::RESUME;
  auto options = ExecuteActionClient::SendGoalOptions();
  options.goal_response_callback = [this](ExecGoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) {
      std::lock_guard<std::mutex> lock(exec_mutex_);
      exec_run_ = false;
      RCLCPP_ERROR(this->get_logger(), "resume goal was rejected by the footstep controller");
    }
  };
  options.result_callback = [this](const ExecGoalHandle::WrappedResult& result) {
    (void)result;
    std::lock_guard<std::mutex> lock(exec_mutex_);
    exec_run_ = false;
  };
  ac_exec_->async_send_goal(goal, options);
}

void FootstepMarker::projectionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
{
  try {
    geometry_msgs::msg::PoseStamped resolved_pose
      = tf_buffer_->transform(*pose, stripSlash(marker_pose_.header.frame_id));
    // Check distance to project
    Eigen::Vector3d projected_point, marker_point;
    tf2::fromMsg(marker_pose_.pose.position, marker_point);
    tf2::fromMsg(resolved_pose.pose.position, projected_point);
    if ((projected_point - marker_point).norm() < 0.3) {
      marker_pose_.pose = resolved_pose.pose;
      snapped_pose_pub_->publish(resolved_pose);
      current_pose_pub_->publish(resolved_pose);
    }
  }
  catch (tf2::TransformException& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lookup transformation: %s", e.what());
  }
}

void FootstepMarker::executeFootstep() {
  if (!use_footstep_controller_) {
    return;
  }
  if (!ac_exec_->action_server_is_ready()) {
    RCLCPP_WARN(this->get_logger(),
                "footstep_controller action server is not available; cannot execute footstep");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    if (exec_run_) {
      RCLCPP_ERROR(this->get_logger(), "still executing footstep");
      return;
    }
  }
  if (!plan_result_) {
    RCLCPP_ERROR(this->get_logger(), "no planner result is available");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(exec_mutex_);
    exec_run_ = true;
  }

  ExecFootsteps::Goal goal;
  goal.footstep = plan_result_->result;
  RCLCPP_INFO(this->get_logger(), "sending goal...");
  auto options = ExecuteActionClient::SendGoalOptions();
  options.goal_response_callback = [this](ExecGoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) {
      std::lock_guard<std::mutex> lock(exec_mutex_);
      exec_run_ = false;
      RCLCPP_ERROR(this->get_logger(), "execute goal was rejected by the footstep controller");
    }
  };
  options.result_callback = [this](const ExecGoalHandle::WrappedResult& result) {
    (void)result;
    std::lock_guard<std::mutex> lock(exec_mutex_);
    exec_run_ = false;
  };
  ac_exec_->async_send_goal(goal, options);
}

void FootstepMarker::planIfPossible() {
  std::lock_guard<std::mutex> lock(plan_run_mutex_);
  // check the status of the ac_
  if (!use_footstep_planner_) {
    return;                     // do nothing
  }
  if (!ac_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "footstep_planner action server is not available; skip planning");
    return;
  }
  bool call_planner = !plan_run_;
  if (call_planner) {
    plan_run_ = true;
    PlanFootsteps::Goal goal;
    jsk_footstep_msgs::msg::FootstepArray goal_footstep;
    goal_footstep.header.frame_id = marker_frame_id_;
    goal_footstep.header.stamp = rclcpp::Time(0);
    jsk_footstep_msgs::msg::Footstep goal_left;
    goal_left.leg = jsk_footstep_msgs::msg::Footstep::LEFT;
    goal_left.pose = getFootstepPose(true);
    goal_left.dimensions.x = foot_size_x_;
    goal_left.dimensions.y = foot_size_y_;
    goal_left.dimensions.z = foot_size_z_;
    jsk_footstep_msgs::msg::Footstep goal_right;
    goal_right.pose = getFootstepPose(false);
    goal_right.leg = jsk_footstep_msgs::msg::Footstep::RIGHT;
    goal_right.dimensions.x = foot_size_x_;
    goal_right.dimensions.y = foot_size_y_;
    goal_right.dimensions.z = foot_size_z_;
    goal_footstep.footsteps.push_back(goal_left);
    goal_footstep.footsteps.push_back(goal_right);
    goal.goal_footstep = goal_footstep;
    jsk_footstep_msgs::msg::FootstepArray initial_footstep;
    initial_footstep.header.frame_id = marker_frame_id_;
    initial_footstep.header.stamp = rclcpp::Time(0);
    // TODO: decide initial footstep by tf
    jsk_footstep_msgs::msg::Footstep initial_left;
    initial_left.leg = jsk_footstep_msgs::msg::Footstep::LEFT;
    initial_left.pose = lleg_initial_pose_;
    initial_left.dimensions.x = foot_size_x_;
    initial_left.dimensions.y = foot_size_y_;
    initial_left.dimensions.z = foot_size_z_;

    jsk_footstep_msgs::msg::Footstep initial_right;
    initial_right.leg = jsk_footstep_msgs::msg::Footstep::RIGHT;
    initial_right.pose = rleg_initial_pose_;
    initial_right.dimensions.x = foot_size_x_;
    initial_right.dimensions.y = foot_size_y_;
    initial_right.dimensions.z = foot_size_z_;
    if (lleg_first_) {
      initial_footstep.footsteps.push_back(initial_left);
      initial_footstep.footsteps.push_back(initial_right);
    }
    else {
      initial_footstep.footsteps.push_back(initial_right);
      initial_footstep.footsteps.push_back(initial_left);
    }
    goal.initial_footstep = initial_footstep;
    auto options = PlanningActionClient::SendGoalOptions();
    options.goal_response_callback = [this](PlanGoalHandle::SharedPtr goal_handle) {
      if (!goal_handle) {
        std::lock_guard<std::mutex> lock(plan_run_mutex_);
        plan_run_ = false;
        RCLCPP_WARN(this->get_logger(), "planning goal was rejected by the footstep planner");
      }
    };
    options.result_callback = std::bind(&FootstepMarker::planDoneCB, this,
                                        std::placeholders::_1);
    ac_->async_send_goal(goal, options);
  }
}

void FootstepMarker::planDoneCB(const PlanGoalHandle::WrappedResult& result)
{
  std::lock_guard<std::mutex> lock(plan_run_mutex_);
  RCLCPP_INFO(this->get_logger(), "planDoneCB");
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result) {
    plan_result_ = result.result;
    footstep_pub_->publish(plan_result_->result);
    RCLCPP_INFO(this->get_logger(), "planning is finished");
  }
  else {
    RCLCPP_WARN(this->get_logger(), "planning did not succeed (result code: %d)",
                static_cast<int>(result.code));
  }
  plan_run_ = false;
}

geometry_msgs::msg::Pose FootstepMarker::getFootstepPose(bool leftp) {
  Eigen::Vector3d offset(0, 0, 0);
  if (leftp) {
    offset[1] = footstep_margin_ / 2.0;
  }
  else {
    offset[1] = - footstep_margin_ / 2.0;
  }
  Eigen::Affine3d marker_origin;
  tf2::fromMsg(marker_pose_.pose, marker_origin);
  Eigen::Affine3d footstep_transform = marker_origin * Eigen::Translation3d(offset);
  return tf2::toMsg(footstep_transform);
}

void FootstepMarker::moveMarkerCB(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  // move the marker
  geometry_msgs::msg::PoseStamped transformed_pose;
  try {
    transformed_pose = tf_buffer_->transform(*msg, marker_frame_id_);
  }
  catch (tf2::TransformException& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lookup transformation: %s", e.what());
    return;
  }
  marker_pose_ = transformed_pose;
  bool skip_plan = false;
  if (use_plane_snap_) {
    // the projection service is called asynchronously; the marker pose is
    // updated and the plan is triggered from the response callback.
    skip_plan = !projectMarkerToPlane();
  }

  // need to solve TF
  server_->setPose("footstep_marker", transformed_pose.pose);
  server_->applyChanges();
  current_pose_pub_->publish(marker_pose_);
  if (!skip_plan) {
    planIfPossible();
  }
}

visualization_msgs::msg::Marker FootstepMarker::makeFootstepMarker(geometry_msgs::msg::Pose pose) {
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = foot_size_x_;
  marker.scale.y = foot_size_y_;
  marker.scale.z = foot_size_z_;
  marker.color.a = 1.0;
  marker.pose = pose;
  return marker;
}

void FootstepMarker::initializeInteractiveMarker() {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = marker_frame_id_;
  int_marker.name = "footstep_marker";
  int_marker.pose = marker_pose_.pose;
  visualization_msgs::msg::Marker left_box_marker = makeFootstepMarker(lleg_pose_);
  left_box_marker.color.g = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl left_box_control;
  left_box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  left_box_control.always_visible = true;
  left_box_control.markers.push_back( left_box_marker );

  int_marker.controls.push_back( left_box_control );

  visualization_msgs::msg::Marker right_box_marker = makeFootstepMarker(rleg_pose_);
  right_box_marker.color.r = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl right_box_control;
  right_box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  right_box_control.always_visible = true;
  right_box_control.markers.push_back( right_box_marker );

  int_marker.controls.push_back( right_box_control );
  if (show_6dof_control_) {
    if (use_2d_) {
      im_helpers::add3Dof2DControl(int_marker, false);
    }
    else {
      im_helpers::add6DofControl(int_marker, false);
    }
  }

  server_->insert(int_marker,
                  std::bind(&FootstepMarker::processFeedbackCB, this, std::placeholders::_1));

  // initial footsteps
  visualization_msgs::msg::InteractiveMarker initial_lleg_int_marker;
  initial_lleg_int_marker.header.frame_id = marker_frame_id_;
  initial_lleg_int_marker.name = "left_initial_footstep_marker";
  initial_lleg_int_marker.pose.orientation.w = 1.0;
  visualization_msgs::msg::Marker initial_left_marker = makeFootstepMarker(lleg_initial_pose_);
  initial_left_marker.color.g = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl initial_left_box_control;
  initial_left_box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  initial_left_box_control.always_visible = true;
  initial_left_box_control.markers.push_back(initial_left_marker);

  initial_lleg_int_marker.controls.push_back( initial_left_box_control );
  server_->insert(initial_lleg_int_marker,
                  std::bind(&FootstepMarker::processFeedbackCB, this, std::placeholders::_1));

  visualization_msgs::msg::InteractiveMarker initial_rleg_int_marker;
  initial_rleg_int_marker.header.frame_id = marker_frame_id_;
  initial_rleg_int_marker.name = "right_initial_footstep_marker";
  initial_rleg_int_marker.pose.orientation.w = 1.0;
  visualization_msgs::msg::Marker initial_right_marker = makeFootstepMarker(rleg_initial_pose_);
  initial_right_marker.color.r = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl initial_right_box_control;
  initial_right_box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  initial_right_box_control.always_visible = true;
  initial_right_box_control.markers.push_back(initial_right_marker);

  initial_rleg_int_marker.controls.push_back( initial_right_box_control );
  server_->insert(initial_rleg_int_marker,
                  std::bind(&FootstepMarker::processFeedbackCB, this, std::placeholders::_1));

  menu_handler_.apply( *server_, "footstep_marker");

  server_->applyChanges();
}

FootstepMarker::~FootstepMarker() {
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto marker = std::make_shared<FootstepMarker>();
  if (rclcpp::ok()) {           // may be interrupted during construction
    rclcpp::spin(marker);
  }
  rclcpp::shutdown();
  return 0;
}
