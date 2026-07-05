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

#ifndef JSK_INTERACTIVE_MARKER_FOOTSTEP_MARKER_H_
#define JSK_INTERACTIVE_MARKER_FOOTSTEP_MARKER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <jsk_recognition_msgs/msg/simple_occupancy_grid_array.hpp>
#include <jsk_recognition_msgs/srv/call_snap_it.hpp>
#include <jsk_interactive_marker_msgs/msg/snap_foot_print_input.hpp>
#include <jsk_interactive_marker_msgs/srv/snap_foot_print.hpp>
#include <jsk_interactive_marker_msgs/srv/set_heuristic.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <jsk_footstep_msgs/action/plan_footsteps.hpp>
#include <jsk_footstep_msgs/action/exec_footsteps.hpp>

#include <Eigen/Geometry>
#include <mutex>

class FootstepMarker : public rclcpp::Node {
public:
  FootstepMarker();
  virtual ~FootstepMarker();
  void updateInitialFootstep();
  typedef jsk_footstep_msgs::action::PlanFootsteps PlanFootsteps;
  typedef jsk_footstep_msgs::action::ExecFootsteps ExecFootsteps;
  typedef rclcpp_action::Client<PlanFootsteps> PlanningActionClient;
  typedef rclcpp_action::Client<ExecFootsteps> ExecuteActionClient;
  typedef rclcpp_action::ClientGoalHandle<PlanFootsteps> PlanGoalHandle;
  typedef rclcpp_action::ClientGoalHandle<ExecFootsteps> ExecGoalHandle;
  typedef PlanFootsteps::Result PlanResult;
protected:
  void initializeInteractiveMarker();
  void processFeedbackCB(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void menuFeedbackCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void moveMarkerCB(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void menuCommandCB(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void executeCB(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void resumeCB(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void planDoneCB(const PlanGoalHandle::WrappedResult& result);
  void processMenuFeedback(uint8_t id);
  geometry_msgs::msg::Polygon computePolygon(uint8_t leg);
  void snapLegs();
  // requests the transformation of the leg to the snapit service
  // asynchronously; the leg pose is updated in the response callback.
  void computeLegTransformation(uint8_t leg);
  geometry_msgs::msg::Pose getFootstepPose(bool leftp);
  void changePlannerHeuristic(const std::string& heuristic);
  void callEstimateOcclusion();
  void cancelWalk();
  void planIfPossible();
  void resetLegPoses();
  void lookGround();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters);
  void forceToReplan(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                     std::shared_ptr<std_srvs::srv::Empty::Response> res);
  std::mutex plane_mutex_;
  std::mutex plan_run_mutex_;
  std::mutex exec_mutex_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  // projection to the planes
  bool projectMarkerToPlane();

  jsk_recognition_msgs::msg::SimpleOccupancyGridArray::ConstSharedPtr latest_grids_;
  // read a geometry_msgs/pose from the parameter specified.
  // the format of the parameter is [x, y, z, xx, yy, zz, ww].
  // where x, y and z means position and xx, yy, zz and ww means
  // orientation.
  void readPoseParam(const std::string& param, Eigen::Affine3d& offset);

  // execute footstep
  // sending action goal to footstep controller
  void executeFootstep();
  void resumeFootstep();

  void projectionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose);

  visualization_msgs::msg::Marker makeFootstepMarker(geometry_msgs::msg::Pose pose);

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  double foot_size_x_;
  double foot_size_y_;
  double foot_size_z_;
  double footstep_margin_;
  std::string marker_frame_id_;
  geometry_msgs::msg::PoseStamped marker_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr move_marker_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr menu_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr exec_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resume_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr projection_sub_;
  rclcpp::Publisher<jsk_interactive_marker_msgs::msg::SnapFootPrintInput>::SharedPtr project_footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr snapped_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<jsk_footstep_msgs::msg::FootstepArray>::SharedPtr footstep_pub_;
  rclcpp::Client<jsk_recognition_msgs::srv::CallSnapIt>::SharedPtr snapit_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr estimate_occlusion_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr look_ground_client_;
  rclcpp::Client<jsk_interactive_marker_msgs::srv::SetHeuristic>::SharedPtr set_heuristic_client_;
  rclcpp::Client<jsk_interactive_marker_msgs::srv::SnapFootPrint>::SharedPtr project_footprint_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr plan_if_possible_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  PlanningActionClient::SharedPtr ac_;
  ExecuteActionClient::SharedPtr ac_exec_;
  rclcpp::TimerBase::SharedPtr initial_footstep_timer_;
  bool use_projection_service_;
  bool use_projection_topic_;
  bool show_6dof_control_;
  bool use_footstep_planner_;
  bool use_footstep_controller_;
  bool plan_run_;
  bool exec_run_;
  bool use_plane_snap_;
  bool wait_snapit_server_;
  bool use_initial_footstep_tf_;
  bool use_initial_reference_;
  bool always_planning_;
  bool lleg_first_;
  bool use_2d_;
  std::string initial_reference_frame_;
  geometry_msgs::msg::Pose lleg_pose_;
  geometry_msgs::msg::Pose rleg_pose_;
  geometry_msgs::msg::Pose lleg_initial_pose_;
  geometry_msgs::msg::Pose rleg_initial_pose_;
  Eigen::Affine3d lleg_offset_;
  Eigen::Affine3d rleg_offset_;
  std::string lfoot_frame_id_;
  std::string rfoot_frame_id_;

  // footstep plannner result
  PlanResult::SharedPtr plan_result_;
};

#endif // JSK_INTERACTIVE_MARKER_FOOTSTEP_MARKER_H_
