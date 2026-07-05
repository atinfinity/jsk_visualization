// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#include "jsk_interactive_marker/camera_info_publisher.h"
#include <sensor_msgs/distortion_models.hpp>
#include <cstring>

namespace jsk_interactive_marker
{
  CameraInfoPublisher::CameraInfoPublisher()
    : rclcpp::Node("camera_info_publisher")
  {
    latest_pose_.orientation.w = 1.0;
    tf_buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(this));
    pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "~/camera_info", 1);
    yaml_filename_ = this->declare_parameter("yaml_filename", std::string(""));
    if (yaml_filename_ == "") {
      RCLCPP_WARN(this->get_logger(),
                  "~yaml_filename is not specified, use default camera info parameters");
    }
    else {
      camera_info_yaml_ = YAML::LoadFile(yaml_filename_);
    }

    // setup declared parameters (dynamic_reconfigure replacement)
    declareConfigParameters();

    // read parameters
    frame_id_ = this->declare_parameter("frame_id", std::string("camera"));
    parent_frame_id_ = this->declare_parameter("parent_frame_id",
                                               std::string("base_link"));

    // interactive marker
    server_.reset(new interactive_markers::InteractiveMarkerServer(
                    this->get_name(), this));
    initializeInteractiveMarker();
    bool sync_pointcloud = this->declare_parameter("sync_pointcloud", false);
    bool sync_image = this->declare_parameter("sync_image", false);

    if (sync_pointcloud) {
      RCLCPP_INFO(this->get_logger(),
                  "~sync_pointcloud is specified, synchronize ~camera_info to pointcloud");
      sub_sync_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input", 1,
        std::bind(&CameraInfoPublisher::pointcloudCallback, this,
                  std::placeholders::_1));
    }
    else {
      if (sync_image) {
        RCLCPP_INFO(this->get_logger(),
                    "~sync_image is specified, synchronize ~camera_info to image");
        sub_sync_image_ = this->create_subscription<sensor_msgs::msg::Image>(
          "~/input", 1,
          std::bind(&CameraInfoPublisher::imageCallback, this,
                    std::placeholders::_1));
      }
      else {
        RCLCPP_INFO(this->get_logger(),
                    "~sync_image or ~sync_pointcloud are not specified, use static_rate");
        double static_rate =
          this->declare_parameter("static_rate", 30.0); // defaults to 30 Hz
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1 / static_rate),
          std::bind(&CameraInfoPublisher::staticRateCallback, this));
      }
    }
  }

  CameraInfoPublisher::~CameraInfoPublisher()
  {

  }

  void CameraInfoPublisher::declareConfigParameters()
  {
    // replacement of the dynamic_reconfigure CameraInfoPublisherConfig
    {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = "width of camera info";
      d.floating_point_range.resize(1);
      d.floating_point_range[0].from_value = 1.0;
      d.floating_point_range[0].to_value = 5000.0;
      width_ = this->declare_parameter("width", 640.0, d);
    }
    {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = "height of camera info";
      d.floating_point_range.resize(1);
      d.floating_point_range[0].from_value = 1.0;
      d.floating_point_range[0].to_value = 5000.0;
      height_ = this->declare_parameter("height", 480.0, d);
    }
    {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = "f of camera_info, used as fx and fy";
      d.floating_point_range.resize(1);
      d.floating_point_range[0].from_value = 1.0;
      d.floating_point_range[0].to_value = 5000.0;
      f_ = this->declare_parameter("f", 525.0, d);
    }
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CameraInfoPublisher::parametersCallback, this,
                std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult CameraInfoPublisher::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters) {
      if (parameter.get_name() == "width") {
        width_ = parameter.as_double();
      }
      else if (parameter.get_name() == "height") {
        height_ = parameter.as_double();
      }
      else if (parameter.get_name() == "f") {
        f_ = parameter.as_double();
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void CameraInfoPublisher::initializeInteractiveMarker()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = parent_frame_id_;
    int_marker.name = "camera info";
    im_helpers::add6DofControl(int_marker, false);
    server_->insert(int_marker,
                    std::bind(&CameraInfoPublisher::processFeedback, this,
                              std::placeholders::_1));
    server_->applyChanges();
  }

  void CameraInfoPublisher::processFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::msg::PoseStamped new_pose, transformed_pose;
    new_pose.pose = feedback->pose;
    new_pose.header = feedback->header;
    try {
      transformed_pose = tf_buffer_->transform(new_pose, parent_frame_id_);
      latest_pose_ = transformed_pose.pose;
    }
    catch (...) {
      RCLCPP_FATAL(this->get_logger(), "tf exception");
      return;
    }
  }

  void CameraInfoPublisher::publishCameraInfo(const rclcpp::Time& stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.stamp = stamp;
    camera_info.header.frame_id = frame_id_;
    if (yaml_filename_ != "") {
      camera_info.height = camera_info_yaml_["image_height"].as<uint32_t>();
      camera_info.width = camera_info_yaml_["image_width"].as<uint32_t>();
      camera_info.distortion_model =
        camera_info_yaml_["camera_model"].as<std::string>();
      std::vector<double> D, K, R, P;
      std::array<double, 9ul> Kl, Rl;
      std::array<double, 12ul> Pl;
      D = camera_info_yaml_["distortion_coefficients"]["data"].as<std::vector<double>>();
      K = camera_info_yaml_["camera_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&Kl[0], &K[0], sizeof(double)*9);
      R = camera_info_yaml_["rectification_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&Rl[0], &R[0], sizeof(double)*9);
      P = camera_info_yaml_["projection_matrix"]["data"].as<std::vector<double>>();
      std::memcpy(&Pl[0], &P[0], sizeof(double)*12);
      camera_info.d = D;
      camera_info.k = Kl;
      camera_info.r = Rl;
      camera_info.p = Pl;
    }
    else {
      camera_info.height = height_;
      camera_info.width = width_;
      camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      camera_info.d.resize(5, 0);
      camera_info.k.fill(0.0);
      camera_info.r.fill(0.0);
      camera_info.p.fill(0.0);
      camera_info.k[0] = camera_info.k[4] = f_;

      camera_info.k[0] = camera_info.p[0] = camera_info.k[4] = camera_info.p[5] = f_;
      camera_info.k[2] = camera_info.p[2] = width_ / 2.0;
      camera_info.k[5] = camera_info.p[6] = height_ / 2.0;
      camera_info.k[8] = camera_info.p[10] = 1.0;
      camera_info.r[0] = camera_info.r[4] = camera_info.r[8] = 1.0;
    }
    pub_camera_info_->publish(camera_info);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame_id_;
    transform.child_frame_id = frame_id_;
    transform.transform.translation.x = latest_pose_.position.x;
    transform.transform.translation.y = latest_pose_.position.y;
    transform.transform.translation.z = latest_pose_.position.z;
    transform.transform.rotation = latest_pose_.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void CameraInfoPublisher::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    publishCameraInfo(msg->header.stamp);
  }

  void CameraInfoPublisher::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    publishCameraInfo(msg->header.stamp);
  }

  void CameraInfoPublisher::staticRateCallback()
  {
    publishCameraInfo(this->now());
  }

}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jsk_interactive_marker::CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
