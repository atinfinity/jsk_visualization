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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "jsk_interactive_marker/interactive_marker_helpers.h"
#include <yaml-cpp/yaml.h>
#include <mutex>

namespace jsk_interactive_marker
{
  class CameraInfoPublisher : public rclcpp::Node
  {
  public:
    typedef std::shared_ptr<CameraInfoPublisher> Ptr;
    CameraInfoPublisher();
    virtual ~CameraInfoPublisher();
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void publishCameraInfo(const rclcpp::Time& stamp);
    virtual void processFeedback(
      visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
    virtual void initializeInteractiveMarker();
    virtual void pointcloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    virtual void imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr msg);
    virtual void staticRateCallback();
    // replacement of the dynamic_reconfigure CameraInfoPublisherConfig
    virtual void declareConfigParameters();
    virtual rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sync_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_sync_image_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::mutex mutex_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ////////////////////////////////////////////////////////
    // variables
    ////////////////////////////////////////////////////////
    std::string frame_id_;
    std::string parent_frame_id_;
    double width_;
    double height_;
    double f_;
    std::string yaml_filename_;
    YAML::Node camera_info_yaml_;
    geometry_msgs::msg::Pose latest_pose_;

  private:

  };
}
