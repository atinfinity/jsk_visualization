/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include <interactive_markers/interactive_marker_server.hpp>
#include <jsk_recognition_msgs/msg/int32_stamped.hpp>
#include <jsk_interactive_marker_msgs/srv/index_request.hpp>
#include <jsk_recognition_msgs/msg/bounding_box.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>


class BoundingBoxMarker : public rclcpp::Node
{
public:
  BoundingBoxMarker() : rclcpp::Node("bounding_box_interactive_marker")
  {
    server_.reset(new interactive_markers::InteractiveMarkerServer("bounding_box_interactive_marker", this));
    pub_ = this->create_publisher<jsk_recognition_msgs::msg::Int32Stamped>("~/selected_index", 1);
    box_pub_ = this->create_publisher<jsk_recognition_msgs::msg::BoundingBox>("~/selected_box", 1);
    box_arr_pub_ = this->create_publisher<jsk_recognition_msgs::msg::BoundingBoxArray>("~/selected_box_array", 1);
    sub_ = this->create_subscription<jsk_recognition_msgs::msg::BoundingBoxArray>(
      "~/bounding_box_array", 1,
      std::bind(&BoundingBoxMarker::boxCallback, this, std::placeholders::_1));
  }

protected:
  void publishClickedBox(jsk_recognition_msgs::msg::Int32Stamped& msg)
  {
    pub_->publish(msg);
    box_pub_->publish(box_msg_->boxes[msg.data]);
    jsk_recognition_msgs::msg::BoundingBoxArray array_msg;
    array_msg.header = box_msg_->header;
    array_msg.boxes.push_back(box_msg_->boxes[msg.data]);
    box_arr_pub_->publish(array_msg);
  }

  void processFeedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
  {
    // control_name is "sec nsec index"
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
      std::string control_name = feedback->control_name;
      RCLCPP_INFO_STREAM(this->get_logger(), "control_name: " << control_name);
      std::list<std::string> splitted_string;
      boost::split(splitted_string, control_name, boost::is_space());
      jsk_recognition_msgs::msg::Int32Stamped index;
      index.header.stamp.sec = boost::lexical_cast<int>(splitted_string.front());
      splitted_string.pop_front();
      index.header.stamp.nanosec = boost::lexical_cast<unsigned int>(splitted_string.front());
      splitted_string.pop_front();
      index.data = boost::lexical_cast<int>(splitted_string.front());
      publishClickedBox(index);
    }
  }

  // NOTE: as in the ROS 1 version, this service callback is defined but the
  // service itself is not advertised.
  void indexRequest(const std::shared_ptr<jsk_interactive_marker_msgs::srv::IndexRequest::Request> req,
                    std::shared_ptr<jsk_interactive_marker_msgs::srv::IndexRequest::Response> res)
  {
    (void)res;
    jsk_recognition_msgs::msg::Int32Stamped index = req->index;
    publishClickedBox(index);
  }

  void boxCallback(const jsk_recognition_msgs::msg::BoundingBoxArray::ConstSharedPtr msg)
  {
    box_msg_ = msg;
    server_->clear();
    // create cube markers
    for (size_t i = 0; i < msg->boxes.size(); i++) {
      jsk_recognition_msgs::msg::BoundingBox box = msg->boxes[i];
      visualization_msgs::msg::InteractiveMarker int_marker;
      int_marker.header.frame_id = box.header.frame_id;
      int_marker.pose = box.pose;
      {
        std::stringstream ss;
        ss << "box" << "_" << i;
        int_marker.name = ss.str();
      }
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;

      {
        std::stringstream ss;
        // encode several informations into control name
        ss << box.header.stamp.sec << " " << box.header.stamp.nanosec << " " << i;
        control.name = ss.str();
      }
      visualization_msgs::msg::Marker marker;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = box.dimensions.x + 0.01;
      marker.scale.y = box.dimensions.y + 0.01;
      marker.scale.z = box.dimensions.z + 0.01;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.0;
      control.markers.push_back(marker);
      control.always_visible = true;
      int_marker.controls.push_back(control);
      server_->insert(int_marker);
      server_->setCallback(int_marker.name,
                           std::bind(&BoundingBoxMarker::processFeedback, this, std::placeholders::_1));
    }
    server_->applyChanges();
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<jsk_recognition_msgs::msg::Int32Stamped>::SharedPtr pub_;
  rclcpp::Publisher<jsk_recognition_msgs::msg::BoundingBox>::SharedPtr box_pub_;
  rclcpp::Publisher<jsk_recognition_msgs::msg::BoundingBoxArray>::SharedPtr box_arr_pub_;
  rclcpp::Subscription<jsk_recognition_msgs::msg::BoundingBoxArray>::SharedPtr sub_;
  jsk_recognition_msgs::msg::BoundingBoxArray::ConstSharedPtr box_msg_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundingBoxMarker>());
  rclcpp::shutdown();
  return 0;
}
