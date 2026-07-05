// -*- mode: c++ -*-
// ROS 2 port of image_view2 node entry point (JSK Lab, BSD license).
#include <memory>
#include <thread>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include "image_view2.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<image_view2::ImageView2>();

  if (node->use_window) {
    // OpenCV HighGUI must be driven from the main thread, so spin the
    // node in a background thread and run the GUI loop here.
    std::thread spin_thread([node]() {rclcpp::spin(node);});
    rclcpp::WallRate rate(30.0);
    while (rclcpp::ok()) {
      int key = cv::waitKey(1000 / 30);
      node->pressKey(key);
      node->showImage();
      rate.sleep();
    }
    spin_thread.join();
  } else {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
