#ifndef __YAML_MENU_HANDLER_H__
#define __YAML_MENU_HANDLER_H__

#include <vector>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>

namespace jsk_interactive_marker {
  class YamlMenuHandler {
   public:
    rclcpp::Node* _node_ptr;
    interactive_markers::MenuHandler _menu_handler;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> _publisher_map;
    YamlMenuHandler(rclcpp::Node* node_ptr, std::string file_name);
    bool initMenu(std::string file);
    void pubTopic(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string topic_name);
    void applyMenu(interactive_markers::InteractiveMarkerServer* server_ptr, std::string name);
  };
};

#endif
