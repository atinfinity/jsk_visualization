#include <jsk_interactive_marker/yaml_menu_handler.h>
#include <fstream>
#include <unistd.h>

using namespace jsk_interactive_marker;
YamlMenuHandler::YamlMenuHandler(rclcpp::Node* node_ptr, std::string file_name) {
  _node_ptr = node_ptr;
  initMenu(file_name);
}

bool YamlMenuHandler::initMenu(std::string file_name) {
  YAML::Node doc;
  RCLCPP_INFO(_node_ptr->get_logger(), "opening yaml file %s", file_name.c_str());
  if ( !(access(file_name.c_str(), F_OK) != -1)) {
    RCLCPP_INFO(_node_ptr->get_logger(), "file not exists :%s", file_name.c_str());
    return false;
  }
  doc = YAML::LoadFile(file_name);
  for (size_t i=0; i<doc.size() ;i++) {
    std::string text, topic_name;
    const YAML::Node& single_menu = doc[i];
    text = single_menu["text"].as<std::string>();
    topic_name = single_menu["topic"].as<std::string>();
    RCLCPP_INFO(_node_ptr->get_logger(), "Regist %s, %s", text.c_str(), topic_name.c_str());
    _publisher_map[topic_name] = _node_ptr->create_publisher<std_msgs::msg::String>(topic_name, 1);
    _menu_handler.insert(
      text,
      [this, topic_name](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
        pubTopic(feedback, topic_name);
      });
  }
  return true;
}

void YamlMenuHandler::pubTopic(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string topic_name) {
  std_msgs::msg::String text_msg;
  text_msg.data = feedback->marker_name;
  if (_publisher_map.find(topic_name)==_publisher_map.end()) {
    return;
  }
  else {
    _publisher_map[topic_name]->publish(text_msg);
  }
}

void YamlMenuHandler::applyMenu(interactive_markers::InteractiveMarkerServer* server_ptr, std::string name) {
  _menu_handler.apply(*server_ptr, name);
}
