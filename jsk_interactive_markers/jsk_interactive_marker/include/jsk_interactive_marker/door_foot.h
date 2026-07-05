#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.hpp>

#include <yaml-cpp/yaml.h>

class DoorFoot : public rclcpp::Node {
 public:
  void procAnimation();
  visualization_msgs::msg::Marker makeRWallMarker();
  visualization_msgs::msg::Marker makeLWallMarker();
  visualization_msgs::msg::Marker makeDoorMarker();
  visualization_msgs::msg::Marker makeKnobMarker();
  visualization_msgs::msg::Marker makeKnobMarker(int position);

  visualization_msgs::msg::Marker makeRFootMarker();
  visualization_msgs::msg::Marker makeLFootMarker();

  visualization_msgs::msg::Marker makeFootMarker(geometry_msgs::msg::Pose pose, bool right);
  visualization_msgs::msg::InteractiveMarker makeInteractiveMarker();
  void moveBoxCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void pushDoorCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void pullDoorCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void showStandLocationCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void showNextStepCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void showPreviousStepCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void updateBoxInteractiveMarker();
  interactive_markers::MenuHandler makeMenuHandler();

  DoorFoot ();
 private:
  bool footstep_show_initial_p_;
  int footstep_index_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  std::string server_name;
  std::string marker_name;

  interactive_markers::MenuHandler menu_handler;
  double size_;
  bool push;
  bool use_color_knob;
  std::vector<geometry_msgs::msg::PoseStamped> foot_list;
  geometry_msgs::msg::Pose door_pose;
};

geometry_msgs::msg::Pose getPose( const YAML::Node& val);
double getYamlValue( const YAML::Node& val );
