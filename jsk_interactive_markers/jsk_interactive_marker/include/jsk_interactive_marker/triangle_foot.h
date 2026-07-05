#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <jsk_interactive_marker/interactive_marker_helpers.h>

#include <interactive_markers/menu_handler.hpp>

class TriangleFoot : public rclcpp::Node {
 public:
  visualization_msgs::msg::Marker makeTriangleMarker();
  visualization_msgs::msg::Marker makeRFootMarker();
  visualization_msgs::msg::Marker makeLFootMarker();
  visualization_msgs::msg::Marker makeFootMarker(geometry_msgs::msg::Pose pose);
  visualization_msgs::msg::InteractiveMarker makeInteractiveMarker();
  void moveBoxCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void reverseTriangleCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void updateBoxInteractiveMarker();
  interactive_markers::MenuHandler makeMenuHandler();

  TriangleFoot ();
 private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  std::string server_name;
  std::string marker_name;

  interactive_markers::MenuHandler menu_handler;
  double size_;
  bool reverse;
};
