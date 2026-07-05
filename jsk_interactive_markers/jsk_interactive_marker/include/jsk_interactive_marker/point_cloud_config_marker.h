#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

#include <interactive_markers/menu_handler.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>


class PointCloudConfigMarker : public rclcpp::Node {
 public:
  struct MarkerControlConfig{
    MarkerControlConfig(): marker_id(0), resolution_(0.05){

    }
    MarkerControlConfig(double s): marker_id(0), resolution_(0.05){
      size.x = s;
      size.y = s;
      size.z = s;
    }
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Vector3 size;
    int marker_id;
    double resolution_;
  };

  visualization_msgs::msg::Marker makeBoxMarker(geometry_msgs::msg::Vector3 size);
  visualization_msgs::msg::Marker makeTextMarker(geometry_msgs::msg::Vector3 size);
  visualization_msgs::msg::InteractiveMarker makeBoxInteractiveMarker(MarkerControlConfig mconfig, std::string name);
  void moveBoxCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  visualization_msgs::msg::Marker makeMarkerMsg( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  void publishMarkerMsg( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void cancelCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void clearCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void clearBoxCB( const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void clearBox();

  void changeResolutionCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void changeBoxSize(geometry_msgs::msg::Vector3 size);
  void changeBoxSizeCB(const geometry_msgs::msg::Vector3::ConstSharedPtr &msg);
  void changeBoxSizeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);


  void updateBoxInteractiveMarker();
  void changeBoxResolution(const std_msgs::msg::Float32::ConstSharedPtr &msg);
  void publishCurrentPose(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void publishCurrentPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose);
  void updatePoseCB(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose);
  void addBoxCB(const std_msgs::msg::Empty::ConstSharedPtr &msg);
  interactive_markers::MenuHandler makeMenuHandler();
  PointCloudConfigMarker ();
 private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr latest_feedback_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;


  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_update_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr add_box_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_box_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr change_box_size_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr change_box_resolution_sub_;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler::EntryHandle resolution_menu_;
  interactive_markers::MenuHandler::EntryHandle checked_resolution_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_20cm_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_10cm_menu_;
  interactive_markers::MenuHandler::EntryHandle resolution_5cm_menu_;

  interactive_markers::MenuHandler::EntryHandle box_size_menu_;
  interactive_markers::MenuHandler::EntryHandle checked_box_size_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_100_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_50_menu_;
  interactive_markers::MenuHandler::EntryHandle box_size_25_menu_;


  std::string server_name;
  std::string marker_name;
  std::string base_frame;
  double size_;
  MarkerControlConfig marker_control_config;
};
