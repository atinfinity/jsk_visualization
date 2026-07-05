#ifndef INTERACTIVE_POINT_CLOUD
#define INTERACTIVE_POINT_CLOUD

#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_movement.hpp>
#include <jsk_recognition_msgs/msg/int32_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

//#include <point_cloud_server/StoreCloudAction.h>
#include <pcl/search/kdtree.h>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

//#include <object_manipulator/tools/mechanism_interface.h>
#include <jsk_interactive_marker/parent_and_child_interactive_marker_server.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mutex>

typedef pcl::PointXYZRGB PointT;


class InteractivePointCloud : public rclcpp::Node
{
public:

  InteractivePointCloud(
                std::string marker_name,
                std::string topic_name, std::string server_name);

  ~InteractivePointCloud();

  //! Clear the cloud stored in this object
  void hide(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void handlePoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &ps);
  void pointCloudAndBoundingBoxCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud, const jsk_recognition_msgs::msg::BoundingBoxArray::ConstSharedPtr &box, const geometry_msgs::msg::PoseStamped::ConstSharedPtr &handle);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud);
  void setHandlePoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &ps);
private:

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::mutex mutex_;

  typedef interactive_markers::MenuHandler MenuHandler;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, jsk_recognition_msgs::msg::BoundingBoxArray, geometry_msgs::msg::PoseStamped> SyncPolicy;
  typedef message_filters::sync_policies::ExactTime<jsk_recognition_msgs::msg::Int32Stamped, geometry_msgs::msg::PoseArray, jsk_recognition_msgs::msg::BoundingBoxArray> SyncHandlePose;


  void makeMenu();

  void makeMarker(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, float size);
  void makeMarker(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, const jsk_recognition_msgs::msg::BoundingBoxArray::ConstSharedPtr box, const geometry_msgs::msg::PoseStamped::ConstSharedPtr handle, float size);

  void menuPoint( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void move( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void leftClickPoint( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void markerFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  void publishGraspPose();
  void publishHandPose( geometry_msgs::msg::PoseStamped box_pose);

  void setMarkerPoseCallback( const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_stamped_msg);

  std::string marker_name_, topic_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_marker_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_click_point_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_left_click_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_left_click_relative_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_handle_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_handle_pose_array_;
  rclcpp::Publisher<jsk_recognition_msgs::msg::BoundingBoxMovement>::SharedPtr pub_box_movement_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_grasp_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_handle_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_marker_pose_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_point_cloud_;
  message_filters::Subscriber<jsk_recognition_msgs::msg::BoundingBoxArray> sub_bounding_box_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_initial_handle_pose_;
  message_filters::Subscriber<jsk_recognition_msgs::msg::Int32Stamped> sub_selected_index_;
  message_filters::Subscriber<geometry_msgs::msg::PoseArray> sub_handle_array_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
  std::shared_ptr<message_filters::Synchronizer<SyncHandlePose> > sync_handle_;

  std::shared_ptr<jsk_interactive_marker::ParentAndChildInteractiveMarkerServer> marker_server_;
  interactive_markers::MenuHandler menu_handler_;

  sensor_msgs::msg::PointCloud2 msg_cloud_;
  double point_size_;
  bool use_bounding_box_;

  geometry_msgs::msg::PoseStamped marker_pose_;
  std::string input_pointcloud_, input_bounding_box_, initial_handle_pose_;

  sensor_msgs::msg::PointCloud2 current_croud_;
  jsk_recognition_msgs::msg::BoundingBoxArray current_box_;
  geometry_msgs::msg::PoseStamped handle_pose_;
  tf2::Transform handle_tf_;
  bool exist_handle_tf_;
  bool display_interactive_manipulator_;

  jsk_recognition_msgs::msg::BoundingBoxMovement box_movement_;
};

#endif
