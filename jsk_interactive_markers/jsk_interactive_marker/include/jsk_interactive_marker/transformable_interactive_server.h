#ifndef __TRANSFORMABLE_INTERACTIVE_SERVER_H__
#define __TRANSFORMABLE_INTERACTIVE_SERVER_H__

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <jsk_interactive_marker/transformable_object.h>
#include <jsk_interactive_marker/yaml_menu_handler.h>
#include <jsk_interactive_marker/parent_and_child_interactive_marker_server.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <mutex>
#include <jsk_rviz_plugins_msgs/msg/overlay_text.hpp>
#include <iostream>
#include <sstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <jsk_interactive_marker_msgs/srv/get_transformable_marker_pose.hpp>
#include <jsk_interactive_marker_msgs/srv/set_transformable_marker_pose.hpp>
#include <jsk_interactive_marker_msgs/srv/get_transformable_marker_color.hpp>
#include <jsk_interactive_marker_msgs/srv/set_transformable_marker_color.hpp>
#include <jsk_interactive_marker_msgs/srv/get_transformable_marker_focus.hpp>
#include <jsk_interactive_marker_msgs/srv/set_transformable_marker_focus.hpp>
#include <jsk_interactive_marker_msgs/srv/get_marker_dimensions.hpp>
#include <jsk_interactive_marker_msgs/srv/set_marker_dimensions.hpp>
#include <jsk_interactive_marker_msgs/srv/get_type.hpp>
#include <jsk_interactive_marker_msgs/srv/get_transformable_marker_existence.hpp>
#include <jsk_interactive_marker_msgs/msg/marker_dimensions.hpp>
#include <jsk_interactive_marker_msgs/msg/pose_stamped_with_name.hpp>
#include <jsk_rviz_plugins_msgs/srv/request_marker_operate.hpp>

using namespace std;

namespace jsk_interactive_marker
{
  class TransformableInteractiveServer: public rclcpp::Node{
  public:
    TransformableInteractiveServer();
    ~TransformableInteractiveServer();

    void processFeedback( visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback );
    void setRadius(std_msgs::msg::Float32 msg);
    void setSmallRadius(std_msgs::msg::Float32 msg);
    void setX(std_msgs::msg::Float32 msg);
    void setY(std_msgs::msg::Float32 msg);
    void setZ(std_msgs::msg::Float32 msg);

    void setPose( const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg_ptr , bool for_interactive_control=false);
    void addPose(geometry_msgs::msg::Pose msg);
    void addPoseRelative(geometry_msgs::msg::Pose msg);

    void setControlRelativePose(geometry_msgs::msg::Pose msg);

    void setColor(std_msgs::msg::ColorRGBA msg);

    void insertNewBox( std::string frame_id, std::string name, std::string description );
    void insertNewCylinder( std::string frame_id, std::string name, std::string description );
    void insertNewTorus( std::string frame_id, std::string name, std::string description );
    void insertNewMesh( std::string frame_id, std::string name, std::string description , std::string mesh_resource, bool mesh_use_embedded_materials);

    void insertNewObject(TransformableObject* tobject, std::string name);
    void eraseObject(std::string name);
    void eraseAllObject();
    void eraseFocusObject();

    void focusTextPublish();
    void focusPosePublish();
    void focusObjectMarkerNamePublish();
    void focusInteractiveManipulatorDisplay();

    void enableInteractiveManipulatorDisplay(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback,
                                             const bool enable);

    void updateTransformableObject(TransformableObject* tobject);

    void getPoseService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Response::SharedPtr res, bool for_interactive_control);
    void setPoseService(const jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Response::SharedPtr res, bool for_interactive_control);
    void getColorService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor::Response::SharedPtr res);
    void setColorService(const jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor::Response::SharedPtr res);
    void getFocusService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus::Response::SharedPtr res);
    void setFocusService(const jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus::Response::SharedPtr res);
    void getTypeService(const jsk_interactive_marker_msgs::srv::GetType::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetType::Response::SharedPtr res);
    void getExistenceService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence::Response::SharedPtr res);
    void setDimensionsService(const jsk_interactive_marker_msgs::srv::SetMarkerDimensions::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetMarkerDimensions::Response::SharedPtr res);
    void getDimensionsService(const jsk_interactive_marker_msgs::srv::GetMarkerDimensions::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetMarkerDimensions::Response::SharedPtr res);
    void hideService(const std_srvs::srv::Empty::Request::SharedPtr req,
                     std_srvs::srv::Empty::Response::SharedPtr res);
    void showService(const std_srvs::srv::Empty::Request::SharedPtr req,
                     std_srvs::srv::Empty::Response::SharedPtr res);
    void publishMarkerDimensions();

    void requestMarkerOperateService(const jsk_rviz_plugins_msgs::srv::RequestMarkerOperate::Request::SharedPtr req, jsk_rviz_plugins_msgs::srv::RequestMarkerOperate::Response::SharedPtr res);
    // parameter callback replacing the dynamic_reconfigure configCallback in ROS 1
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    void declareInteractiveSettingParameters();
    void SetInitialInteractiveMarkerConfig( TransformableObject* tobject );

    void tfTimerCallback();
    bool setPoseWithTfTransformation(TransformableObject* tobject, geometry_msgs::msg::PoseStamped pose_stamped, bool for_interactive_control=false);

    std::string focus_object_marker_name_;

    std::mutex mutex_;

    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr setcolor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setcontrolpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr addpose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr addpose_relative_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr setcontrol_relative_sub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_r_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_sm_r_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_h_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_x_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_y_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_z_sub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hide_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr show_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose>::SharedPtr get_pose_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose>::SharedPtr get_control_pose_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose>::SharedPtr set_pose_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose>::SharedPtr set_control_pose_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor>::SharedPtr get_color_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor>::SharedPtr set_color_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus>::SharedPtr get_focus_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus>::SharedPtr set_focus_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetType>::SharedPtr get_type_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence>::SharedPtr get_exist_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::SetMarkerDimensions>::SharedPtr set_dimensions_srv;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetMarkerDimensions>::SharedPtr get_dimensions_srv;
    rclcpp::Publisher<jsk_interactive_marker_msgs::msg::MarkerDimensions>::SharedPtr marker_dimensions_pub_;
    rclcpp::Service<jsk_rviz_plugins_msgs::srv::RequestMarkerOperate>::SharedPtr request_marker_operate_srv_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setrad_sub_;
    rclcpp::Publisher<jsk_rviz_plugins_msgs::msg::OverlayText>::SharedPtr focus_name_text_pub_;
    rclcpp::Publisher<jsk_rviz_plugins_msgs::msg::OverlayText>::SharedPtr focus_pose_text_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr focus_object_marker_name_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<jsk_interactive_marker_msgs::msg::PoseStampedWithName>::SharedPtr pose_with_name_pub_;
    interactive_markers::InteractiveMarkerServer* server_;
    map<string, TransformableObject*> transformable_objects_map_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int torus_udiv_;
    int torus_vdiv_;
    jsk_interactive_marker::InteractiveSettingConfig config_;
    bool strict_tf_;
    int interactive_manipulator_orientation_;
    rclcpp::TimerBase::SharedPtr tf_timer;
    std::shared_ptr <YamlMenuHandler> yaml_menu_handler_ptr_;
  };
}

#endif
