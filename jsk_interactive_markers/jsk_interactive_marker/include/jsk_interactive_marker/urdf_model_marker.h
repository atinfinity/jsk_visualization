#ifndef _URDF_MODEL_MARKER_H_
#define _URDF_MODEL_MARKER_H_


#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

#include <interactive_markers/menu_handler.hpp>
#include <jsk_interactive_marker_msgs/srv/set_pose.hpp>
#include <jsk_interactive_marker_msgs/srv/marker_set_pose.hpp>

#include <math.h>
#include <jsk_interactive_marker_msgs/msg/marker_menu.hpp>
#include <jsk_interactive_marker_msgs/msg/marker_pose.hpp>
#include <jsk_interactive_marker_msgs/msg/move_object.hpp>
#include <jsk_interactive_marker_msgs/msg/move_model.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <jsk_recognition_msgs/msg/int32_stamped.hpp>

#include <urdf_model/types.h>
#include <urdf_world/types.h>

using namespace urdf;
using namespace std;


class UrdfModelMarker {
 public:
  UrdfModelMarker(string model_name, string model_description, string model_file, string frame_id, geometry_msgs::msg::PoseStamped root_pose, geometry_msgs::msg::Pose root_offset, double scale_factor, string mode, bool robot_mode, bool registration, vector<string> fixed_link, bool use_robot_description, bool use_visible_color, map<string, double> initial_pose_map, int index, rclcpp::Node::SharedPtr node, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server);
  UrdfModelMarker();

  void addMoveMarkerControl(visualization_msgs::msg::InteractiveMarker &int_marker, LinkConstSharedPtr link, bool root);
  void addInvisibleMeshMarkerControl(visualization_msgs::msg::InteractiveMarker &int_marker, LinkConstSharedPtr link, const std_msgs::msg::ColorRGBA &color);
  void addGraspPointControl(visualization_msgs::msg::InteractiveMarker &int_marker, std::string link_frame_name_);

  void publishBasePose( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void publishBasePose( geometry_msgs::msg::Pose pose, std_msgs::msg::Header header);
  void publishMarkerPose ( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void publishMarkerPose ( geometry_msgs::msg::Pose pose, std_msgs::msg::Header header, std::string marker_name);
  void publishMarkerMenu( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int menu );
  void publishMoveObject( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void publishJointState( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void republishJointState( sensor_msgs::msg::JointState js);

  void proc_feedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, string parent_frame_id, string frame_id);

  void graspPoint_feedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, string link_name);
  void moveCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void setPoseCB();
  void setPoseCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void hideMarkerCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void hideAllMarkerCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void hideModelMarkerCB( const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void showModelMarkerCB( const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void setUrdfCB( const std_msgs::msg::String::ConstSharedPtr &msg);

  visualization_msgs::msg::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA &color, bool use_color);
  visualization_msgs::msg::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale);
  visualization_msgs::msg::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA &color);



  visualization_msgs::msg::InteractiveMarkerControl makeCylinderMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, double length, double radius, const std_msgs::msg::ColorRGBA &color, bool use_color);
  visualization_msgs::msg::InteractiveMarkerControl makeBoxMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, Vector3 dim, const std_msgs::msg::ColorRGBA &color, bool use_color);
  visualization_msgs::msg::InteractiveMarkerControl makeSphereMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, double rad, const std_msgs::msg::ColorRGBA &color, bool use_color);
  //joint state methods
  void publishJointState();
  void getJointState();
  void getJointState(LinkConstSharedPtr link);

  void setJointState(LinkConstSharedPtr link, const sensor_msgs::msg::JointState::ConstSharedPtr &js);
  void setJointAngle(LinkConstSharedPtr link, double joint_angle);
  geometry_msgs::msg::Pose getRootPose(geometry_msgs::msg::Pose pose);
  geometry_msgs::msg::PoseStamped getOriginPoseStamped();
  void setOriginalPose(LinkConstSharedPtr link);
  void addChildLinkNames(LinkConstSharedPtr link, bool root, bool init);
  void addChildLinkNames(LinkConstSharedPtr link, bool root, bool init, bool use_color, int color_index);

  // dynamic_tf_publisher replacement: keep transforms in a map and
  // broadcast them periodically with tf2_ros::TransformBroadcaster.
  void callSetDynamicTf(string parent_frame_id, string frame_id, geometry_msgs::msg::Transform transform);
  void callPublishTf();
  void publishDynamicTf();

  int main(string file);

  void graspPointCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void jointMoveCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void resetMarkerCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void resetBaseMarkerCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void resetBaseMsgCB( const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void resetBaseCB();

  void resetRobotBase();
  void resetRootForVisualization();
  void registrationCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void setRootPoseCB( const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg );
  void setRootPose( geometry_msgs::msg::PoseStamped ps);
  void resetJointStatesCB( const sensor_msgs::msg::JointState::ConstSharedPtr &msg, bool update_root);
  void lockJointStates(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                       std::shared_ptr<std_srvs::srv::Empty::Response> res);
  void unlockJointStates(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res);

 protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  /* publisher */
  rclcpp::Publisher<jsk_interactive_marker_msgs::msg::MarkerPose>::SharedPtr pub_;
  rclcpp::Publisher<jsk_interactive_marker_msgs::msg::MarkerMenu>::SharedPtr pub_move_;
  rclcpp::Publisher<jsk_interactive_marker_msgs::msg::MoveObject>::SharedPtr pub_move_object_;
  rclcpp::Publisher<jsk_interactive_marker_msgs::msg::MoveModel>::SharedPtr pub_move_model_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_base_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_selected_;
  rclcpp::Publisher<jsk_recognition_msgs::msg::Int32Stamped>::SharedPtr pub_selected_index_;

  /* subscriber */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_reset_joints_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_reset_joints_and_root_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_set_root_pose_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hide_marker_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr show_marker_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_root_pose_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_set_urdf_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr serv_lock_joint_states_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr serv_unlock_joint_states_;
  std::mutex joint_states_mutex_;
  bool is_joint_states_locked_;
  interactive_markers::MenuHandler model_menu_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* dynamic_tf_publisher replacement */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> dynamic_tf_map_;
  std::mutex dynamic_tf_mutex_;
  rclcpp::TimerBase::SharedPtr dynamic_tf_timer_;

  std::string server_name;
  std::string base_frame;
  std::string move_base_frame;
  std::string target_frame;


  ModelInterfaceSharedPtr model;
  std::string model_name_;
  std::string model_description_;
  std::string frame_id_;
  std::string model_file_;
  geometry_msgs::msg::Pose root_pose_;
  geometry_msgs::msg::Pose root_pose_origin_;
  geometry_msgs::msg::Pose root_offset_;
  geometry_msgs::msg::Pose fixed_link_offset_; //used when fixel_link_ is used
  double scale_factor_;
  bool robot_mode_;
  bool registration_;
  bool use_dynamic_tf_;
  string mode_;
  vector<string> fixed_link_;
  bool use_robot_description_;
  bool use_visible_color_;
  std::string tf_prefix_;
  map<string, double> initial_pose_map_;
  int index_;
  builtin_interfaces::msg::Time init_stamp_;

  //joint states
  sensor_msgs::msg::JointState joint_state_;
  sensor_msgs::msg::JointState joint_state_origin_;

  struct graspPoint{
    graspPoint(){
      displayMoveMarker = false;
      displayGraspPoint = false;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
    }
    bool displayGraspPoint;
    bool displayMoveMarker;
    geometry_msgs::msg::Pose pose;
  };


  struct linkProperty{
    linkProperty(){
      displayMoveMarker = false;
      displayModelMarker = true;
      mesh_file = "";
    }
    bool displayMoveMarker;
    bool displayModelMarker;
    graspPoint gp;
    string frame_id;
    string movable_link;
    //pose from frame_id
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Pose origin;
    geometry_msgs::msg::Pose initial_pose;
    urdf::Vector3 joint_axis;
    double joint_angle;
    int rotation_count;
    string mesh_file;
  };

  map<string, linkProperty> linkMarkerMap;
};

#endif
