#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.hpp>

#include <interactive_markers/menu_handler.hpp>
#include <jsk_interactive_marker_msgs/srv/set_pose.hpp>
#include <jsk_interactive_marker_msgs/srv/marker_set_pose.hpp>

#include <math.h>
#include <jsk_interactive_marker_msgs/msg/marker_menu.hpp>
#include <jsk_interactive_marker_msgs/msg/marker_pose.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "urdf_parser/urdf_parser.h"
#include <urdf_world/types.h>

#include <yaml-cpp/yaml.h>

#include <list>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class InteractiveMarkerInterface : public rclcpp::Node {
 private:
  struct MeshProperty{
    std::string link_name;
    std::string mesh_file;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;

  };

  struct UrdfProperty{
    urdf::ModelInterfaceSharedPtr model;
    std::string root_link_name;
    geometry_msgs::msg::Pose pose;
    double scale;
    std_msgs::msg::ColorRGBA color;
    bool use_original_color;
  };

 public:
  visualization_msgs::msg::InteractiveMarker make6DofControlMarker( std::string name, geometry_msgs::msg::PoseStamped &stamped, float scale, bool fixed_position, bool fixed_rotation);

  void proc_feedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void proc_feedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int type );
  void pub_marker_tf ( std_msgs::msg::Header header, geometry_msgs::msg::Pose pose);
  void pub_marker_pose ( std_msgs::msg::Header header, geometry_msgs::msg::Pose pose, std::string name, int type );

  void pub_marker_menu(std::string marker,int menu, int type);
  void pub_marker_menu(std::string marker,int menu);

  void pub_marker_menuCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int menu );

  void pub_marker_menuCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, int menu, int type);

  void IMSizeLargeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void IMSizeMiddleCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void IMSizeSmallCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void changeMoveModeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void changeMoveModeCb1( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void changeMoveModeCb2( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void changeForceModeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void changeForceModeCb1( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void changeForceModeCb2( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );


  void targetPointMenuCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void lookAutomaticallyMenuCB( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void ConstraintCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void modeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void changeMoveArm( std::string m_name, int menu );

  void setOriginCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, bool origin_hand);

  void ikmodeCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void toggleIKModeCb( const std_msgs::msg::Empty::ConstSharedPtr &msg);
  void useTorsoCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void usingIKCb( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );

  void marker_menu_cb( const jsk_interactive_marker_msgs::msg::MarkerMenu::ConstSharedPtr msg);

  void updateHeadGoal( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void updateBase( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void updateFinger( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string hand);

  visualization_msgs::msg::InteractiveMarker makeBaseMarker( const char *name, const geometry_msgs::msg::PoseStamped &stamped, float scale, bool fixed);



  void changeMarkerForceMode( std::string mk_name , int im_mode);

  void toggleStartIKCb( const std_msgs::msg::Empty::ConstSharedPtr &msg);

  void initControlMarkers(void);

  void initBodyMarkers(void);

  void initHandler(void);

  void changeMarkerMoveMode( std::string mk_name , int im_mode);

  void changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size);

  void changeMarkerMoveMode( std::string mk_name , int im_mode, float mk_size, geometry_msgs::msg::PoseStamped dist_pose);

  void changeMarkerOperationModelMode( std::string mk_name );

  void addHandMarker(visualization_msgs::msg::InteractiveMarker &im,std::vector < UrdfProperty > urdf_vec);
  void addSphereMarker(visualization_msgs::msg::InteractiveMarker &im, double scale, std_msgs::msg::ColorRGBA color);
  void makeCenterSphere(visualization_msgs::msg::InteractiveMarker &mk, double mk_size);

  InteractiveMarkerInterface ();

  void markers_set_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Request> req,
                        std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Response> res );

  void markers_del_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Request> req,
                        std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Response> res );

  void move_marker_cb ( const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  void set_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Request> req,
                std::shared_ptr<jsk_interactive_marker_msgs::srv::MarkerSetPose::Response> res );

  void reset_cb ( const std::shared_ptr<jsk_interactive_marker_msgs::srv::SetPose::Request> req,
                  std::shared_ptr<jsk_interactive_marker_msgs::srv::SetPose::Response> res );

  void loadUrdfFromYaml(const YAML::Node &val, std::string name, std::vector<UrdfProperty>& mesh);
  void loadMeshes(const YAML::Node &val);

  void makeIMVisible(visualization_msgs::msg::InteractiveMarker &im);

  // dynamic_tf_publisher replacement
  void publishDynamicTf();

 private:

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<jsk_interactive_marker_msgs::msg::MarkerPose>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_update_;
  rclcpp::Publisher<jsk_interactive_marker_msgs::msg::MarkerMenu>::SharedPtr pub_move_;
  rclcpp::Service<jsk_interactive_marker_msgs::srv::SetPose>::SharedPtr serv_reset_;
  rclcpp::Service<jsk_interactive_marker_msgs::srv::MarkerSetPose>::SharedPtr serv_set_;
  rclcpp::Service<jsk_interactive_marker_msgs::srv::MarkerSetPose>::SharedPtr serv_markers_set_;
  rclcpp::Service<jsk_interactive_marker_msgs::srv::MarkerSetPose>::SharedPtr serv_markers_del_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_marker_pose_;
  rclcpp::Subscription<jsk_interactive_marker_msgs::msg::MarkerMenu>::SharedPtr sub_marker_menu_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_toggle_start_ik_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_toggle_ik_mode_;

  /* dynamic_tf_publisher replacement */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> dynamic_tf_map_;
  std::mutex dynamic_tf_mutex_;
  rclcpp::TimerBase::SharedPtr dynamic_tf_timer_;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler menu_handler1;
  interactive_markers::MenuHandler menu_handler2;
  interactive_markers::MenuHandler menu_handler_force;
  interactive_markers::MenuHandler menu_handler_force1;
  interactive_markers::MenuHandler menu_handler_force2;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle2;
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle_ik;

  interactive_markers::MenuHandler menu_head_;
  interactive_markers::MenuHandler::EntryHandle head_target_handle_;
  interactive_markers::MenuHandler::EntryHandle head_auto_look_handle_;

  interactive_markers::MenuHandler menu_head_target_;

  interactive_markers::MenuHandler menu_base_;
  interactive_markers::MenuHandler menu_finger_r_;
  interactive_markers::MenuHandler menu_finger_l_;

  // parameters
  std::string marker_name;
  std::string server_name;
  std::string base_frame;
  std::string move_base_frame;
  std::string target_frame;
  bool fix_marker;
  interactive_markers::MenuHandler::EntryHandle h_mode_last;
  interactive_markers::MenuHandler::EntryHandle h_mode_last2;
  interactive_markers::MenuHandler::EntryHandle h_mode_last3;

  interactive_markers::MenuHandler::EntryHandle rotation_t_menu_;
  interactive_markers::MenuHandler::EntryHandle rotation_nil_menu_;

  interactive_markers::MenuHandler::EntryHandle use_torso_menu_;
  interactive_markers::MenuHandler::EntryHandle use_torso_t_menu_;
  interactive_markers::MenuHandler::EntryHandle use_torso_nil_menu_;
  interactive_markers::MenuHandler::EntryHandle use_fullbody_menu_;


  interactive_markers::MenuHandler::EntryHandle start_ik_menu_;
  interactive_markers::MenuHandler::EntryHandle stop_ik_menu_;

  int h_mode_rightarm;
  int h_mode_constrained;
  int h_mode_ikmode;
  int use_arm;



  std::list<visualization_msgs::msg::InteractiveMarker> imlist;

  struct GripperState{
  GripperState() : on_(false), view_facing_(false), edit_control_(false), torso_frame_(false) {}

    bool on_;
    bool view_facing_;
    bool edit_control_;
    bool torso_frame_;
  };

  struct ControlState{
  ControlState() : posture_r_(false), posture_l_(false), torso_on_(false), head_on_(false),
      projector_on_(false), init_head_goal_(false), base_on_(true),  r_finger_on_(false), l_finger_on_(false), move_arm_(RARM), move_origin_state_(HAND_ORIGIN) {}

    enum MoveArmState { RARM, LARM, ARMS};
    enum MoveOriginState { HAND_ORIGIN, DESIGNATED_ORIGIN};

    MoveArmState move_arm_;
    MoveOriginState move_origin_state_;

    geometry_msgs::msg::PoseStamped marker_pose_;

    bool posture_r_;
    bool posture_l_;
    bool torso_on_;
    bool head_on_;
    bool projector_on_;
    bool look_auto_on_;
    bool init_head_goal_;
    bool base_on_;
    bool planar_only_;
    bool r_finger_on_;
    bool l_finger_on_;
    GripperState dual_grippers_;
    GripperState r_gripper_;
    GripperState l_gripper_;
  };

  bool use_finger_marker_;
  bool use_body_marker_;
  bool use_center_sphere_;


  ControlState control_state_;

  geometry_msgs::msg::PoseStamped head_goal_pose_;

  std::string hand_type_;

  std::string head_link_frame_;
  std::string head_mesh_;
  std::vector< MeshProperty > rhand_mesh_, lhand_mesh_;
  std::vector< UrdfProperty > rhand_urdf_, lhand_urdf_;

};
