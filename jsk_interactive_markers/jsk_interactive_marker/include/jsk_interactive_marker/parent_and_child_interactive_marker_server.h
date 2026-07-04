#ifndef JSK_INTERACTIVE_MARKER_PARENT_AND_CHILD_INTERACTIVE_MARKER_SERVER_H_
#define JSK_INTERACTIVE_MARKER_PARENT_AND_CHILD_INTERACTIVE_MARKER_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <std_srvs/srv/empty.hpp>
#include <jsk_interactive_marker_msgs/srv/get_transformable_marker_pose.hpp>
#include <jsk_interactive_marker_msgs/srv/set_parent_marker.hpp>
#include <jsk_interactive_marker_msgs/srv/remove_parent_marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <memory>
#include <map>
#include <string>

namespace jsk_interactive_marker
{
  class ParentMarkerInformation{
  public:
    ParentMarkerInformation(std::string p_t_n, std::string p_m_n, Eigen::Affine3d r_p)
    {
      parent_topic_name = p_t_n;
      parent_marker_name = p_m_n;
      relative_pose = r_p;
    }
    ParentMarkerInformation()
    {
    }
    std::string parent_topic_name;
    std::string parent_marker_name;
    Eigen::Affine3d relative_pose;
  };

  class ParentAndChildInteractiveMarkerServer: public interactive_markers::InteractiveMarkerServer
  {
  public:
    class FeedbackSynthesizer{
    public:
      FeedbackSynthesizer(FeedbackCallback c1, FeedbackCallback c2) {cb1=c1; cb2=c2;}
      FeedbackCallback cb1, cb2;
      void call_func(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {if(cb1) cb1(feedback); if(cb2) cb2(feedback);}
    };

    ParentAndChildInteractiveMarkerServer(const std::string &topic_ns, rclcpp::Node::SharedPtr node);
    void setParentService(const jsk_interactive_marker_msgs::srv::SetParentMarker::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetParentMarker::Response::SharedPtr res);
    void renewPoseWithParent(std::map <std::string, ParentMarkerInformation>::iterator assoc_it_, geometry_msgs::msg::Pose parent_pose, std_msgs::msg::Header parent_header);
    void removeParentService(const jsk_interactive_marker_msgs::srv::RemoveParentMarker::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::RemoveParentMarker::Response::SharedPtr res);
    bool removeParent(std::string child_marker_name);
    bool registerAssociationItself(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::msg::PoseStamped child_pose_stamped);
    // NOTE: in ROS 1 this was a synchronous service call; in ROS 2 the parent
    // pose is requested asynchronously and the association (and, on the first
    // association for a parent topic, the parent subscribers) is registered in
    // the response callback.
    void registerAssociationWithOtherNode(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::msg::PoseStamped child_pose_stamped, bool register_parent_subscriber = false);
    bool registerAssociation(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::msg::PoseStamped child_pose_stamped, geometry_msgs::msg::PoseStamped parent_pose_stamped);
    bool getMarkerPose(std::string target_name, geometry_msgs::msg::PoseStamped &pose_stamped);
    void getMarkerPoseService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Response::SharedPtr res);
    void registerParentSubscribers(std::string parent_topic_name);
    void selfFeedbackCb(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
    void parentUpdateCb(const visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr update, std::string parent_topic_name);
    void parentFeedbackCb(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback, std::string parent_topic_name);
    // overwrite public functions
    void applyChanges(); // selfUpdateDb
    bool setCallback(const std::string &name, FeedbackCallback feedback_cb, uint8_t feedback_type=DEFAULT_FEEDBACK_CB);
    void insert(const visualization_msgs::msg::InteractiveMarker &int_marker);
    void insert(const visualization_msgs::msg::InteractiveMarker &int_marker, FeedbackCallback feedback_cb, uint8_t feedback_type=DEFAULT_FEEDBACK_CB);
    bool erase(const std::string &name);
    rclcpp::Node::SharedPtr node_;

    rclcpp::Service<jsk_interactive_marker_msgs::srv::SetParentMarker>::SharedPtr set_parent_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::RemoveParentMarker>::SharedPtr remove_parent_srv_;
    rclcpp::Service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose>::SharedPtr get_marker_pose_srv_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string topic_server_name_;
    // map of association including self association
    std::map <std::string, ParentMarkerInformation> association_list_;
    // map of subscriber
    // access: parent server -> subscriber
    std::map <std::string, rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr> parent_update_subscribers_; // self association is not included
    std::map <std::string, rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr> parent_feedback_subscribers_; // self association is not included
    std::map <std::string, rclcpp::Client<jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose>::SharedPtr> get_parent_pose_clients_;
    std::map <std::string, std::shared_ptr <FeedbackSynthesizer> > callback_map_;
    std::map <std::string, int> parent_subscriber_nums_;
  };
}

#endif
