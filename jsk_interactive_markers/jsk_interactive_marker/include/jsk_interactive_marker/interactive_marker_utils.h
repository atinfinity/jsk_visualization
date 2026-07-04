#ifndef _MARKER_UTILS_H_
#define _MARKER_UTILS_H_


#include <rclcpp/rclcpp.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

#include <interactive_markers/menu_handler.hpp>
#include <jsk_interactive_marker_msgs/srv/set_pose.hpp>
#include <jsk_interactive_marker_msgs/srv/marker_set_pose.hpp>

#include <math.h>
#include <jsk_interactive_marker_msgs/msg/marker_menu.hpp>
#include <jsk_interactive_marker_msgs/msg/marker_pose.hpp>

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include "urdf_parser/urdf_parser.h"
#include <fstream>

#include <kdl/frames_io.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include <yaml-cpp/yaml.h>


using namespace urdf;

namespace im_utils{
  geometry_msgs::msg::Transform Pose2Transform( const geometry_msgs::msg::Pose pose_msg);
  geometry_msgs::msg::Pose Transform2Pose( const geometry_msgs::msg::Transform tf_msg);
  geometry_msgs::msg::Pose UrdfPose2Pose( const urdf::Pose pose);

  visualization_msgs::msg::InteractiveMarkerControl makeCylinderMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, double length,  double radius, const std_msgs::msg::ColorRGBA &color, bool use_color);
  visualization_msgs::msg::InteractiveMarkerControl makeBoxMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, Vector3 dim, const std_msgs::msg::ColorRGBA &color, bool use_color);
  visualization_msgs::msg::InteractiveMarkerControl makeSphereMarkerControl(const geometry_msgs::msg::PoseStamped &stamped, double rad, const std_msgs::msg::ColorRGBA &color, bool use_color);
  visualization_msgs::msg::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA &color, bool use_color);
  visualization_msgs::msg::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource, const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale);
  visualization_msgs::msg::InteractiveMarkerControl makeMeshMarkerControl(const std::string &mesh_resource,
                                                                     const geometry_msgs::msg::PoseStamped &stamped, geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA &color);
  void addMeshLinksControl(visualization_msgs::msg::InteractiveMarker &im, LinkConstSharedPtr link, KDL::Frame previous_frame, bool use_color, std_msgs::msg::ColorRGBA color, double scale);
  void addMeshLinksControl(visualization_msgs::msg::InteractiveMarker &im, LinkConstSharedPtr link, KDL::Frame previous_frame, bool use_color, std_msgs::msg::ColorRGBA color, double scale, bool root);

  ModelInterfaceSharedPtr getModelInterface(std::string model_file);
  visualization_msgs::msg::InteractiveMarker makeLinksMarker(LinkConstSharedPtr link, bool use_color, std_msgs::msg::ColorRGBA color, geometry_msgs::msg::PoseStamped marker_ps, geometry_msgs::msg::Pose origin_pose);

  visualization_msgs::msg::InteractiveMarker makeFingerControlMarker(const char *name, geometry_msgs::msg::PoseStamped ps);
  visualization_msgs::msg::InteractiveMarker makeSandiaHandMarker(geometry_msgs::msg::PoseStamped ps);

  visualization_msgs::msg::Marker makeSandiaFinger0Marker(std::string frame_id);
  visualization_msgs::msg::Marker makeSandiaFinger1Marker(std::string frame_id);
  visualization_msgs::msg::Marker makeSandiaFinger2Marker(std::string frame_id);

  visualization_msgs::msg::InteractiveMarker makeSandiaHandInteractiveMarker(geometry_msgs::msg::PoseStamped ps, std::string hand, int finger, int link);

  std::string getRosPathFromModelPath(std::string path);
  std::string getRosPathFromFullPath(std::string path);
  std::string getFullPathFromModelPath(std::string path);
  std::string getFilePathFromRosPath( std::string rospath);

  geometry_msgs::msg::Pose getPose( const YAML::Node &val);
  double getXmlValue( const YAML::Node &val );
}
#endif
