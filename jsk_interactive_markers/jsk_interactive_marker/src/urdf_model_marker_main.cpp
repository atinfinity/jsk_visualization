// urdf_model_marker_main: read a models config and instantiate
// UrdfModelMarker instances.
//
// In ROS 1 the configuration was passed via the structured rosparam
// "~model_config".  In ROS 2 this is replaced by the string parameter
// "models_config_file" which points to a YAML file with the same layout:
//
//   models:                             # (or a top-level sequence)
//     - name: sample_model              # required
//       description: "sample"          # optional
//       scale: 1.02                     # optional, marker scale factor
//       pose:                           # optional, initial root pose
//         position: {x: 0.0, y: 0.0, z: 0.0}
//         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
//       offset:                         # optional, root offset pose
//         position: {x: 0.0, y: 0.0, z: 0.0}
//         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
//       use_visible_color: false        # optional
//       frame-id: map                   # required, parent frame of the model
//       registration: false             # optional, sets mode to registration
//       fixed_link: link_name           # optional, string or list of strings
//       model: package://pkg/model.urdf # URDF file ("package://", "model://"
//                                       # or full path)
//       use_robot_description: false    # optional; if true the URDF is read
//                                       # from the "robot_description" node
//                                       # parameter instead of a file
//       model_param: robot_description  # optional; read the URDF from this
//                                       # node parameter
//       robot: false                    # optional, robot mode
//       mode: model                     # optional: model/robot/visualization/
//                                       # registration
//       initial_joint_state:            # optional
//         - name: joint1
//           position: 0.0
//       display: true                   # optional, display marker at startup
//
// The URDF read from a parameter follows the rviz2/robot_state_publisher
// convention (declare_parameter of e.g. "robot_description").

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <memory>
#include <vector>
#include <interactive_markers/tools.hpp>
#include <jsk_interactive_marker/urdf_model_marker.h>
#include <jsk_interactive_marker/interactive_marker_utils.h>
#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace urdf;
using namespace std;
using namespace im_utils;

class UrdfModelSettings {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr display_marker_sub_;
  YAML::Node model_config_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  string model_name_;
  string model_description_;
  double scale_factor_;
  geometry_msgs::msg::PoseStamped pose_stamped_;
  geometry_msgs::msg::Pose root_offset_;
  bool use_visible_color_;
  string mode_;
  string model_file_;
  bool registration_;
  vector<string> fixed_link_;
  string frame_id_;
  bool use_robot_description_;
  bool robot_mode_;
  bool display_;
  map<string, double> initial_pose_map_;

  typedef std::shared_ptr<UrdfModelMarker> umm_ptr;
  typedef vector<umm_ptr> umm_vec;
  umm_vec umm_vec_;

public:
  void displayMarkerArrayCB(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg) {
    std_msgs::msg::Header header = msg->header;
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;

    int msg_size = msg->poses.size();
    int umm_vec_size = umm_vec_.size();

    if (msg_size <= umm_vec_size) {
      for (int i = 0; i < msg_size; i++) {
        //change pose
        ps.pose = msg->poses[i];
        umm_vec_[i]->setRootPose(ps);
      }
      for (int i = msg_size; i< umm_vec_size; i++) {
        //move far away
        ps.pose.position.x = ps.pose.position.y = ps.pose.position.z = 1000000;
        ps.pose.orientation.w = 1;
        umm_vec_[i]->setRootPose(ps);
      }
    }
    else {
      for (int i = 0; i < umm_vec_size; i++) {
        //change pose
        ps.pose = msg->poses[i];
        umm_vec_[i]->setRootPose(ps);
      }
      for (int i = umm_vec_size; i < msg_size; i++) {
        ps.pose = msg->poses[i];
        umm_vec_.push_back(umm_ptr(new UrdfModelMarker(model_name_, model_description_, model_file_, header.frame_id, ps, root_offset_, scale_factor_, mode_ , robot_mode_, registration_,fixed_link_, use_robot_description_, use_visible_color_, initial_pose_map_, i, node_, server_)));
      }
    }
  }

  void init() {
    //name
    model_name_ = model_config_["name"].as<std::string>();

    //description
    model_description_ = "";
    if (model_config_["description"]) {
      model_description_ = model_config_["description"].as<std::string>();
    }
    //scale
    scale_factor_ = 1.02;
    if (model_config_["scale"]) {
      scale_factor_ = getXmlValue(model_config_["scale"]);
    }
    //pose
    if (model_config_["pose"]) {
      pose_stamped_.pose = getPose(model_config_["pose"]);
    }
    else {
      pose_stamped_.pose = geometry_msgs::msg::Pose();
      pose_stamped_.pose.orientation.w = 1.0;
    }
    pose_stamped_.header.stamp = node_->now();

    if (model_config_["offset"]) {
      root_offset_ = getPose(model_config_["offset"]);
    }
    else {
      root_offset_ = geometry_msgs::msg::Pose();
      root_offset_.orientation.w = 1.0;
    }

    //color
    use_visible_color_ = false;
    if (model_config_["use_visible_color"]) {
      use_visible_color_ = model_config_["use_visible_color"].as<bool>();
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "use_visible_color: " << use_visible_color_);
    //frame id
    frame_id_ = model_config_["frame-id"].as<std::string>();

    //mode
    mode_ = "model";
    model_file_ = "";
    registration_ = false;
    if (model_config_["registration"]) {
      registration_ = model_config_["registration"].as<bool>();
      mode_ = "registration";
    }

    if (model_config_["fixed_link"]) {
      YAML::Node fixed_links = model_config_["fixed_link"];
      if (fixed_links.IsScalar()) {
        fixed_link_.push_back(fixed_links.as<std::string>());
      }
      else if (fixed_links.IsSequence()) {
        for(size_t i=0; i< fixed_links.size(); i++) {
          fixed_link_.push_back(fixed_links[i].as<std::string>());
        }
      }
    }

    if (model_config_["model"]) {
      model_file_ = model_config_["model"].as<std::string>();
    }
    use_robot_description_ = false;
    if (model_config_["use_robot_description"]) {
      model_file_ = "robot_description";
      use_robot_description_ = model_config_["use_robot_description"].as<bool>();
    }
    if (model_config_["model_param"]) {
      use_robot_description_ = true;
      model_file_ = model_config_["model_param"].as<std::string>();
    }
    robot_mode_ = false;
    if (model_config_["robot"]) {
      robot_mode_ = model_config_["robot"].as<bool>();
      if (robot_mode_) {
        mode_ = "robot";
      }
    }
    if (model_config_["mode"]) {
      mode_ = model_config_["mode"].as<std::string>();
    }

    //initial pose
    if (model_config_["initial_joint_state"]) {
      YAML::Node initial_pose = model_config_["initial_joint_state"];
      for(size_t i=0; i< initial_pose.size(); i++) {
        YAML::Node v = initial_pose[i];
        string name;
        double position;
        if (v["name"] && v["position"]) {
          name = v["name"].as<std::string>();
          position = getXmlValue(v["position"]);
          initial_pose_map_[name] = position;
        }
      }
    }

    //default display
    if (model_config_["display"]) {
      display_ = model_config_["display"].as<bool>();
    }
    else {
      display_ = true;
    }
    RCLCPP_INFO(node_->get_logger(), "Loading model config");
    RCLCPP_INFO(node_->get_logger(), "model_name: %s", model_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "model_description: %s", model_description_.c_str());
    RCLCPP_INFO(node_->get_logger(), "scale_factor: %f", scale_factor_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "frame_id: " << frame_id_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "mode: " << mode_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "registration: " << registration_);

    RCLCPP_INFO_STREAM(node_->get_logger(), "model_file: " << model_file_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "use_robot_description: " << use_robot_description_);
  }

  void addUrdfMarker() {
    umm_vec_.push_back(umm_ptr(new UrdfModelMarker(model_name_, model_description_, model_file_, frame_id_, pose_stamped_ ,root_offset_, scale_factor_, mode_ , robot_mode_, registration_,fixed_link_, use_robot_description_, use_visible_color_,initial_pose_map_, -1, node_, server_)));
  }

  UrdfModelSettings(YAML::Node model, rclcpp::Node::SharedPtr node, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server) : node_(node) {
    model_config_ = model;
    server_ = server;
    init();

    if (display_) {
      addUrdfMarker();
    }
    display_marker_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "~/" + model_name_ + "/pose_array", 1,
      std::bind(&UrdfModelSettings::displayMarkerArrayCB, this, std::placeholders::_1));
  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("jsk_model_marker_interface");

  string server_name = node->declare_parameter("server_name", std::string(""));
  if (server_name == "") {
    server_name = node->get_name();
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  server.reset(new interactive_markers::InteractiveMarkerServer(server_name, node));

  std::string models_config_file =
    node->declare_parameter("models_config_file", std::string(""));

  std::vector<std::shared_ptr<UrdfModelSettings> > model_settings;
  if (models_config_file.empty()) {
    RCLCPP_WARN(node->get_logger(),
                "~models_config_file is not specified; no model marker is created");
  }
  else {
    try {
      YAML::Node config = YAML::LoadFile(models_config_file);
      YAML::Node models;
      if (config.IsSequence()) {
        models = config;
      }
      else if (config["models"]) {
        models = config["models"];
      }
      else if (config["model_config"]) {  // ROS 1 rosparam layout
        models = config["model_config"];
      }
      if (!models.IsDefined() || !models.IsSequence()) {
        RCLCPP_ERROR(node->get_logger(),
                     "%s does not contain a sequence of model configurations",
                     models_config_file.c_str());
      }
      else {
        for (size_t i = 0; i < models.size(); i++) {
          model_settings.push_back(
            std::make_shared<UrdfModelSettings>(models[i], node, server));
        }
      }
    }
    catch (const YAML::Exception &e) {
      RCLCPP_ERROR(node->get_logger(), "failed to load %s: %s",
                   models_config_file.c_str(), e.what());
    }
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
