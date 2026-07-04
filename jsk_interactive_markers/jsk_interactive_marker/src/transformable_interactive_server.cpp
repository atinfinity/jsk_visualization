#include <jsk_interactive_marker/transformable_interactive_server.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace jsk_interactive_marker;

TransformableInteractiveServer::TransformableInteractiveServer(): rclcpp::Node("transformable_interactive_server"){
  torus_udiv_ = this->declare_parameter("torus_udiv", 20);
  torus_vdiv_ = this->declare_parameter("torus_vdiv", 20);
  strict_tf_ = this->declare_parameter("strict_tf", false);
  declareInteractiveSettingParameters();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  setpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/set_pose", 1,
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { setPose(msg, false); });
  setcontrolpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/set_control_pose", 1,
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { setPose(msg, true); });
  setcolor_sub_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "~/set_color", 1,
    [this](const std_msgs::msg::ColorRGBA::ConstSharedPtr msg) { setColor(*msg); });

  set_r_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "~/set_radius", 1,
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { setRadius(*msg); });
  set_sm_r_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "~/set_small_radius", 1,
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { setSmallRadius(*msg); });
  set_x_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "~/set_x", 1,
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { setX(*msg); });
  set_y_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "~/set_y", 1,
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { setY(*msg); });
  set_z_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "~/set_z", 1,
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { setZ(*msg); });

  addpose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "~/add_pose", 1,
    [this](const geometry_msgs::msg::Pose::ConstSharedPtr msg) { addPose(*msg); });
  addpose_relative_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "~/add_pose_relative", 1,
    [this](const geometry_msgs::msg::Pose::ConstSharedPtr msg) { addPoseRelative(*msg); });

  setcontrol_relative_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "~/set_control_relative_pose", 1,
    [this](const geometry_msgs::msg::Pose::ConstSharedPtr msg) { setControlRelativePose(*msg); });

  focus_name_text_pub_ = this->create_publisher<jsk_rviz_plugins_msgs::msg::OverlayText>("~/focus_marker_name_text", 1);
  focus_pose_text_pub_ = this->create_publisher<jsk_rviz_plugins_msgs::msg::OverlayText>("~/focus_marker_pose_text", 1);
  focus_object_marker_name_pub_ = this->create_publisher<std_msgs::msg::String>("~/focus_object_marker_name", 1);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 1);
  pose_with_name_pub_ = this->create_publisher<jsk_interactive_marker_msgs::msg::PoseStampedWithName>("~/pose_with_name", 1);

  get_pose_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose>(
    "~/get_pose",
    [this](const jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Response::SharedPtr res) { getPoseService(req, res, false); });
  get_control_pose_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose>(
    "~/get_control_pose",
    [this](const jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Response::SharedPtr res) { getPoseService(req, res, true); });
  set_pose_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose>(
    "~/set_pose",
    [this](const jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Response::SharedPtr res) { setPoseService(req, res, false); });
  set_control_pose_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose>(
    "~/set_control_pose",
    [this](const jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Response::SharedPtr res) { setPoseService(req, res, true); });
  get_color_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor>(
    "~/get_color",
    [this](const jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor::Response::SharedPtr res) { getColorService(req, res); });
  set_color_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor>(
    "~/set_color",
    [this](const jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor::Response::SharedPtr res) { setColorService(req, res); });
  get_focus_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus>(
    "~/get_focus",
    [this](const jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus::Response::SharedPtr res) { getFocusService(req, res); });
  set_focus_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus>(
    "~/set_focus",
    [this](const jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus::Response::SharedPtr res) { setFocusService(req, res); });
  get_type_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::GetType>(
    "~/get_type",
    [this](const jsk_interactive_marker_msgs::srv::GetType::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetType::Response::SharedPtr res) { getTypeService(req, res); });
  get_exist_srv_ = this->create_service<jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence>(
    "~/get_existence",
    [this](const jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence::Response::SharedPtr res) { getExistenceService(req, res); });
  set_dimensions_srv = this->create_service<jsk_interactive_marker_msgs::srv::SetMarkerDimensions>(
    "~/set_dimensions",
    [this](const jsk_interactive_marker_msgs::srv::SetMarkerDimensions::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetMarkerDimensions::Response::SharedPtr res) { setDimensionsService(req, res); });
  get_dimensions_srv = this->create_service<jsk_interactive_marker_msgs::srv::GetMarkerDimensions>(
    "~/get_dimensions",
    [this](const jsk_interactive_marker_msgs::srv::GetMarkerDimensions::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetMarkerDimensions::Response::SharedPtr res) { getDimensionsService(req, res); });
  hide_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/hide",
    [this](const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) { hideService(req, res); });
  show_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/show",
    [this](const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) { showService(req, res); });
  marker_dimensions_pub_ = this->create_publisher<jsk_interactive_marker_msgs::msg::MarkerDimensions>("~/marker_dimensions", 1);
  request_marker_operate_srv_ = this->create_service<jsk_rviz_plugins_msgs::srv::RequestMarkerOperate>(
    "~/request_marker_operate",
    [this](const jsk_rviz_plugins_msgs::srv::RequestMarkerOperate::Request::SharedPtr req, jsk_rviz_plugins_msgs::srv::RequestMarkerOperate::Response::SharedPtr res) { requestMarkerOperateService(req, res); });

  tf_timer = this->create_wall_timer(
    std::chrono::duration<double>(0.05),
    std::bind(&TransformableInteractiveServer::tfTimerCallback, this));

  // initialize yaml-menu-handler
  std::string yaml_filename = this->declare_parameter("yaml_filename", std::string(""));
  yaml_menu_handler_ptr_ = std::make_shared <YamlMenuHandler> (this, yaml_filename);
  yaml_menu_handler_ptr_->_menu_handler.insert(
    "enable manipulator",
    [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
      enableInteractiveManipulatorDisplay(feedback, /*enable=*/true);
    });
  yaml_menu_handler_ptr_->_menu_handler.insert(
    "disable manipulator",
    [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
      enableInteractiveManipulatorDisplay(feedback, /*enable=*/false);
    });

  bool use_parent_and_child = this->declare_parameter("use_parent_and_child", false);
  if (use_parent_and_child)
  {
    RCLCPP_INFO(this->get_logger(), "initialize parent and child marker");
    // non-owning shared_ptr: the server is a member of this node, so it never
    // outlives the node (shared_from_this() is not usable in a constructor)
    rclcpp::Node::SharedPtr node_ptr(this, [](rclcpp::Node*){});
    server_ = new jsk_interactive_marker::ParentAndChildInteractiveMarkerServer("simple_marker", node_ptr);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "initialize simple marker");
    server_ = new interactive_markers::InteractiveMarkerServer("simple_marker", this);
  }
}

TransformableInteractiveServer::~TransformableInteractiveServer()
{
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    delete itpairstri->second;
  }
  transformable_objects_map_.clear();
  delete server_;
}

void TransformableInteractiveServer::declareInteractiveSettingParameters()
{
  config_.display_interactive_manipulator =
    this->declare_parameter("display_interactive_manipulator", true);
  config_.display_interactive_manipulator_only_selected =
    this->declare_parameter("display_interactive_manipulator_only_selected", false);
  config_.display_description_only_selected =
    this->declare_parameter("display_description_only_selected", false);
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = "Orientation of the interactive manipulator";
    d.additional_constraints = "0: INHERIT, 1: FIXED, 2: VIEW_FACING";
    d.integer_range.resize(1);
    d.integer_range[0].from_value = 0;
    d.integer_range[0].to_value = 2;
    d.integer_range[0].step = 1;
    config_.interactive_manipulator_orientation =
      this->declare_parameter("interactive_manipulator_orientation", 0, d);
    interactive_manipulator_orientation_ = config_.interactive_manipulator_orientation;
  }
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = "Interaction mode of the marker (visualization_msgs/InteractiveMarkerControl)";
    d.integer_range.resize(1);
    d.integer_range[0].from_value = 0;
    d.integer_range[0].to_value = 9;
    d.integer_range[0].step = 1;
    config_.interaction_mode = this->declare_parameter("interaction_mode", 7, d);
  }
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TransformableInteractiveServer::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult TransformableInteractiveServer::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const rclcpp::Parameter& parameter : parameters) {
    if (parameter.get_name() == "display_interactive_manipulator") {
      config_.display_interactive_manipulator = parameter.as_bool();
    } else if (parameter.get_name() == "display_interactive_manipulator_only_selected") {
      config_.display_interactive_manipulator_only_selected = parameter.as_bool();
    } else if (parameter.get_name() == "display_description_only_selected") {
      config_.display_description_only_selected = parameter.as_bool();
    } else if (parameter.get_name() == "interactive_manipulator_orientation") {
      config_.interactive_manipulator_orientation = parameter.as_int();
      interactive_manipulator_orientation_ = config_.interactive_manipulator_orientation;
    } else if (parameter.get_name() == "interaction_mode") {
      config_.interaction_mode = parameter.as_int();
    }
  }
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    TransformableObject* tobject = itpairstri->second;
    tobject->setInteractiveMarkerSetting(config_);
    updateTransformableObject(tobject);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}


void TransformableInteractiveServer::processFeedback(
                                                     visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback )
{
  switch ( feedback->event_type )
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
      focus_object_marker_name_ = feedback->marker_name;
      focusTextPublish();
      focusPosePublish();
      focusObjectMarkerNamePublish();
      focusInteractiveManipulatorDisplay();
      break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      {
        TransformableObject* tobject = transformable_objects_map_[feedback->marker_name.c_str()];
        if(tobject){
          geometry_msgs::msg::PoseStamped input_pose_stamped;
          input_pose_stamped.header = feedback->header;
          input_pose_stamped.pose = feedback->pose;
          setPoseWithTfTransformation(tobject, input_pose_stamped, true);
        }else{
          RCLCPP_ERROR(this->get_logger(), "Invalid ObjectId Request Received %s", feedback->marker_name.c_str());
        }
        focusTextPublish();
        focusPosePublish();
        focusObjectMarkerNamePublish();
      }
      break;
    }
}

void TransformableInteractiveServer::setColor(std_msgs::msg::ColorRGBA msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->setRGBA(msg.r, msg.g, msg.b, msg.a);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::setRadius(std_msgs::msg::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setRadius(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setSmallRadius(std_msgs::msg::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setSmallRadius(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setX(std_msgs::msg::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setX(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setY(std_msgs::msg::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setY(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::setZ(std_msgs::msg::Float32 msg)
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  if(tobject->setZ(msg)){
    updateTransformableObject(tobject);
    publishMarkerDimensions();
  }
}

void TransformableInteractiveServer::updateTransformableObject(TransformableObject* tobject)
{
  visualization_msgs::msg::InteractiveMarker int_marker = tobject->getInteractiveMarker();
  server_->insert(int_marker, std::bind( &TransformableInteractiveServer::processFeedback, this, std::placeholders::_1));
  yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
  server_->applyChanges();
}

void TransformableInteractiveServer::setPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg_ptr, bool for_interactive_control){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject =  transformable_objects_map_[focus_object_marker_name_];
  setPoseWithTfTransformation(tobject, *msg_ptr, for_interactive_control);
  std_msgs::msg::Header header = msg_ptr->header;
  header.frame_id = tobject->getFrameId();
  server_->setPose(focus_object_marker_name_, tobject->pose_, header);
  yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
  server_->applyChanges();
}

void TransformableInteractiveServer::getPoseService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerPose::Response::SharedPtr res, bool for_interactive_control)
{
  TransformableObject* tobject;
  geometry_msgs::msg::PoseStamped transformed_pose_stamped;
  if(req->target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[req->target_name];
  }
  transformed_pose_stamped.header.stamp = this->now();
  transformed_pose_stamped.header.frame_id = tobject->frame_id_;
  transformed_pose_stamped.pose = tobject->getPose(for_interactive_control);
  res->pose_stamped = transformed_pose_stamped;
}

void TransformableInteractiveServer::setPoseService(const jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerPose::Response::SharedPtr res, bool for_interactive_control)
{
  (void)res;
  TransformableObject* tobject;
  if(req->target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) { return; }
    focus_object_marker_name_ = req->target_name;
    tobject = transformable_objects_map_[req->target_name];
  }
  if(setPoseWithTfTransformation(tobject, req->pose_stamped, for_interactive_control)){
    std_msgs::msg::Header header = req->pose_stamped.header;
    header.frame_id = tobject->getFrameId();
    server_->setPose(focus_object_marker_name_, tobject->pose_, header);
    yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
    server_->applyChanges();
  }
}

void TransformableInteractiveServer::getColorService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerColor::Response::SharedPtr res)
{
  TransformableObject* tobject;
  if(req->target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[req->target_name];
  }
  tobject->getRGBA(res->color.r, res->color.g, res->color.b, res->color.a);
}

void TransformableInteractiveServer::setColorService(const jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerColor::Response::SharedPtr res)
{
  (void)res;
  TransformableObject* tobject;
  if(req->target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[req->target_name];
  }
  tobject->setRGBA(req->color.r, req->color.g, req->color.b, req->color.a);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::getFocusService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus::Response::SharedPtr res)
{
  (void)req;
  res->target_name = focus_object_marker_name_;
}

void TransformableInteractiveServer::setFocusService(const jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetTransformableMarkerFocus::Response::SharedPtr res)
{
  (void)res;
  focus_object_marker_name_ = req->target_name;
  focusTextPublish();
  focusPosePublish();
  focusObjectMarkerNamePublish();
}

void TransformableInteractiveServer::getTypeService(const jsk_interactive_marker_msgs::srv::GetType::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetType::Response::SharedPtr res)
{
  TransformableObject* tobject;
  if(req->target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
    res->type = tobject->type_;
  }else{
    if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[req->target_name];
    res->type = tobject->type_;
  }
}

void TransformableInteractiveServer::getExistenceService(const jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetTransformableMarkerExistence::Response::SharedPtr res)
{
  if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) {
    res->existence = false;
  } else {
    res->existence = true;
  }
}

void TransformableInteractiveServer::setDimensionsService(const jsk_interactive_marker_msgs::srv::SetMarkerDimensions::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::SetMarkerDimensions::Response::SharedPtr res)
{
  (void)res;
  TransformableObject* tobject;
  if(req->target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[req->target_name];
  }
  if (tobject) {
    if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_BOX) {
      tobject->setXYZ(req->dimensions.x, req->dimensions.y, req->dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_CYLINDER) {
      tobject->setRZ(req->dimensions.radius, req->dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_TORUS) {
      tobject->setRSR(req->dimensions.radius, req->dimensions.small_radius);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_MESH_RESOURCE) {
    }
    publishMarkerDimensions();
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::getDimensionsService(const jsk_interactive_marker_msgs::srv::GetMarkerDimensions::Request::SharedPtr req, jsk_interactive_marker_msgs::srv::GetMarkerDimensions::Response::SharedPtr res)
{
  TransformableObject* tobject;
  if(req->target_name.compare(std::string("")) == 0){
    if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[focus_object_marker_name_];
  }else{
    if (transformable_objects_map_.find(req->target_name) == transformable_objects_map_.end()) { return; }
    tobject = transformable_objects_map_[req->target_name];
  }
  if (tobject) {
    if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_BOX) {
      tobject->getXYZ(res->dimensions.x, res->dimensions.y, res->dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_CYLINDER) {
      tobject->getRZ(res->dimensions.radius, res->dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_TORUS) {
      tobject->getRSR(res->dimensions.radius, res->dimensions.small_radius);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_MESH_RESOURCE) {
    }

    res->dimensions.type = tobject->getType();
  }
}

void TransformableInteractiveServer::hideService(const std_srvs::srv::Empty::Request::SharedPtr req,
                                                 std_srvs::srv::Empty::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin();
       itpairstri != transformable_objects_map_.end();
       ++itpairstri) {
    TransformableObject* tobject = itpairstri->second;
    tobject->setDisplayInteractiveManipulator(false);
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::showService(const std_srvs::srv::Empty::Request::SharedPtr req,
                                                 std_srvs::srv::Empty::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin();
       itpairstri != transformable_objects_map_.end();
       ++itpairstri) {
    TransformableObject* tobject = itpairstri->second;
    tobject->setDisplayInteractiveManipulator(true);
    updateTransformableObject(tobject);
  }
}
void TransformableInteractiveServer::publishMarkerDimensions()
{
  TransformableObject* tobject;
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  tobject = transformable_objects_map_[focus_object_marker_name_];
  if (tobject) {
    jsk_interactive_marker_msgs::msg::MarkerDimensions marker_dimensions;
    if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_BOX) {
      tobject->getXYZ(marker_dimensions.x, marker_dimensions.y, marker_dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_CYLINDER) {
      tobject->getRZ(marker_dimensions.radius, marker_dimensions.z);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_TORUS) {
      tobject->getRSR(marker_dimensions.radius, marker_dimensions.small_radius);
    } else if (tobject->getType() == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_MESH_RESOURCE) {
    }
    marker_dimensions.type = tobject->type_;
    marker_dimensions_pub_->publish(marker_dimensions);
  }
}

void TransformableInteractiveServer::requestMarkerOperateService(const jsk_rviz_plugins_msgs::srv::RequestMarkerOperate::Request::SharedPtr req, jsk_rviz_plugins_msgs::srv::RequestMarkerOperate::Response::SharedPtr res)
{
  (void)res;
  switch(req->operate.action){
  case jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_INSERT:
    // validation
    if (req->operate.name.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Non empty name is required to insert object.");
      return;
    }

    if (req->operate.type == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_BOX) {
      insertNewBox(req->operate.frame_id, req->operate.name, req->operate.description);
    } else if (req->operate.type == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_CYLINDER) {
      insertNewCylinder(req->operate.frame_id, req->operate.name, req->operate.description);
    } else if (req->operate.type == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_TORUS) {
      insertNewTorus(req->operate.frame_id, req->operate.name, req->operate.description);
    } else if (req->operate.type == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_MESH_RESOURCE) {
      insertNewMesh(req->operate.frame_id, req->operate.name, req->operate.description, req->operate.mesh_resource, req->operate.mesh_use_embedded_materials);
    }
    return;
  case jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_ERASE:
    eraseObject(req->operate.name);
    return;
  case jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_ERASE_ALL:
    eraseAllObject();
    return;
  case jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_ERASE_FOCUS:
    eraseFocusObject();
    return;
  case jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_COPY:
    {
      if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
      TransformableObject *tobject = transformable_objects_map_[focus_object_marker_name_], *new_tobject;
      if (tobject->type_ == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_BOX) {
        float x, y, z;
        tobject->getXYZ(x, y, z);
        insertNewBox(tobject->frame_id_, req->operate.name, req->operate.description);
        new_tobject = transformable_objects_map_[req->operate.name];
        new_tobject->setXYZ(x, y, z);
        new_tobject->setPose(tobject->getPose());
      } else if (tobject->type_ == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_CYLINDER) {
        float r, z;
        tobject->getRZ(r, z);
        insertNewCylinder(tobject->frame_id_, req->operate.name, req->operate.description);
        new_tobject = transformable_objects_map_[req->operate.name];
        new_tobject->setRZ(r, z);
        new_tobject->setPose(tobject->getPose());
      } else if (tobject->type_ == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_TORUS) {
        float r, sr;
        tobject->getRSR(r, sr);
        insertNewTorus(tobject->frame_id_, req->operate.name, req->operate.description);
        new_tobject = transformable_objects_map_[req->operate.name];
        new_tobject->setRSR(r, sr);
        new_tobject->setPose(tobject->getPose());
      } else if (tobject->type_ == jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_MESH_RESOURCE) {
        insertNewMesh(tobject->frame_id_, req->operate.name, req->operate.description, req->operate.mesh_resource, req->operate.mesh_use_embedded_materials);
        new_tobject = transformable_objects_map_[req->operate.name];
        new_tobject->setPose(tobject->getPose());
      } else {
        return;
      }
      float r, g, b, a;
      tobject->getRGBA(r, g, b, a);
      new_tobject->setRGBA(r, g, b, a);
    }
    return;
  };
}

void TransformableInteractiveServer::addPose(geometry_msgs::msg::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->addPose(msg,false);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::addPoseRelative(geometry_msgs::msg::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->addPose(msg,true);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::setControlRelativePose(geometry_msgs::msg::Pose msg){
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  geometry_msgs::msg::Pose pose = tobject->getPose(); //reserve marker pose
  tobject->control_offset_pose_ = msg;
  updateTransformableObject(tobject);
  tobject->setPose(pose);
  std_msgs::msg::Header header;
  header.frame_id = tobject->getFrameId();
  header.stamp = this->now();
  server_->setPose(focus_object_marker_name_, tobject->pose_, header);
  yaml_menu_handler_ptr_->applyMenu(server_, focus_object_marker_name_);
  server_->applyChanges();
}

void TransformableInteractiveServer::enableInteractiveManipulatorDisplay(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback,
    const bool enable) {
  (void)feedback;
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->setDisplayInteractiveManipulator(enable);
  updateTransformableObject(tobject);
}

void TransformableInteractiveServer::focusInteractiveManipulatorDisplay() {
  for (std::map<string, TransformableObject* >::iterator it = transformable_objects_map_.begin();
        it != transformable_objects_map_.end(); it++) {
    std::string object_name = it->first;
    TransformableObject* tobject = it->second;
    if (config_.display_interactive_manipulator && config_.display_interactive_manipulator_only_selected) {
      // display interactive manipulator only for the focused object
      tobject->setDisplayInteractiveManipulator(object_name == focus_object_marker_name_);
    }
    if (config_.display_description_only_selected) {
      // display description only for the focused object
      tobject->setDisplayDescription(object_name == focus_object_marker_name_);
    }
    updateTransformableObject(tobject);
  }
}

void TransformableInteractiveServer::focusTextPublish(){
  jsk_rviz_plugins_msgs::msg::OverlayText focus_text;
  focus_text.text = focus_object_marker_name_;
  focus_text.top = 0;
  focus_text.left = 0;
  focus_text.width = 300;
  focus_text.height = 50;
  focus_text.bg_color.r = 0.9;
  focus_text.bg_color.b = 0.9;
  focus_text.bg_color.g = 0.9;
  focus_text.bg_color.a = 0.1;
  focus_text.fg_color.r = 0.3;
  focus_text.fg_color.g = 0.3;
  focus_text.fg_color.b = 0.8;
  focus_text.fg_color.a = 1;
  focus_text.line_width = 1;
  focus_text.text_size = 30;
  focus_name_text_pub_->publish(focus_text);
}

void TransformableInteractiveServer::focusPosePublish(){
  geometry_msgs::msg::Pose target_pose;
  std::stringstream ss;
  if (transformable_objects_map_.find(focus_object_marker_name_) != transformable_objects_map_.end()) {
    target_pose = transformable_objects_map_[focus_object_marker_name_]->getPose();
    ss << "Pos x: " << target_pose.position.x  << " y: " << target_pose.position.y << " z: " << target_pose.position.z
       << std::endl
       << "Ori x: " << target_pose.orientation.x << " y: " << target_pose.orientation.y << " z: " << target_pose.orientation.z << " w: " << target_pose.orientation.w;
  }

  jsk_rviz_plugins_msgs::msg::OverlayText focus_pose;
  focus_pose.text = ss.str();
  focus_pose.top = 50;
  focus_pose.left = 0;
  focus_pose.width = 500;
  focus_pose.height = 50;
  focus_pose.bg_color.r = 0.9;
  focus_pose.bg_color.b = 0.9;
  focus_pose.bg_color.g = 0.9;
  focus_pose.bg_color.a = 0.1;
  focus_pose.fg_color.r = 0.8;
  focus_pose.fg_color.g = 0.3;
  focus_pose.fg_color.b = 0.3;
  focus_pose.fg_color.a = 1;
  focus_pose.line_width = 1;
  focus_pose.text_size = 15;
  focus_pose_text_pub_->publish(focus_pose);
}

void TransformableInteractiveServer::focusObjectMarkerNamePublish(){
  std_msgs::msg::String msg;
  msg.data = focus_object_marker_name_;
  focus_object_marker_name_pub_->publish(msg);
}

void TransformableInteractiveServer::insertNewBox(std::string frame_id, std::string name, std::string description)
{
  TransformableBox* transformable_box = new TransformableBox(0.45, 0.45, 0.45, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_box, name);
}

void TransformableInteractiveServer::insertNewCylinder( std::string frame_id, std::string name, std::string description)
{
  TransformableCylinder* transformable_cylinder = new TransformableCylinder(0.45, 0.45, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_cylinder, name);
}

void TransformableInteractiveServer::insertNewTorus( std::string frame_id, std::string name, std::string description)
{
  TransformableTorus* transformable_torus = new TransformableTorus(0.45, 0.2, torus_udiv_, torus_vdiv_, 0.5, 0.5, 0.5, 1.0, frame_id, name, description);
  insertNewObject(transformable_torus, name);
}

void TransformableInteractiveServer::insertNewMesh( std::string frame_id, std::string name, std::string description, std::string mesh_resource, bool mesh_use_embedded_materials)
{
  TransformableMesh* transformable_mesh = new TransformableMesh(frame_id, name, description, mesh_resource, mesh_use_embedded_materials);
  insertNewObject(transformable_mesh, name);
}

void TransformableInteractiveServer::insertNewObject( TransformableObject* tobject , std::string name )
{
  SetInitialInteractiveMarkerConfig(tobject);
  visualization_msgs::msg::InteractiveMarker int_marker = tobject->getInteractiveMarker();
  transformable_objects_map_[name] = tobject;
  server_->insert(int_marker, std::bind( &TransformableInteractiveServer::processFeedback, this, std::placeholders::_1));
  yaml_menu_handler_ptr_->applyMenu(server_, name);
  server_->applyChanges();

  focus_object_marker_name_ = name;
  focusTextPublish();
  focusPosePublish();
  focusObjectMarkerNamePublish();
}

void TransformableInteractiveServer::SetInitialInteractiveMarkerConfig( TransformableObject* tobject )
{
  InteractiveSettingConfig config(config_);
  if (config.display_interactive_manipulator && !config.display_interactive_manipulator_only_selected) {
    config.display_interactive_manipulator = true;
  } else {
    config.display_interactive_manipulator = false;
  }
  tobject->setInteractiveMarkerSetting(config);
}

void TransformableInteractiveServer::eraseObject( std::string name )
{
  server_->erase(name);
  server_->applyChanges();
  if (focus_object_marker_name_.compare(name) == 0) {
    focus_object_marker_name_ = "";
    focusTextPublish();
    focusPosePublish();
    focusObjectMarkerNamePublish();
  }
  delete transformable_objects_map_[name];
  transformable_objects_map_.erase(name);
}

void TransformableInteractiveServer::eraseAllObject()
{
  // collect the names first: eraseObject() erases from the map
  std::vector<std::string> names;
  for (std::map<string, TransformableObject* >::iterator itpairstri = transformable_objects_map_.begin(); itpairstri != transformable_objects_map_.end(); itpairstri++) {
    names.push_back(itpairstri->first);
  }
  for (size_t i = 0; i < names.size(); i++) {
    eraseObject(names[i]);
  }
}

void TransformableInteractiveServer::eraseFocusObject()
{
  eraseObject(focus_object_marker_name_);
}

void TransformableInteractiveServer::tfTimerCallback()
{
  if (transformable_objects_map_.find(focus_object_marker_name_) == transformable_objects_map_.end()) { return; }
  TransformableObject* tobject = transformable_objects_map_[focus_object_marker_name_];
  tobject->publishTF(*tf_broadcaster_, this->now());
}

bool TransformableInteractiveServer::setPoseWithTfTransformation(TransformableObject* tobject, geometry_msgs::msg::PoseStamped pose_stamped, bool for_interactive_control)
{
  try {
    geometry_msgs::msg::PoseStamped transformed_pose_stamped;
    jsk_interactive_marker_msgs::msg::PoseStampedWithName transformed_pose_stamped_with_name;
    rclcpp::Time stamp;
    if (strict_tf_) {
      stamp = pose_stamped.header.stamp;
    }
    else {
      stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      pose_stamped.header.stamp = stamp;
    }
    if (!strict_tf_ || tf_buffer_->canTransform(tobject->getFrameId(),
                                                pose_stamped.header.frame_id, stamp, rclcpp::Duration::from_seconds(1.0))) {
      transformed_pose_stamped = tf_buffer_->transform(pose_stamped, tobject->getFrameId());
      tobject->setPose(transformed_pose_stamped.pose, for_interactive_control);
      transformed_pose_stamped.pose=tobject->getPose(true);
      pose_pub_->publish(transformed_pose_stamped);
      transformed_pose_stamped_with_name.pose = transformed_pose_stamped;
      transformed_pose_stamped_with_name.name = tobject->name_;
      //transformed_pose_stamped_with_name.name = focus_object_marker_name_;
      pose_with_name_pub_->publish(transformed_pose_stamped_with_name);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "failed to lookup transform %s -> %s", tobject->getFrameId().c_str(),
                   pose_stamped.header.frame_id.c_str());
      return false;
    }
  }
  catch (tf2::TransformException &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", e.what());
    return false;
  }
  return true;
}
