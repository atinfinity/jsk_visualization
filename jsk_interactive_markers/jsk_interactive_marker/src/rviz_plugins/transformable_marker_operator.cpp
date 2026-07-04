#include <iostream>

#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QLabel>
#include <QTimer>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "jsk_interactive_marker/rviz_plugins/transformable_marker_operator.h"

namespace jsk_interactive_marker
{
  namespace
  {
    // NOTE: resource_retriever is not available as a dependency of this
    // package in ROS 2, so resolve file:// and package:// URLs directly.
    QPixmap loadPixmapFromResource(const std::string& url)
    {
      QPixmap pixmap;
      std::string path;
      if (url.rfind("file://", 0) == 0) {
        path = url.substr(7);
      }
      else if (url.rfind("package://", 0) == 0) {
        std::string rest = url.substr(10);
        size_t pos = rest.find('/');
        if (pos != std::string::npos) {
          try {
            path = ament_index_cpp::get_package_share_directory(rest.substr(0, pos)) + rest.substr(pos);
          }
          catch (const std::exception&) {
            return pixmap;
          }
        }
      }
      else {
        path = url;
      }
      if (!path.empty()) {
        pixmap.load(QString::fromStdString(path));
      }
      return pixmap;
    }
  }
  TransformableMarkerOperatorAction::TransformableMarkerOperatorAction( QWidget* parent )
    : rviz_common::Panel( parent ),
      update_timer_(NULL),
      focus_request_pending_(false),
      dimensions_request_pending_(false)
  {
    layout = new QVBoxLayout;

    // server name
    QHBoxLayout* server_name_layout = new QHBoxLayout;
    server_name_layout->addWidget( new QLabel( "Server Name:" ));
    server_name_editor_ = new QLineEdit;
    server_name_layout->addWidget( server_name_editor_ );
    layout->addLayout( server_name_layout );

    // topic name
    QHBoxLayout* obj_array_topic_layout = new QHBoxLayout;
    obj_array_topic_layout->addWidget( new QLabel( "ObjectArray Topic:" ));
    topic_name_editor_ = new QLineEdit;
    obj_array_topic_layout->addWidget( topic_name_editor_ );
    layout->addLayout( obj_array_topic_layout );

    // tabs for operations
    QTabWidget* tabs = new QTabWidget();

    QVBoxLayout* layout1 = new QVBoxLayout;
    QVBoxLayout* layout2 = new QVBoxLayout;
    QVBoxLayout* layout3 = new QVBoxLayout;

    QWidget* tab_1 = new QWidget();
    QWidget* tab_2 = new QWidget();
    QWidget* tab_3 = new QWidget();

    // tab_1, layout1: insert

    insert_box_button_ = new QPushButton("Insert New Box Marker");
    layout1->addWidget( insert_box_button_ );

    insert_cylinder_button_ = new QPushButton("Insert New Cylinder Marker");
    layout1->addWidget( insert_cylinder_button_ );

    insert_torus_button_ = new QPushButton("Insert New Torus Marker");
    layout1->addWidget( insert_torus_button_ );

    insert_mesh_button_ = new QPushButton("Insert New Mesh Marker");
    layout1->addWidget( insert_mesh_button_ );

    QHBoxLayout* object_layout = new QHBoxLayout;
    object_layout->addWidget( new QLabel( "Object:" ));
    object_editor_ = new QComboBox;
    object_editor_->setIconSize(QSize(50, 50));
    object_editor_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    object_layout->addWidget( object_editor_ );
    layout1->addLayout( object_layout );

    QHBoxLayout* name_layout = new QHBoxLayout;
    name_layout->addWidget( new QLabel( "Name:" ));
    name_editor_ = new QLineEdit;
    name_layout->addWidget( name_editor_ );
    layout1->addLayout( name_layout );

    QHBoxLayout* description_layout = new QHBoxLayout;
    description_layout->addWidget( new QLabel( "Description:" ));
    description_editor_ = new QLineEdit;
    description_layout->addWidget( description_editor_ );
    layout1->addLayout( description_layout );

    QHBoxLayout* frame_layout = new QHBoxLayout;
    frame_layout->addWidget( new QLabel( "Frame:" ));
    frame_editor_ = new QLineEdit;
    frame_layout->addWidget( frame_editor_ );
    layout1->addLayout( frame_layout );

    // tab_2, layout2: transform
    QVBoxLayout* transform_layout = new QVBoxLayout;
    transform_layout->addWidget( new QLabel( "Object Name:" ));
    transform_name_editor_ = new QLineEdit;
    transform_layout->addWidget( transform_name_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension X:" ));
    dimension_x_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_x_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Y:" ));
    dimension_y_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_y_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Z:" ));
    dimension_z_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_z_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Radius:" ));
    dimension_radius_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_radius_editor_ );
    transform_layout->addWidget( new QLabel( "Dimension Small Radius:" ));
    dimension_sm_radius_editor_ = new QLineEdit;
    transform_layout->addWidget( dimension_sm_radius_editor_ );
    layout2->addLayout( transform_layout );

    // tab_3, layout3: erase

    erase_with_id_button_ = new QPushButton("Erase with id");
    layout3->addWidget( erase_with_id_button_ );

    QHBoxLayout* id_layout = new QHBoxLayout;
    id_layout->addWidget( new QLabel( "Id:" ));
    id_editor_ = new QLineEdit;
    id_layout->addWidget( id_editor_ );
    layout3->addLayout( id_layout );

    erase_all_button_ = new QPushButton("Erase all");
    layout3->addWidget( erase_all_button_ );

    erase_focus_button_ = new QPushButton("Erase focus");
    layout3->addWidget( erase_focus_button_ );

    tab_1->setLayout( layout1 );
    tab_2->setLayout( layout2 );
    tab_3->setLayout( layout3 );

    tabs->addTab(tab_1, QString("Insert"));
    tabs->addTab(tab_2, QString("Transform"));
    tabs->addTab(tab_3, QString("Erase"));

    layout->addWidget( tabs );
    setLayout( layout );

    connect( insert_box_button_, SIGNAL( clicked() ), this, SLOT( insertBoxService ()));
    connect( insert_cylinder_button_, SIGNAL( clicked() ), this, SLOT( insertCylinderService ()));
    connect( insert_torus_button_, SIGNAL( clicked() ), this, SLOT( insertTorusService ()));
    connect( insert_mesh_button_, SIGNAL( clicked() ), this, SLOT( insertMeshService ()));
    connect( erase_with_id_button_, SIGNAL( clicked() ), this, SLOT( eraseWithIdService ()));
    connect( erase_all_button_, SIGNAL( clicked() ), this, SLOT( eraseAllService ()));
    connect( erase_focus_button_, SIGNAL( clicked() ), this, SLOT( eraseFocusService ()));

    connect( dimension_x_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_y_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_z_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_radius_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( dimension_sm_radius_editor_, SIGNAL( editingFinished() ), this, SLOT( updateDimensionsService ()));
    connect( object_editor_, SIGNAL( currentIndexChanged(int)), SLOT( updateName ()));
    connect( topic_name_editor_, SIGNAL( editingFinished() ), this, SLOT( updateObjectArrayTopic ()));
  }

  void TransformableMarkerOperatorAction::objectArrayCb(const jsk_recognition_msgs::msg::ObjectArray::ConstSharedPtr obj_array_msg) {
    objects_ = obj_array_msg->objects;
    int current_index = object_editor_->currentIndex();
    object_editor_->clear();
    for (size_t i = 0; i < obj_array_msg->objects.size(); i++) {
      jsk_recognition_msgs::msg::Object object =  objects_[i];
      // thumbnail
      QPixmap pixmap = QPixmap();
      if (object.image_resources.size() > 0)
      {
        std::string thumbnail = object.image_resources[0];
        pixmap = loadPixmapFromResource(thumbnail);
      }
      // name
      std::stringstream ss;
      ss << object.id << ": " << object.name;
      //
      object_editor_->addItem(QIcon(pixmap), QString::fromStdString(ss.str()));
    }
    object_editor_->setCurrentIndex(current_index);
  }

  void TransformableMarkerOperatorAction::onInitialize() {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    ensureServiceClients();
    // In ROS 1 update() was connected to the preUpdate signal of the
    // VisualizationManager; use a periodic Qt timer instead.
    update_timer_ = new QTimer(this);
    connect( update_timer_, SIGNAL( timeout() ), this, SLOT( update() ));
    update_timer_->start(1000);
    updateObjectArrayTopic();
  }

  void TransformableMarkerOperatorAction::ensureServiceClients() {
    if (!nh_) return;
    std::string server_name = server_name_editor_->text().toStdString();
    if (request_marker_operate_client_ && server_name == client_server_name_) {
      return;
    }
    client_server_name_ = server_name;
    request_marker_operate_client_ = nh_->create_client<jsk_rviz_plugins_msgs::srv::RequestMarkerOperate>(
      server_name + "/request_marker_operate");
    set_dimensions_client_ = nh_->create_client<jsk_interactive_marker_msgs::srv::SetMarkerDimensions>(
      server_name + "/set_dimensions");
    get_dimensions_client_ = nh_->create_client<jsk_interactive_marker_msgs::srv::GetMarkerDimensions>(
      server_name + "/get_dimensions");
    get_focus_client_ = nh_->create_client<jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus>(
      server_name + "/get_focus");
    focus_request_pending_ = false;
    dimensions_request_pending_ = false;
  }

  void TransformableMarkerOperatorAction::update() {
    if (!nh_) return;
    updateServerName();
    ensureServiceClients();
    updateFocusMarkerDimensions();
    updateFrameId();
    // updateDimensionsService();
  }

  void TransformableMarkerOperatorAction::updateObjectArrayTopic() {
    if (!nh_) return;
    sub_obj_array_.reset();
    std::string topic = topic_name_editor_->text().toStdString();
    if (topic.empty()) {
      std::map<std::string, std::vector<std::string> > topics = nh_->get_topic_names_and_types();
      for (std::map<std::string, std::vector<std::string> >::iterator it = topics.begin();
           it != topics.end(); ++it) {
        if (std::find(it->second.begin(), it->second.end(),
                      "jsk_recognition_msgs/msg/ObjectArray") != it->second.end()) {
          topic = it->first;
          break;
        }
      }
      topic_name_editor_->setText(QString::fromStdString(topic));
    }
    if (!topic.empty()) {
      sub_obj_array_ = nh_->create_subscription<jsk_recognition_msgs::msg::ObjectArray>(
        topic, 1,
        std::bind(&TransformableMarkerOperatorAction::objectArrayCb, this, std::placeholders::_1));
    }
  }

  void TransformableMarkerOperatorAction::insertBoxService(){
    jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate;
    operate.type = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_BOX;
    operate.action = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_INSERT;
    operate.name = name_editor_->text().toStdString();
    operate.description = description_editor_->text().toStdString();
    operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operate);
  };

  void TransformableMarkerOperatorAction::insertCylinderService(){
    jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate;
    operate.type = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_CYLINDER;
    operate.action = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_INSERT;
    operate.name = name_editor_->text().toStdString();
    operate.description = description_editor_->text().toStdString();
    operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operate);
  };

  void TransformableMarkerOperatorAction::insertMeshService() {
    int current_index = object_editor_->currentIndex();
    if (!(0 <= current_index && current_index < (int)objects_.size())) {
      RCLCPP_ERROR(rclcpp::get_logger("TransformableMarkerOperatorAction"),
                   "Invalid index for object selection: %d. Please select again.", current_index);
      return;
    }
    jsk_recognition_msgs::msg::Object object = objects_[current_index];
    if (object.mesh_resource.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("TransformableMarkerOperatorAction"),
                   "Mesh resource of object '%s' is empty, so skipping.", object.name.c_str());
      return;
    }

    jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate;
    operate.type = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_MESH_RESOURCE;
    operate.action = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_INSERT;
    operate.name = object.name;
    operate.description = description_editor_->text().toStdString();
    operate.frame_id = frame_editor_->text().toStdString();
    operate.mesh_resource = object.mesh_resource;
    operate.mesh_use_embedded_materials = true;
    callRequestMarkerOperateService(operate);
  };

  void TransformableMarkerOperatorAction::insertTorusService(){
    jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate;
    operate.type = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::SHAPE_TORUS;
    operate.action = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_INSERT;
    operate.name = name_editor_->text().toStdString();
    operate.description = description_editor_->text().toStdString();
    operate.frame_id = frame_editor_->text().toStdString();
    callRequestMarkerOperateService(operate);
  };

  void TransformableMarkerOperatorAction::eraseWithIdService(){
    jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate;
    operate.action = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_ERASE;
    operate.name = id_editor_->text().toStdString();
    callRequestMarkerOperateService(operate);
  };

  void TransformableMarkerOperatorAction::eraseAllService(){
    jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate;
    operate.action = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_ERASE_ALL;
    callRequestMarkerOperateService(operate);
  };

  void TransformableMarkerOperatorAction::eraseFocusService(){
    jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate;
    operate.action = jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate::ACTION_ERASE_FOCUS;
    callRequestMarkerOperateService(operate);
  };

  // NOTE: in ROS 1 this was a blocking service call; in ROS 2 the request is
  // sent asynchronously and the result is only logged in the response callback.
  void TransformableMarkerOperatorAction::callRequestMarkerOperateService(jsk_rviz_plugins_msgs::msg::TransformableMarkerOperate operate){
    if (!nh_) return;
    ensureServiceClients();
    std::string service_name = client_server_name_ + "/request_marker_operate";
    if (!request_marker_operate_client_->service_is_ready()) {
      RCLCPP_ERROR(nh_->get_logger(), "Service call FAIL: %s (service not available)", service_name.c_str());
      return;
    }
    auto req = std::make_shared<jsk_rviz_plugins_msgs::srv::RequestMarkerOperate::Request>();
    req->operate = operate;
    request_marker_operate_client_->async_send_request(
      req,
      [this](rclcpp::Client<jsk_rviz_plugins_msgs::srv::RequestMarkerOperate>::SharedFuture) {
        RCLCPP_INFO(nh_->get_logger(), "Call Success");
      });
  }

  // NOTE: converted from a blocking service call to async_send_request.
  void TransformableMarkerOperatorAction::updateDimensionsService() {
    if (!nh_) return;
    ensureServiceClients();
    std::string service_name = client_server_name_ + "/set_dimensions";
    if (!set_dimensions_client_->service_is_ready()) {
      RCLCPP_ERROR(nh_->get_logger(), "Service call fail: %s (service not available)", service_name.c_str());
      return;
    }

    auto req = std::make_shared<jsk_interactive_marker_msgs::srv::SetMarkerDimensions::Request>();
    if (transform_name_editor_->text().toStdString().empty()) {
      req->target_name = transform_name_editor_->placeholderText().toStdString();
    } else {
      req->target_name = transform_name_editor_->text().toStdString();
    }
    if (dimension_x_editor_->text().toStdString().empty()) {
      req->dimensions.x = dimension_x_editor_->placeholderText().toFloat();
    } else {
      req->dimensions.x = dimension_x_editor_->text().toFloat();
    }
    if (dimension_y_editor_->text().toStdString().empty()) {
      req->dimensions.y = dimension_y_editor_->placeholderText().toFloat();
    } else {
      req->dimensions.y = dimension_y_editor_->text().toFloat();
    }
    if (dimension_z_editor_->text().toStdString().empty()) {
      req->dimensions.z = dimension_z_editor_->placeholderText().toFloat();
    } else {
      req->dimensions.z = dimension_z_editor_->text().toFloat();
    }

    set_dimensions_client_->async_send_request(
      req,
      [this, service_name](rclcpp::Client<jsk_interactive_marker_msgs::srv::SetMarkerDimensions>::SharedFuture) {
        RCLCPP_INFO(nh_->get_logger(), "Call success: %s", service_name.c_str());
      });
  }

  void TransformableMarkerOperatorAction::updateFrameId() {
    if (frame_editor_->text().isEmpty()) {
      frame_editor_->setText(getDisplayContext()->getFixedFrame());
    }
  }

  void TransformableMarkerOperatorAction::updateName() {
    int current_index = object_editor_->currentIndex();
    if (0 <= current_index && current_index < (int)objects_.size()) {
      jsk_recognition_msgs::msg::Object object = objects_[current_index];
      name_editor_->setText(QString::fromStdString(object.name));
    } else {
      name_editor_->setText(QString(""));
    }
  }

  void TransformableMarkerOperatorAction::updateServerName() {
    if (!nh_) return;
    std::string server_name = server_name_editor_->text().toStdString();
    if (server_name.empty()) {
      // search for an advertised <server_name>/request_marker_operate service
      const std::string suffix = "/request_marker_operate";
      std::map<std::string, std::vector<std::string> > services = nh_->get_service_names_and_types();
      for (std::map<std::string, std::vector<std::string> >::iterator it = services.begin();
           it != services.end(); ++it) {
        const std::string& name = it->first;
        if (name.size() > suffix.size() &&
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
          server_name_editor_->setText(QString::fromStdString(name.substr(0, name.size() - suffix.size())));
          break;
        }
      }
    }
  }

  // NOTE: converted from blocking service calls to async_send_request; the
  // placeholder texts are updated when the responses arrive.
  void TransformableMarkerOperatorAction::updateFocusMarkerDimensions() {
    if (!nh_) return;
    ensureServiceClients();
    if (!get_focus_client_->service_is_ready() || !get_dimensions_client_->service_is_ready()) {
      auto clk = nh_->get_clock();
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *clk, 10000,
                            "Service call FAIL: %s", client_server_name_.c_str());
      return;
    }
    if (!focus_request_pending_) {
      focus_request_pending_ = true;
      auto req_focus = std::make_shared<jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus::Request>();
      get_focus_client_->async_send_request(
        req_focus,
        [this](rclcpp::Client<jsk_interactive_marker_msgs::srv::GetTransformableMarkerFocus>::SharedFuture future) {
          focus_request_pending_ = false;
          transform_name_editor_->setPlaceholderText(QString::fromStdString(future.get()->target_name));
        });
    }
    if (!dimensions_request_pending_) {
      dimensions_request_pending_ = true;
      auto req_dim = std::make_shared<jsk_interactive_marker_msgs::srv::GetMarkerDimensions::Request>();
      get_dimensions_client_->async_send_request(
        req_dim,
        [this](rclcpp::Client<jsk_interactive_marker_msgs::srv::GetMarkerDimensions>::SharedFuture future) {
          dimensions_request_pending_ = false;
          jsk_interactive_marker_msgs::msg::MarkerDimensions dimensions = future.get()->dimensions;
          dimension_x_editor_->setPlaceholderText(QString::number(dimensions.x, 'f', 4));
          dimension_y_editor_->setPlaceholderText(QString::number(dimensions.y, 'f', 4));
          dimension_z_editor_->setPlaceholderText(QString::number(dimensions.z, 'f', 4));
          dimension_radius_editor_->setPlaceholderText(QString::number(dimensions.radius, 'f', 4));
          dimension_sm_radius_editor_->setPlaceholderText(QString::number(dimensions.small_radius, 'f', 4));
        });
    }
  }

  void TransformableMarkerOperatorAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );
    config.mapSetValue( "ServerName", server_name_editor_->text().toStdString().c_str() );
  }

  void TransformableMarkerOperatorAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
    QString server_name;
    config.mapGetString( "ServerName", &server_name );
    server_name_editor_->setText(server_name);
  }
}  // namespace jsk_interactive_marker

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_interactive_marker::TransformableMarkerOperatorAction, rviz_common::Panel )

// the Q_OBJECT header lives in include/jsk_interactive_marker/rviz_plugins/,
// which CMake AUTOMOC does not scan implicitly, so include the moc explicitly
#include "jsk_interactive_marker/rviz_plugins/moc_transformable_marker_operator.cpp"
