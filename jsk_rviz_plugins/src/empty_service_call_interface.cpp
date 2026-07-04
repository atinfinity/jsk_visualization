#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "empty_service_call_interface.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

namespace jsk_rviz_plugins
{
  EmptyServiceCallInterfaceAction::EmptyServiceCallInterfaceAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    layout = new QVBoxLayout();
    signal_mapper = new QSignalMapper(this);
    connect(signal_mapper, SIGNAL(mapped(int)),
            this, SLOT(callRequestEmptyCommand(int)));
    setLayout( layout );
  }

  void EmptyServiceCallInterfaceAction::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    parseROSParameters();

    QHBoxLayout* h_layout = new QHBoxLayout;
    h_layout->setAlignment(Qt::AlignLeft);

    for(size_t i = 0; i < service_call_button_infos_.size();i++){
      ServiceCallButtonInfo target_button = service_call_button_infos_[i];
      QToolButton* tbutton = new QToolButton(this);
      tbutton->setText(target_button.text.c_str());
      tbutton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
      tbutton->setIconSize(QSize(100, 100));
      tbutton->setIcon(QIcon(QPixmap(QString(target_button.icon_file_path.c_str()))));
      connect(tbutton, SIGNAL(clicked()), signal_mapper, SLOT(map()));
      signal_mapper->setMapping(tbutton, i);
      h_layout->addWidget(tbutton);
    };
    layout->addLayout(h_layout);
  }

  void EmptyServiceCallInterfaceAction::parseROSParameters(){
    auto getStringParam = [this](const std::string& name, const std::string& default_value)
      {
        if (!nh_->has_parameter(name)) {
          nh_->declare_parameter(name, default_value);
        }
        return nh_->get_parameter(name).as_string();
      };

    //icon file package file_name
    std::string icon_package_name =
      getStringParam("icon_include_package", std::string("jsk_rviz_plugins"));
    RCLCPP_INFO(nh_->get_logger(), "Find Icons In %s package.", icon_package_name.c_str());

    std::string icon_path_prefix;
    if(!icon_package_name.empty())
      icon_path_prefix = ament_index_cpp::get_package_share_directory(icon_package_name) + std::string("/icons/");

    // NOTE: ROS 2 does not support arrays of structs as parameters, so the
    // ROS 1 parameter "rviz_service_call/buttons" is mapped to indexed
    // parameters: "rviz_service_call.buttons.<i>.{icon,service_name,text}".
    for (int i = 0; ; ++i)
      {
        std::string prefix = "rviz_service_call.buttons." + std::to_string(i) + ".";
        std::string service_name = getStringParam(prefix + "service_name", std::string(""));
        if (service_name.empty()) {
          break;
        }
        ServiceCallButtonInfo new_button;
        new_button.icon_file_path = icon_path_prefix + getStringParam(prefix + "icon", std::string(""));
        new_button.service_name = service_name;
        new_button.text = getStringParam(prefix + "text", std::string(""));
        service_call_button_infos_.push_back(new_button);
      }
  };

  void EmptyServiceCallInterfaceAction::callRequestEmptyCommand(const int button_id){
    if (service_clients_.find(button_id) == service_clients_.end()) {
      service_clients_[button_id] = nh_->create_client<std_srvs::srv::Empty>(
        service_call_button_infos_[button_id].service_name);
    }
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client = service_clients_[button_id];
    if (!client->service_is_ready()) {
      RCLCPP_ERROR(nh_->get_logger(), "Service %s is not available",
                   service_call_button_infos_[button_id].service_name.c_str());
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    // fire-and-forget: do not block the GUI thread waiting for the response
    client->async_send_request(
      req,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*future*/)
      {
        RCLCPP_INFO(nh_->get_logger(), "Call Success");
      });
  }

  void EmptyServiceCallInterfaceAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );
  }

  void EmptyServiceCallInterfaceAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::EmptyServiceCallInterfaceAction, rviz_common::Panel )
