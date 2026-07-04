#include <stdio.h>

#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "robot_command_interface.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <exception>

namespace jsk_rviz_plugins
{

  // Exception class
  class RobotCommandParseException: public std::runtime_error
  {
  public:
    RobotCommandParseException(const std::string& text): std::runtime_error(text) {}
  };

  RobotCommandInterfaceAction::RobotCommandInterfaceAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    signal_mapper_ = new QSignalMapper(this);
    connect(signal_mapper_, SIGNAL(mapped(int)), this, SLOT(buttonCallback(int)));
  }

  void RobotCommandInterfaceAction::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    auto getStringParam = [this](const std::string& name)
      {
        if (!nh_->has_parameter(name)) {
          nh_->declare_parameter(name, std::string(""));
        }
        return nh_->get_parameter(name).as_string();
      };

    QHBoxLayout* layout = new QHBoxLayout();
    // Parse button configuration from parameters.
    // NOTE: ROS 2 does not support arrays of structs as parameters, so the
    // ROS 1 parameter "~robot_command_buttons" is mapped to indexed
    // parameters: "robot_command_buttons.<i>.{name,icon,type,command,srv}".
    bool found_any_button = false;
    try {
      for (int i = 0; ; ++i) {
        std::string prefix = "robot_command_buttons." + std::to_string(i) + ".";
        std::string name = getStringParam(prefix + "name");
        std::string type = getStringParam(prefix + "type");
        if (name.empty() && type.empty()) {
          break;
        }
        found_any_button = true;
        if (name.empty()) {
          throw RobotCommandParseException("element of robot_command_buttons should have name field");
        }
        QToolButton* button = new QToolButton();
        //button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button->setText(QString(name.c_str()));
        std::string icon = getStringParam(prefix + "icon");
        if (!icon.empty()) {
          if (icon.find("package://") == 0) {
            icon.erase(0, strlen("package://"));
            size_t package_end = icon.find("/");
            std::string package = icon.substr(0, package_end);
            icon.erase(0, package_end);
            std::string package_path;
            package_path = ament_index_cpp::get_package_share_directory(package);
            icon = package_path + icon;
          }
          button->setIcon(QIcon(QPixmap(QString(icon.c_str()))));
          button->setIconSize(QSize(80, 80));
          button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        }
        if (type == "euscommand") {
          std::string command = getStringParam(prefix + "command");
          if (!command.empty()) {
            euscommand_mapping_[i] = command;
            button->setToolTip(euscommand_mapping_[i].c_str());
          }
          else {
            throw RobotCommandParseException("type: euscommand requires command field");
          }
        }
        else if (type == "emptysrv") {
          std::string srv = getStringParam(prefix + "srv");
          if (!srv.empty()) {
            emptyservice_mapping_[i] = srv;
            button->setToolTip(emptyservice_mapping_[i].c_str());
          }
          else {
            throw RobotCommandParseException("type: emptysrv requires srv field");
          }
        }
        else {
          throw RobotCommandParseException("type field is required");
        }
        // connect
        connect(button, SIGNAL(clicked()), signal_mapper_, SLOT(map()));
        signal_mapper_->setMapping(button, i);
        layout->addWidget(button);
      }
    }
    catch (RobotCommandParseException& e) {
      popupDialog(std::string("Malformed robot_command_buttons parameter.\n")
                  + e.what() + std::string("\n")
                  + std::string("See package://jsk_rviz_plugins/config/default_robot_command.yaml"));
    }
    if (!found_any_button) {
      popupDialog("You need to specify robot_command_buttons parameter.\n"
                  "See package://jsk_rviz_plugins/launch/robot_command_interface_sample.launch");
    }
    layout->addStretch();
    this->setLayout(layout);
  }

  bool RobotCommandInterfaceAction::callRequestEusCommand(const std::string& command){
    if (!eus_command_client_) {
      eus_command_client_ = nh_->create_client<jsk_rviz_plugins_msgs::srv::EusCommand>("/eus_command");
    }
    if (!eus_command_client_->service_is_ready()) {
      return false;
    }
    auto req = std::make_shared<jsk_rviz_plugins_msgs::srv::EusCommand::Request>();
    req->command = command;
    // fire-and-forget: do not block the GUI thread waiting for the response
    eus_command_client_->async_send_request(
      req,
      [this](rclcpp::Client<jsk_rviz_plugins_msgs::srv::EusCommand>::SharedFuture /*future*/)
      {
        RCLCPP_INFO(nh_->get_logger(), "Call Success");
      });
    return true;
  }

  void RobotCommandInterfaceAction::buttonCallback(int i)
  {
    RCLCPP_INFO(nh_->get_logger(), "buttonCallback(%d)", i);
    if (euscommand_mapping_.find(i) != euscommand_mapping_.end()) {
      if(!callRequestEusCommand(euscommand_mapping_[i])) {
        popupDialog(std::string("Failed to call ") + euscommand_mapping_[i]);
      }
    }
    else if (emptyservice_mapping_.find(i) != emptyservice_mapping_.end()) {
      if (empty_service_clients_.find(i) == empty_service_clients_.end()) {
        empty_service_clients_[i] = nh_->create_client<std_srvs::srv::Empty>(emptyservice_mapping_[i]);
      }
      rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client = empty_service_clients_[i];
      if (!client->service_is_ready()) {
        popupDialog(std::string("Failed to call ") + emptyservice_mapping_[i]);
        return;
      }
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      client->async_send_request(
        req,
        [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*future*/)
        {
          RCLCPP_INFO(nh_->get_logger(), "Call Success");
        });
    }
    else {
      popupDialog(std::string("Failed to find corresponding command for ") + std::to_string(i));
    }
  }

  void RobotCommandInterfaceAction::popupDialog(const std::string& text)
  {
    QMessageBox msg_box;
    msg_box.setText("Unexpected error");
    msg_box.setText(QString(text.c_str()));
    msg_box.exec();
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::RobotCommandInterfaceAction, rviz_common::Panel )
