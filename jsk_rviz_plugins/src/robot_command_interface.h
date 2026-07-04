#ifndef ROBOT_COMMAND_INTERFACE_H
#define ROBOT_COMMAND_INTERFACE_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#include <jsk_rviz_plugins_msgs/srv/eus_command.hpp>
#include <std_srvs/srv/empty.hpp>
#include <map>
#include <string>
#endif

namespace jsk_rviz_plugins
{
  class RobotCommandInterfaceAction: public rviz_common::Panel
  {
    Q_OBJECT
    public:
    RobotCommandInterfaceAction( QWidget* parent = 0 );

    virtual void onInitialize();

  protected Q_SLOTS:
    bool callRequestEusCommand(const std::string& command);
    void buttonCallback(int i);
  protected:
    void popupDialog(const std::string& text);
    // The ROS node.
    rclcpp::Node::SharedPtr nh_;
    QSignalMapper* signal_mapper_;
    std::map<int, std::string> euscommand_mapping_;
    std::map<int, std::string> emptyservice_mapping_;
    rclcpp::Client<jsk_rviz_plugins_msgs::srv::EusCommand>::SharedPtr eus_command_client_;
    std::map<int, rclcpp::Client<std_srvs::srv::Empty>::SharedPtr> empty_service_clients_;
    //std::vector<QToolButton*> buttons_;
  };

}

#endif // TELEOP_PANEL_H
