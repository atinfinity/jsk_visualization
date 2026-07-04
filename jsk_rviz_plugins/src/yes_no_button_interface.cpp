#include "yes_no_button_interface.h"
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QThread>

#include <chrono>
#include <thread>

#include <jsk_gui_msgs/srv/yes_no.hpp>


namespace jsk_rviz_plugins
{

  YesNoButtonInterface::YesNoButtonInterface(QWidget* parent)
    : rviz_common::Panel(parent), need_user_input_(false)
  {
    layout_ = new QHBoxLayout;

    yes_button_ = new QPushButton("Yes");
    layout_->addWidget(yes_button_);
    yes_button_->setEnabled(false);

    no_button_ = new QPushButton("No");
    layout_->addWidget(no_button_);
    no_button_->setEnabled(false);

    connect(yes_button_, SIGNAL(clicked()), this, SLOT(respondYes()));
    connect(no_button_, SIGNAL(clicked()), this, SLOT(respondNo()));

    setLayout(layout_);
  }

  void YesNoButtonInterface::onInitialize()
  {
    rclcpp::Node::SharedPtr nh =
      getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    yes_no_button_service_ = nh->create_service<jsk_gui_msgs::srv::YesNo>(
      "/rviz/yes_no_button",
      [this](const jsk_gui_msgs::srv::YesNo::Request::SharedPtr req,
             jsk_gui_msgs::srv::YesNo::Response::SharedPtr res) {
        requested(req, res);
      });
  }

  void YesNoButtonInterface::enableButtons()
  {
    yes_button_->setEnabled(true);
    no_button_->setEnabled(true);
  }

  void YesNoButtonInterface::disableButtons()
  {
    yes_button_->setEnabled(false);
    no_button_->setEnabled(false);
  }

  void YesNoButtonInterface::requested(
      const jsk_gui_msgs::srv::YesNo::Request::SharedPtr /*req*/,
      jsk_gui_msgs::srv::YesNo::Response::SharedPtr res)
  {
    need_user_input_ = true;
    // widgets may only be touched from the GUI thread
    QMetaObject::invokeMethod(this, "enableButtons", Qt::QueuedConnection);
    const bool in_gui_thread =
      QThread::currentThread() == QApplication::instance()->thread();
    while (need_user_input_) {
      if (in_gui_thread) {
        QApplication::processEvents(QEventLoop::AllEvents, 100);
      }
      else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    QMetaObject::invokeMethod(this, "disableButtons", Qt::QueuedConnection);
    res->yes = yes_;
  }

  void YesNoButtonInterface::respondYes()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    yes_ = true;
    need_user_input_ = false;
  }

  void YesNoButtonInterface::respondNo()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    yes_ = false;
    need_user_input_ = false;
  }

  void YesNoButtonInterface::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
  }

  void YesNoButtonInterface::load(const rviz_common::Config& config)
  {
    rviz_common::Panel::load(config);
  }

}  // namespace jsk_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::YesNoButtonInterface, rviz_common::Panel)
