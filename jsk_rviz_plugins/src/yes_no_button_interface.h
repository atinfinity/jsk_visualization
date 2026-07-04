#ifndef YES_NO_BUTTON_INTERFACE_H
#define YES_NO_BUTTON_INTERFACE_H

#ifndef Q_MOC_RUN
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QtWidgets>
#include <atomic>
#include <mutex>
#endif

#include <jsk_gui_msgs/srv/yes_no.hpp>


namespace jsk_rviz_plugins
{

  class YesNoButtonInterface: public rviz_common::Panel
  {
  Q_OBJECT
  public:
    YesNoButtonInterface(QWidget* parent = 0);

    void onInitialize() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

  protected Q_SLOTS:
    void respondYes();
    void respondNo();
    void enableButtons();
    void disableButtons();
  protected:
    virtual void requested(
      const jsk_gui_msgs::srv::YesNo::Request::SharedPtr req,
      jsk_gui_msgs::srv::YesNo::Response::SharedPtr res);
    QHBoxLayout* layout_;
    QPushButton* yes_button_;
    QPushButton* no_button_;
    bool yes_;
    std::atomic<bool> need_user_input_;
    std::mutex mutex_;
    rclcpp::Service<jsk_gui_msgs::srv::YesNo>::SharedPtr yes_no_button_service_;
  };

}  // namespace jsk_rviz_plugins


#endif  // YES_NO_BUTTON_INTERFACE_H
