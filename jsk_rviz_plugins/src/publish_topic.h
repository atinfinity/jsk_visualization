#ifndef PUBLISH_TOPIC_H
#define PUBLISH_TOPIC_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <rviz_common/panel.hpp>
#endif

class QLineEdit;
class QPushButton;

namespace jsk_rviz_plugins
{
  class PublishTopic: public rviz_common::Panel
    {
Q_OBJECT
  public:
      PublishTopic( QWidget* parent = 0 );

      virtual void onInitialize();

      virtual void load( const rviz_common::Config& config );
      virtual void save( rviz_common::Config config ) const;

      public Q_SLOTS:

      void setTopic( const QString& topic );

      protected Q_SLOTS:

      void sendVel() {}

      void updateTopic();

      void sendTopic();

    protected:
      QLineEdit* output_topic_editor_;

      QString output_topic_;

      QPushButton* send_topic_button_;

      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr velocity_publisher_;

      rclcpp::Node::SharedPtr nh_;
    };

}

#endif
