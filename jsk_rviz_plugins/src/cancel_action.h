#ifndef CANCEL_ACTION_H
#define CANCEL_ACTION_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;

namespace jsk_rviz_plugins
{
  class CancelAction: public rviz_common::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      CancelAction( QWidget* parent = 0 );

      virtual void onInitialize();

      virtual void load( const rviz_common::Config& config );
      virtual void save( rviz_common::Config config ) const;

      public Q_SLOTS:

      void setTopic( const QString& topic ) {};

      protected Q_SLOTS:

      void updateTopic() {};

      void sendTopic();
      void addTopic();
      void initComboBox();

      void addTopicList(std::string topic_name);

      void OnClickDeleteButton(int id);

    protected:
      QString output_topic_;

      QPushButton* add_topic_button_;

      QComboBox* add_topic_box_;

      QPushButton* send_topic_button_;

      QSignalMapper *m_sigmap;

      QVBoxLayout* layout;

      struct topicListLayout{
	int id;
	QHBoxLayout* layout_;
	QPushButton* remove_button_;
	QLabel* topic_name_;
	// In ROS 2 actions are cancelled by calling the
	// <action_name>/_action/cancel_goal service instead of publishing
	// actionlib_msgs/GoalID on <action_name>/cancel.
	rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr client_;
      };

      std::vector<topicListLayout> topic_list_layouts_;

      // The ROS node.
      rclcpp::Node::SharedPtr nh_;

    };

}

#endif // TELEOP_PANEL_H
