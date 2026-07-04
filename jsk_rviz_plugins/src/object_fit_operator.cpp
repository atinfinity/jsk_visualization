#include <QLineEdit>
#include <QToolButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QCheckBox>
#include <QLabel>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/display_context.hpp>

#include "object_fit_operator.h"

namespace jsk_rviz_plugins
{
  ObjectFitOperatorAction::ObjectFitOperatorAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    layout = new QVBoxLayout;

    horizontal_layout1_ = new QHBoxLayout();
    horizontal_layout2_ = new QHBoxLayout();

    //Button to send cancel topic
    // NOTE: in ROS 1 the icon paths could be overridden with the global
    // parameters /object_fit_icon, /object_near_icon and /object_other_icon.
    // In ROS 2 the panel uses the icons shipped with this package.
    std::string package_path = ament_index_cpp::get_package_share_directory("jsk_rviz_plugins");
    std::string fit_button_name = package_path + std::string("/icons/fit.jpg");
    std::string near_button_name = package_path + std::string("/icons/near.jpg");
    std::string other_button_name = package_path + std::string("/icons/other.jpg");

    QSize iconSize(150, 150);
    fit_button_ = new QToolButton();
    fit_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    fit_button_->setIcon(QIcon(QPixmap(QString(fit_button_name.c_str()))));
    fit_button_->setText("Onto");
    fit_button_->setIconSize(iconSize);
    horizontal_layout1_->addWidget( fit_button_ );

    check_box_ = new QCheckBox(QString("Reverse"));
    horizontal_layout1_->addWidget(check_box_);

    near_button_ = new QToolButton();
    near_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    near_button_->setIcon(QIcon(QPixmap(QString(near_button_name.c_str()))));
    near_button_->setIconSize(iconSize);
    near_button_->setText("Parallel");
    horizontal_layout2_->addWidget( near_button_ );

    other_button_ = new QToolButton();
    other_button_->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    other_button_->setIcon(QIcon(QPixmap(QString(other_button_name.c_str()))));
    other_button_->setText("Perpendicular");
    other_button_->setIconSize(iconSize);
    horizontal_layout2_->addWidget( other_button_ );

    layout->addLayout(horizontal_layout1_);
    layout->addLayout(horizontal_layout2_);
    setLayout( layout );

    connect( fit_button_, SIGNAL( clicked() ), this, SLOT( commandFit()));
    connect( check_box_, SIGNAL(clicked(bool)), this, SLOT(checkBoxChanged(bool)));
    connect( near_button_, SIGNAL( clicked() ), this, SLOT( commandNear()));
    connect( other_button_, SIGNAL( clicked() ), this, SLOT( commandOther()));
  }

  void ObjectFitOperatorAction::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    pub_ = nh_->create_publisher<jsk_rviz_plugins_msgs::msg::ObjectFitCommand>( "/object_fit_command", 1 );
  }

  void ObjectFitOperatorAction::checkBoxChanged(bool state){
    reverse_ = state;
  }

  void ObjectFitOperatorAction::commandFit(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins_msgs::msg::ObjectFitCommand::REVERSE_FIT);
    else
      publishObjectFitOder(jsk_rviz_plugins_msgs::msg::ObjectFitCommand::FIT);
  }

  void ObjectFitOperatorAction::commandNear(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins_msgs::msg::ObjectFitCommand::REVERSE_NEAR);
    else
      publishObjectFitOder(jsk_rviz_plugins_msgs::msg::ObjectFitCommand::NEAR);
  }

  void ObjectFitOperatorAction::commandOther(){
    if(reverse_)
      publishObjectFitOder(jsk_rviz_plugins_msgs::msg::ObjectFitCommand::REVERSE_OTHER);
    else
      publishObjectFitOder(jsk_rviz_plugins_msgs::msg::ObjectFitCommand::OTHER);
  }

  void ObjectFitOperatorAction::publishObjectFitOder(int type){
    if(!pub_)
      return;
    jsk_rviz_plugins_msgs::msg::ObjectFitCommand msg;
    msg.command = type;
    pub_->publish(msg);
  }

  void ObjectFitOperatorAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );
  }

  void ObjectFitOperatorAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::ObjectFitOperatorAction, rviz_common::Panel )
