#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <rviz_common/display_context.hpp>

#include "cancel_action.h"

namespace jsk_rviz_plugins
{

  CancelAction::CancelAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    layout = new QVBoxLayout;

    //Text Box and Add Button to add new topic
    QHBoxLayout* topic_layout = new QHBoxLayout;

    add_topic_box_ = new QComboBox;
    topic_layout->addWidget( add_topic_box_ );

    QPushButton* add_topic_button_ = new QPushButton("Add Action");
    topic_layout->addWidget( add_topic_button_ );

    layout->addLayout( topic_layout );
    //End of Text Box and Add Button

    m_sigmap = new QSignalMapper(this);

    connect(m_sigmap, SIGNAL(mapped(int)),this, SLOT(OnClickDeleteButton(int)));

    //Button to send cancel topic
    QPushButton* send_topic_button_ = new QPushButton("Cancel Action");
    layout->addWidget( send_topic_button_ );

    setLayout( layout );

    connect( send_topic_button_, SIGNAL( clicked() ), this, SLOT( sendTopic ()));
    connect( add_topic_button_, SIGNAL( clicked() ), this, SLOT( addTopic() ));
  }

  void CancelAction::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    initComboBox();
  }

  void CancelAction::initComboBox(){
    add_topic_box_->addItem("");
    // In ROS 2, action servers are discovered by looking for their
    // <action_name>/_action/cancel_goal service of type action_msgs/srv/CancelGoal.
    const std::string suffix = "/_action/cancel_goal";
    std::map<std::string, std::vector<std::string> > services = nh_->get_service_names_and_types();
    std::map<std::string, std::vector<std::string> >::iterator it = services.begin();
    while( it != services.end()){
      if(std::find(it->second.begin(), it->second.end(),
                   "action_msgs/srv/CancelGoal") != it->second.end()){
	std::string action_name = it->first;
	if(action_name.length() > suffix.length() &&
	   action_name.compare(action_name.length() - suffix.length(),
			       suffix.length(), suffix) == 0){
	  action_name.erase(action_name.length() - suffix.length());
	  add_topic_box_->addItem(action_name.c_str());
	}
      }
      it ++;
    }
  }


  void CancelAction::OnClickDeleteButton(int id){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      if(it->id == id){
	it->topic_name_->hide();
	delete it->topic_name_;

	it->remove_button_->hide();
	delete it->remove_button_;

	delete it->layout_;
	it->client_.reset();
	it = topic_list_layouts_.erase( it );
	Q_EMIT configChanged();
      }else{
	++it;
      }
    }
  }

  void CancelAction::addTopic()
  {
    output_topic_ = add_topic_box_->currentText();
    if( output_topic_ != "" ){
      add_topic_box_->setCurrentIndex( 0 );
      addTopicList(output_topic_.toStdString());
    }
    Q_EMIT configChanged();
  }

  void CancelAction::addTopicList(std::string topic_name){
    topicListLayout tll;

    if(!topic_list_layouts_.empty()){
      topicListLayout lastTll = topic_list_layouts_.back();
      tll.id = lastTll.id + 1;
    }else{
      tll.id = 0;
    }

    tll.layout_ = new QHBoxLayout;

    tll.topic_name_ = new QLabel( topic_name.c_str() );
    tll.layout_->addWidget( tll.topic_name_ );

    tll.remove_button_ = new QPushButton("Delete");
    tll.layout_->addWidget( tll.remove_button_ );

    layout->addLayout(tll.layout_);

    if( nh_ ){
      tll.client_ = nh_->create_client<action_msgs::srv::CancelGoal>( topic_name + "/_action/cancel_goal" );
    }

    topic_list_layouts_.push_back(tll);

    connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
    m_sigmap->setMapping(tll.remove_button_, tll.id);

  }

  void CancelAction::sendTopic(){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      if( it->client_ ){
	if( it->client_->service_is_ready() ){
	  // an empty goal_info cancels all goals of the action server
	  auto req = std::make_shared<action_msgs::srv::CancelGoal::Request>();
	  // fire-and-forget: do not block the GUI thread waiting for the response
	  it->client_->async_send_request(
	    req,
	    [this](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture /*future*/)
	    {
	      RCLCPP_INFO(nh_->get_logger(), "Cancel Success");
	    });
	}else{
	  RCLCPP_ERROR(nh_->get_logger(), "Service %s is not available",
		       (it->topic_name_->text().toStdString() + "/_action/cancel_goal").c_str());
	}
      }
      it++;
    }
  }

  void CancelAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );

    rviz_common::Config topic_list_config = config.mapMakeChild( "topics" );

    std::vector<topicListLayout>::const_iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      topic_list_config.listAppendNew().setValue( it->topic_name_->text() );
      it ++;
    }
    config.mapSetValue( "Topic", output_topic_ );
  }

  // Load all configuration data for this panel from the given Config object.
  void CancelAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
    rviz_common::Config topic_list_config = config.mapGetChild( "topics" );
    int num_topics = topic_list_config.listLength();

    for( int i = 0; i < num_topics; i++ ) {
      addTopicList(topic_list_config.listChildAt( i ).getValue().toString().toStdString());
    }
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::CancelAction, rviz_common::Panel )
