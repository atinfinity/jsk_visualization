#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <jsk_interactive_marker/interactive_marker_helpers.h>
#include <jsk_interactive_marker/interactive_point_cloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace im_helpers;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

InteractivePointCloud::InteractivePointCloud(std::string marker_name,
			   std::string topic_name, std::string server_name ):
  rclcpp::Node("interactive_point_cloud"),
  marker_name_(marker_name)
{
  (void)server_name;
  tf_buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  // non-owning shared_ptr: the marker server never outlives this node
  // (shared_from_this() is not usable in a constructor)
  rclcpp::Node::SharedPtr node_ptr(this, [](rclcpp::Node*){});
  marker_server_.reset(
    new jsk_interactive_marker::ParentAndChildInteractiveMarkerServer(topic_name, node_ptr));

  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = "size of interactive marker points";
    d.floating_point_range.resize(1);
    d.floating_point_range[0].from_value = 0.0;
    d.floating_point_range[0].to_value = 1.0;
    point_size_ = this->declare_parameter("point_size", 0.004, d);
  }
  input_pointcloud_ = this->declare_parameter("input", std::string("/selected_pointcloud"));
  use_bounding_box_ = this->declare_parameter("use_bounding_box", true);
  input_bounding_box_ = this->declare_parameter("input_bounding_box", std::string("/bounding_box_marker/selected_box_array"));
  initial_handle_pose_ = this->declare_parameter("handle_pose", std::string("/handle_estimator/output_best"));
  display_interactive_manipulator_ = this->declare_parameter("display_interactive_manipulator", true);
  //publish
  pub_click_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/right_click_point", 1);
  pub_left_click_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/left_click_point", 1);
  pub_left_click_relative_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/left_click_point_relative", 1);
  pub_marker_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/marker_pose", 1);
  pub_grasp_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/grasp_pose", 1);
  pub_box_movement_ = this->create_publisher<jsk_recognition_msgs::msg::BoundingBoxMovement>("~/box_movement", 1);
  pub_handle_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/handle_pose", 1);
  pub_handle_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/handle_pose_array", 1);

  //subscribe
  sub_handle_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/set_handle_pose", 1, std::bind( &InteractivePointCloud::setHandlePoseCallback, this, _1));
  sub_marker_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/set_marker_pose", 1, std::bind( &InteractivePointCloud::setMarkerPoseCallback, this, _1));
  sub_point_cloud_.subscribe(this, input_pointcloud_);


  if(use_bounding_box_){
    //point cloud and bounding box
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10));
    sub_bounding_box_.subscribe(this, input_bounding_box_);
    sub_initial_handle_pose_.subscribe(this, initial_handle_pose_);
    sync_->connectInput(sub_point_cloud_, sub_bounding_box_, sub_initial_handle_pose_);
    sync_->registerCallback(std::bind(&InteractivePointCloud::pointCloudAndBoundingBoxCallback, this, _1, _2, _3));

  }else{
    sub_point_cloud_.registerCallback(
      std::bind(&InteractivePointCloud::pointCloudCallback, this, _1));
  }
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&InteractivePointCloud::parametersCallback, this, _1));

  makeMenu();
}

InteractivePointCloud::~InteractivePointCloud(){};

rcl_interfaces::msg::SetParametersResult InteractivePointCloud::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const rclcpp::Parameter& parameter : parameters) {
    if (parameter.get_name() == "point_size") {
      point_size_ = parameter.as_double();
    } else if (parameter.get_name() == "display_interactive_manipulator") {
      if (display_interactive_manipulator_ != parameter.as_bool()) {
        display_interactive_manipulator_ = parameter.as_bool();
        visualization_msgs::msg::InteractiveMarker int_marker;
        marker_server_->get(marker_name_, int_marker );
        if(display_interactive_manipulator_){
          addVisible6DofControl(int_marker);
        }else{
          for(std::vector<visualization_msgs::msg::InteractiveMarkerControl>::iterator it=int_marker.controls.begin(); it!=int_marker.controls.end();){
            if(it->interaction_mode==visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS
               || it->interaction_mode==visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS){
              it = int_marker.controls.erase(it);
            }else{
              it++;
            }
          }
        }
        marker_server_->erase(marker_name_);
        marker_server_->insert(int_marker);
        marker_server_->applyChanges();
      }
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void InteractivePointCloud::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud){
  makeMarker(cloud, point_size_ );
}

void InteractivePointCloud::pointCloudAndBoundingBoxCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud, const jsk_recognition_msgs::msg::BoundingBoxArray::ConstSharedPtr &box, const geometry_msgs::msg::PoseStamped::ConstSharedPtr &handle){
  makeMarker(cloud, box, handle, point_size_ );
}



void InteractivePointCloud::setHandlePoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &ps){
  if( rclcpp::Time(ps->header.stamp) == rclcpp::Time(current_box_.header.stamp) ){
    if(current_box_.boxes.size() > 0){
      handle_pose_ = *ps;

      tf2::Transform tf_ps, tf_box;
      tf_ps.setOrigin(tf2::Vector3(ps->pose.position.x, ps->pose.position.y, ps->pose.position.z));
      tf_ps.setRotation(tf2::Quaternion( ps->pose.orientation.x, ps->pose.orientation.y, ps->pose.orientation.z, ps->pose.orientation.w));

      geometry_msgs::msg::Pose pose = current_box_.boxes[0].pose;
      tf_box.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
      tf_box.setRotation(tf2::Quaternion( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

      handle_tf_ = tf_box.inverseTimes(tf_ps);
      exist_handle_tf_ = true;
      publishHandPose( marker_pose_ );

      /* set box_movement_ */
      geometry_msgs::msg::Pose handle_pose;
      handle_pose.position.x = handle_tf_.getOrigin().x();
      handle_pose.position.y = handle_tf_.getOrigin().y();
      handle_pose.position.z = handle_tf_.getOrigin().z();
      handle_pose.orientation.x = handle_tf_.getRotation().x();
      handle_pose.orientation.y = handle_tf_.getRotation().y();
      handle_pose.orientation.z = handle_tf_.getRotation().z();
      handle_pose.orientation.w = handle_tf_.getRotation().w();

      box_movement_.handle_pose = handle_pose;
    }
  }
}

void InteractivePointCloud::setMarkerPoseCallback( const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_stamped_msg){
  marker_server_->setPose(marker_name_, pose_stamped_msg->pose, pose_stamped_msg->header);
  marker_server_->applyChanges();
}

// create menu
void InteractivePointCloud::makeMenu()
{
  menu_handler_.insert( "Move",  std::bind( &InteractivePointCloud::move, this, _1) );

  menu_handler_.insert( "Hide",  std::bind( &InteractivePointCloud::hide, this, _1));
}

//publish marker pose
void InteractivePointCloud::move( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback ){
  (void)feedback;
  box_movement_.destination = marker_pose_;
  pub_box_movement_->publish(box_movement_);

}

void InteractivePointCloud::leftClickPoint( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if(!feedback->mouse_point_valid)
  {
    RCLCPP_WARN(this->get_logger(), "Clicked point had an invalid position. Not looking there!");
    return;
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Button click in frame "
                      << feedback->header.frame_id << " at point\n"
                      << geometry_msgs::msg::to_yaml(feedback->mouse_point) );
  geometry_msgs::msg::PointStamped click_point;
  click_point.point = feedback->mouse_point;
  click_point.header = feedback->header;
  click_point.header.stamp = this->now();
  pub_left_click_->publish(click_point);
  tf2::Transform transform;
  tf2::fromMsg(feedback->pose, transform);
  tf2::Vector3 vector_absolute(feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z);
  tf2::Vector3 vector_relative=transform.inverse() * vector_absolute;
  click_point.point.x=vector_relative.getX();
  click_point.point.y=vector_relative.getY();
  click_point.point.z=vector_relative.getZ();
  pub_left_click_relative_->publish(click_point);
}

void InteractivePointCloud::hide( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  (void)feedback;
  marker_server_->erase(marker_name_);
  marker_server_->applyChanges();
}


void InteractivePointCloud::markerFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
  marker_pose_.pose = feedback->pose;
  marker_pose_.header = feedback->header;
  if(exist_handle_tf_){
    publishHandPose( marker_pose_);
  }
}

void InteractivePointCloud::publishGraspPose(){
  pub_grasp_pose_->publish(handle_pose_);
}

void InteractivePointCloud::publishHandPose( geometry_msgs::msg::PoseStamped box_pose){
  tf2::Transform tf_marker;
  tf_marker.setOrigin(tf2::Vector3(box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z));
  tf_marker.setRotation(tf2::Quaternion( box_pose.pose.orientation.x, box_pose.pose.orientation.y, box_pose.pose.orientation.z, box_pose.pose.orientation.w));

  tf_marker = tf_marker * handle_tf_;
  geometry_msgs::msg::PoseStamped handle_pose;
  handle_pose.header.frame_id = box_pose.header.frame_id;
  handle_pose.pose.position.x = tf_marker.getOrigin().x();
  handle_pose.pose.position.y = tf_marker.getOrigin().y();
  handle_pose.pose.position.z = tf_marker.getOrigin().z();
  handle_pose.pose.orientation.x = tf_marker.getRotation().x();
  handle_pose.pose.orientation.y = tf_marker.getRotation().y();
  handle_pose.pose.orientation.z = tf_marker.getRotation().z();
  handle_pose.pose.orientation.w = tf_marker.getRotation().w();

  pub_handle_pose_->publish(handle_pose);

  geometry_msgs::msg::PoseArray handle_pose_array;
  handle_pose_array.header = handle_pose.header;
  handle_pose_array.poses.push_back(handle_pose.pose);
  pub_handle_pose_array_->publish(handle_pose_array);
}

void InteractivePointCloud::makeMarker(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,float size){
  makeMarker(cloud, jsk_recognition_msgs::msg::BoundingBoxArray::ConstSharedPtr(), geometry_msgs::msg::PoseStamped::ConstSharedPtr(), size);
}
void InteractivePointCloud::makeMarker(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, const jsk_recognition_msgs::msg::BoundingBoxArray::ConstSharedPtr box, const geometry_msgs::msg::PoseStamped::ConstSharedPtr handle, float size)
{
  exist_handle_tf_ = false;
  if(cloud){
    current_croud_ = *cloud;
  }
  if(box){
    current_box_ = *box;
  }

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.name = marker_name_;

  pcl::PointCloud<PointT> pcl_cloud, transform_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);

  if(box && box->boxes.size() > 0){
    jsk_recognition_msgs::msg::BoundingBox first_box = box->boxes[0];
    int_marker.pose = first_box.pose;
    int_marker.header.frame_id = first_box.header.frame_id;

    marker_pose_.header = box->header;
    marker_pose_.pose = first_box.pose;

    tf2::Transform transform;
    tf2::fromMsg(first_box.pose, transform);

    pcl_ros::transformPointCloud(pcl_cloud, transform_cloud, transform.inverse());
    pcl_cloud = transform_cloud;

    box_movement_.box = first_box;
    box_movement_.header.frame_id = first_box.header.frame_id;

    if(handle->header.frame_id != ""){
      setHandlePoseCallback(handle);
    }
  }
  else{
    int_marker.pose.position.x=int_marker.pose.position.y=int_marker.pose.position.z=int_marker.pose.orientation.x=int_marker.pose.orientation.y=int_marker.pose.orientation.z=0;
    int_marker.pose.orientation.w=1;
  }


  int num_points = pcl_cloud.points.size();

  visualization_msgs::msg::Marker marker;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.frame_locked = false;
  if(num_points > 0)
  {

      int_marker.header = cloud->header;

      visualization_msgs::msg::InteractiveMarkerControl control;
      control.always_visible = true;
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
      //control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
      control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;

      int_marker.header.stamp = this->now();

      marker.scale.x = size;
      marker.scale.y = size;
      marker.scale.z = size;
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

      marker.points.resize( num_points );
      marker.colors.resize( num_points );
      //point cloud
      for ( int i=0; i<num_points; i++)
	{
	  marker.points[i].x = pcl_cloud.points[i].x;
	  marker.points[i].y = pcl_cloud.points[i].y;
	  marker.points[i].z = pcl_cloud.points[i].z;
	  marker.colors[i].r = pcl_cloud.points[i].r/255.;
	  marker.colors[i].g = pcl_cloud.points[i].g/255.;
	  marker.colors[i].b = pcl_cloud.points[i].b/255.;
	  marker.colors[i].a = 1.0;
	}
      control.markers.push_back( marker );

      //bounding box
      if(box && box->boxes.size() > 0){
	visualization_msgs::msg::Marker bounding_box_marker;
	bounding_box_marker.color.r = 0.0;
	bounding_box_marker.color.g = 0.0;
	bounding_box_marker.color.b = 1.0;
	bounding_box_marker.color.a = 1.0;
	bounding_box_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
	bounding_box_marker.scale.x = 0.01; //line width
	bounding_box_marker.points.resize(24);

	double x = box->boxes[0].dimensions.x / 2;
	double y = box->boxes[0].dimensions.y / 2;
	double z = box->boxes[0].dimensions.z / 2;
	for(int i=0; i<4; i++)
	  {
	    if(i%2 == 0){
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = x;

	    }else{
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = - x;
	    }
	    if(i%4 < 2){
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = y;

	    }else{
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = - y;

	    }
	    bounding_box_marker.points[2*i].z = z;
	    bounding_box_marker.points[2*i+1].z = - z;
	  }

	for(int i=4; i<8; i++)
	  {
	    if(i%2 == 0){
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = x;

	    }else{
	      bounding_box_marker.points[2*i].x = bounding_box_marker.points[2*i+1].x = - x;
	    }
	    if(i%4 < 2){
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = z;

	    }else{
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = - z;

	    }
	    bounding_box_marker.points[2*i].y = y;
	    bounding_box_marker.points[2*i+1].y = - y;
	  }

	for(int i=8; i<12; i++)
	  {
	    if(i%2 == 0){
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = z;

	    }else{
	      bounding_box_marker.points[2*i].z = bounding_box_marker.points[2*i+1].z = - z;
	    }
	    if(i%4 < 2){
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = y;

	    }else{
	      bounding_box_marker.points[2*i].y = bounding_box_marker.points[2*i+1].y = - y;

	    }
	    bounding_box_marker.points[2*i].x = x;
	    bounding_box_marker.points[2*i+1].x = - x;
	  }

	control.markers.push_back( bounding_box_marker );
      }
      int_marker.controls.push_back( control );
      if(display_interactive_manipulator_){
	addVisible6DofControl(int_marker);
      }
      marker_server_->insert( int_marker, std::bind( &InteractivePointCloud::leftClickPoint, this, _1 ),
			      visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK);
      marker_server_->setCallback( int_marker.name,
				   std::bind( &InteractivePointCloud::markerFeedback, this, _1) );

      menu_handler_.apply( *marker_server_, marker_name_ );
      marker_server_->applyChanges();
      RCLCPP_INFO(this->get_logger(), "made interactive point cloud");

      publishGraspPose();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto interactive_point_cloud = std::make_shared<InteractivePointCloud>(
    "interactive_manipulation_snapshot",
    "interactive_point_cloud", "interactive_manipulation_snapshot_server");
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::spin(interactive_point_cloud);
  rclcpp::shutdown();
  return 0;
}
