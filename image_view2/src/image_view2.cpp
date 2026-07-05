// -*- mode: c++ -*-
// ROS 2 port of image_view2 (JSK Lab, BSD license). Phase 1.
#include "image_view2.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace image_view2
{

namespace
{
constexpr int DEFAULT_CIRCLE_SCALE = 20;
constexpr int DEFAULT_LINE_WIDTH = 3;

double durationToSec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}
}  // namespace

ImageView2::ImageView2(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_view2", options),
  marker_topic_("image_marker"),
  window_initialized_(false),
  font_(cv::FONT_HERSHEY_DUPLEX),
  warned_3d_(false),
  mode_(MODE_RECTANGLE),
  selecting_fg_(true),
  left_button_clicked_(false),
  continuous_ready_(false),
  count_(0),
  line_select_start_point_(true),
  line_selected_(false),
  poly_selecting_done_(false)
{
  window_selection_ = cv::Rect(0, 0, 0, 0);
  rect_fg_ = cv::Rect(0, 0, 0, 0);
  rect_bg_ = cv::Rect(0, 0, 0, 0);
  // Parameters (private in ROS 1; plain node parameters here).
  transport_ = this->declare_parameter("image_transport", std::string("raw"));
  use_window = this->declare_parameter("use_window", true);
  autosize_ = this->declare_parameter("autosize", false);
  skip_draw_rate_ = this->declare_parameter("skip_draw_rate", 0);
  blurry_mode_ = this->declare_parameter("blurry", false);
  show_info_ = this->declare_parameter("show_info", false);
  tf_timeout_ = this->declare_parameter("tf_timeout", 1.0);
  resize_x_ = this->declare_parameter("resize_scale_x", 1.0);
  resize_y_ = this->declare_parameter("resize_scale_y", 1.0);
  region_continuous_publish_ =
    this->declare_parameter("region_continuous_publish", false);
  filename_format_ = this->declare_parameter(
    "filename_format", std::string("frame%04i.jpg"));
  const std::string interaction_mode =
    this->declare_parameter("interaction_mode", std::string("rectangle"));
  try {
    mode_ = stringToMode(interaction_mode);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      this->get_logger(), "unknown interaction_mode '%s', using rectangle",
      interaction_mode.c_str());
  }

  // Grid parameters (formerly the dynamic_reconfigure ImageView2Config).
  draw_grid_ = this->declare_parameter("grid", false);
  fisheye_mode_ = this->declare_parameter("fisheye_mode", false);
  div_u_ = this->declare_parameter("div_u", 10);
  div_v_ = this->declare_parameter("div_v", 10);
  grid_red_ = this->declare_parameter("grid_red", 255);
  grid_green_ = this->declare_parameter("grid_green", 0);
  grid_blue_ = this->declare_parameter("grid_blue", 0);
  grid_thickness_ = this->declare_parameter("grid_thickness", 2);
  space_ = this->declare_parameter("grid_space", 10);
  prev_space_ = prev_red_ = prev_green_ = prev_blue_ = prev_thickness_ = -1;
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return this->onSetParameters(params);
    });

  // Resolve the (remapped) input image topic so the "<camera>/marked" and
  // "<camera>/event" topics match the ROS 1 layout that rqt_image_view2
  // expects.
  const std::string camera =
    this->get_node_topics_interface()->resolve_topic_name("image");
  window_name_ = this->declare_parameter(
    "window_name", std::string("image_view2 [") + camera + std::string("]"));

  if (camera == "/image") {
    RCLCPP_WARN(
      this->get_logger(),
      "image has not been remapped! Typical usage:\n"
      "\t$ ros2 run image_view2 image_view2 --ros-args -r image:=<image topic>");
  }

  rclcpp::QoS marker_qos(10);
  rclcpp::QoS event_qos(100);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  image_pub_ = image_transport::create_publisher(this, "image_marked");
  local_image_pub_ = image_transport::create_publisher(this, camera + "/marked");

  image_sub_ = image_transport::create_subscription(
    this, camera,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {this->imageCb(msg);},
    transport_);
  marker_sub_ = this->create_subscription<Marker>(
    marker_topic_, marker_qos,
    [this](const Marker::ConstSharedPtr & msg) {this->markerCb(msg);});
  event_sub_ = this->create_subscription<image_view2::msg::MouseEvent>(
    camera + "/event", event_qos,
    [this](const image_view2::msg::MouseEvent::ConstSharedPtr & msg) {this->eventCb(msg);});
  info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::QoS(1),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {this->infoCb(msg);});

  // interaction result publishers (camera-namespaced, matching ROS 1)
  const rclcpp::QoS pub_qos(100);
  point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    camera + "/screenpoint", pub_qos);
  point_array_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    camera + "/screenpoint_array", pub_qos);
  rectangle_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    camera + "/screenrectangle", pub_qos);
  rectangle_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    camera + "/screenrectangle_image", pub_qos);
  move_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    camera + "/movepoint", pub_qos);
  foreground_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    camera + "/foreground", pub_qos);
  background_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    camera + "/background", pub_qos);
  foreground_rect_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    camera + "/foreground_rect", pub_qos);
  background_rect_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    camera + "/background_rect", pub_qos);
  line_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    camera + "/line", pub_qos);
  poly_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
    camera + "/poly", pub_qos);

  // mode services
  change_mode_srv_ = this->create_service<image_view2::srv::ChangeMode>(
    "change_mode",
    [this](
      const std::shared_ptr<image_view2::srv::ChangeMode::Request> req,
      std::shared_ptr<image_view2::srv::ChangeMode::Response> res) {
      this->changeModeCb(req, res);
    });
  const std::vector<std::pair<std::string, KEY_MODE>> mode_map = {
    {"rectangle_mode", MODE_RECTANGLE},
    {"series_mode", MODE_SERIES},
    {"grabcut_mode", MODE_SELECT_FORE_AND_BACK},
    {"grabcut_rect_mode", MODE_SELECT_FORE_AND_BACK_RECT},
    {"line_mode", MODE_LINE},
    {"poly_mode", MODE_POLY},
    {"none_mode", MODE_NONE},
  };
  for (const auto & entry : mode_map) {
    const KEY_MODE m = entry.second;
    mode_srvs_.push_back(
      this->create_service<std_srvs::srv::Empty>(
        entry.first,
        [this, m](
          const std::shared_ptr<std_srvs::srv::Empty::Request>,
          std::shared_ptr<std_srvs::srv::Empty::Response>) {
          this->resetInteraction();
          this->setMode(m);
          this->resetInteraction();
        }));
  }

  RCLCPP_INFO(this->get_logger(), "image_view2 subscribing image '%s'", camera.c_str());
}

ImageView2::~ImageView2()
{
  if (use_window && window_initialized_) {
    cv::destroyWindow(window_name_);
  }
}

void ImageView2::markerCb(const Marker::ConstSharedPtr & marker)
{
  // Make a mutable copy and convert a relative lifetime into an absolute
  // expiry time (seconds since epoch), matching the ROS 1 behavior.
  auto m = std::make_shared<Marker>(*marker);
  if (durationToSec(m->lifetime) != 0.0) {
    double expiry = this->now().seconds() + durationToSec(marker->lifetime);
    m->lifetime.sec = static_cast<int32_t>(expiry);
    m->lifetime.nanosec =
      static_cast<uint32_t>((expiry - static_cast<double>(m->lifetime.sec)) * 1e9);
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    marker_queue_.push_back(m);
  }
  redraw();
}

// ---- interaction (Phase 3) ----

void ImageView2::setMode(KEY_MODE mode) {mode_ = mode;}
ImageView2::KEY_MODE ImageView2::getMode() {return mode_;}

ImageView2::KEY_MODE ImageView2::stringToMode(const std::string & s)
{
  if (s == "rectangle") {return MODE_RECTANGLE;}
  if (s == "freeform" || s == "series") {return MODE_SERIES;}
  if (s == "grabcut") {return MODE_SELECT_FORE_AND_BACK;}
  if (s == "grabcut_rect") {return MODE_SELECT_FORE_AND_BACK_RECT;}
  if (s == "line") {return MODE_LINE;}
  if (s == "poly") {return MODE_POLY;}
  if (s == "none") {return MODE_NONE;}
  throw std::runtime_error("Unknown mode: " + s);
}

bool ImageView2::toggleSelection()
{
  std::lock_guard<std::mutex> lock(point_array_mutex_);
  selecting_fg_ = !selecting_fg_;
  return selecting_fg_;
}

void ImageView2::addPoint(int x, int y)
{
  std::lock_guard<std::mutex> lock(point_array_mutex_);
  point_array_.push_back(cv::Point2d(x, y));
}

void ImageView2::addRegionPoint(int x, int y)
{
  std::lock_guard<std::mutex> lock(point_array_mutex_);
  (selecting_fg_ ? point_fg_array_ : point_bg_array_).push_back(cv::Point2d(x, y));
}

void ImageView2::setRegionWindowPoint(int x, int y)
{
  std::lock_guard<std::mutex> lock(point_array_mutex_);
  cv::Rect & r = selecting_fg_ ? rect_fg_ : rect_bg_;
  r.x = x; r.y = y; r.width = 0; r.height = 0;
}

void ImageView2::updateRegionWindowSize(int x, int y)
{
  std::lock_guard<std::mutex> lock(point_array_mutex_);
  cv::Rect & r = selecting_fg_ ? rect_fg_ : rect_bg_;
  r.width = x - r.x; r.height = y - r.y;
}

void ImageView2::checkMousePos(int & x, int & y)
{
  if (last_msg_) {
    x = std::max(std::min(x, static_cast<int>(last_msg_->width)), 0);
    y = std::max(std::min(y, static_cast<int>(last_msg_->height)), 0);
  }
}

bool ImageView2::isValidMovement(const cv::Point2f & p0, const cv::Point2f & p1)
{
  return cv::norm(cv::Mat(p0), cv::Mat(p1)) > 3.0;
}

void ImageView2::pointArrayToMask(std::vector<cv::Point2d> & points, cv::Mat & mask)
{
  if (points.size() > 1) {
    cv::Point2d from = points[0];
    for (size_t i = 1; i < points.size(); ++i) {
      cv::line(mask, from, points[i], cv::Scalar(255), 8, 8, 0);
      from = points[i];
    }
  }
}

void ImageView2::publishMonoImage(
  const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
  const cv::Mat & image, const std_msgs::msg::Header & header)
{
  cv_bridge::CvImage bridge(header, sensor_msgs::image_encodings::MONO8, image);
  pub->publish(*bridge.toImageMsg());
}

void ImageView2::publishRectFromMaskImage(
  const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr & pub,
  const cv::Mat & image, const std_msgs::msg::Header & header)
{
  int min_x = image.cols, min_y = image.rows, max_x = 0, max_y = 0;
  for (int j = 0; j < image.rows; ++j) {
    for (int i = 0; i < image.cols; ++i) {
      if (image.at<uchar>(j, i) != 0) {
        min_x = std::min(min_x, i); min_y = std::min(min_y, j);
        max_x = std::max(max_x, i); max_y = std::max(max_y, j);
      }
    }
  }
  geometry_msgs::msg::PolygonStamped poly;
  poly.header = header;
  geometry_msgs::msg::Point32 min_pt, max_pt;
  min_pt.x = min_x; min_pt.y = min_y;
  max_pt.x = max_x; max_pt.y = max_y;
  poly.polygon.points.push_back(min_pt);
  poly.polygon.points.push_back(max_pt);
  pub->publish(poly);
}

void ImageView2::publishForegroundBackgroundMask()
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  std::lock_guard<std::mutex> lock2(point_array_mutex_);
  if (!last_msg_) {return;}
  cv::Mat fg = cv::Mat::zeros(last_msg_->height, last_msg_->width, CV_8UC1);
  cv::Mat bg = cv::Mat::zeros(last_msg_->height, last_msg_->width, CV_8UC1);
  if (mode_ == MODE_SELECT_FORE_AND_BACK) {
    pointArrayToMask(point_fg_array_, fg);
    pointArrayToMask(point_bg_array_, bg);
  } else if (mode_ == MODE_SELECT_FORE_AND_BACK_RECT) {
    cv::rectangle(fg, rect_fg_, cv::Scalar(255), cv::FILLED);
    cv::rectangle(bg, rect_bg_, cv::Scalar(255), cv::FILLED);
  }
  publishMonoImage(foreground_mask_pub_, fg, last_msg_->header);
  publishMonoImage(background_mask_pub_, bg, last_msg_->header);
  publishRectFromMaskImage(foreground_rect_pub_, fg, last_msg_->header);
  publishRectFromMaskImage(background_rect_pub_, bg, last_msg_->header);
}

void ImageView2::publishLinePoints()
{
  std::lock_guard<std::mutex> lock(line_point_mutex_);
  geometry_msgs::msg::PolygonStamped line;
  line.header = last_msg_->header;
  geometry_msgs::msg::Point32 s, e;
  s.x = line_start_point_.x; s.y = line_start_point_.y;
  e.x = line_end_point_.x; e.y = line_end_point_.y;
  line.polygon.points.push_back(s);
  line.polygon.points.push_back(e);
  line_pub_->publish(line);
}

void ImageView2::publishPolyPoints()
{
  std::lock_guard<std::mutex> lock(poly_point_mutex_);
  geometry_msgs::msg::PolygonStamped poly;
  poly.header = last_msg_->header;
  for (const auto & p : poly_points_) {
    geometry_msgs::msg::Point32 p32;
    p32.x = p.x; p32.y = p.y; p32.z = 0;
    poly.polygon.points.push_back(p32);
  }
  poly_pub_->publish(poly);
}

void ImageView2::publishPointArray()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = this->now();
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(point_array_.size());
  sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  for (const auto & p : point_array_) {
    *ix = p.x; *iy = p.y; *iz = 0.0f;
    ++ix; ++iy; ++iz;
  }
  point_array_pub_->publish(cloud);
}

void ImageView2::publishMouseInteractionResult()
{
  if (mode_ == MODE_SERIES) {
    publishPointArray();
  } else if (mode_ == MODE_SELECT_FORE_AND_BACK ||
    mode_ == MODE_SELECT_FORE_AND_BACK_RECT)
  {
    publishForegroundBackgroundMask();
  } else if (mode_ == MODE_LINE) {
    publishLinePoints();
  } else {
    cv::Point2f p0(window_selection_.x, window_selection_.y);
    cv::Point2f p1(button_up_pos_);
    if (!isValidMovement(p0, p1)) {
      geometry_msgs::msg::PointStamped screen_msg;
      screen_msg.point.x = window_selection_.x * resize_x_;
      screen_msg.point.y = window_selection_.y * resize_y_;
      screen_msg.point.z = 0;
      screen_msg.header.stamp = last_msg_->header.stamp;
      point_pub_->publish(screen_msg);
    } else {
      geometry_msgs::msg::PolygonStamped screen_msg;
      screen_msg.polygon.points.resize(2);
      screen_msg.polygon.points[0].x = window_selection_.x * resize_x_;
      screen_msg.polygon.points[0].y = window_selection_.y * resize_y_;
      screen_msg.polygon.points[1].x =
        (window_selection_.x + window_selection_.width) * resize_x_;
      screen_msg.polygon.points[1].y =
        (window_selection_.y + window_selection_.height) * resize_y_;
      screen_msg.header = last_msg_->header;
      rectangle_pub_->publish(screen_msg);
      continuous_ready_ = true;
    }
  }
}

void ImageView2::drawInteraction()
{
  if (mode_ == MODE_RECTANGLE) {
    cv::rectangle(
      draw_, cv::Point(window_selection_.x, window_selection_.y),
      cv::Point(window_selection_.x + window_selection_.width,
                window_selection_.y + window_selection_.height),
      defaultColor(), 3, 8, 0);
  } else if (mode_ == MODE_SERIES) {
    std::lock_guard<std::mutex> lock(point_array_mutex_);
    for (size_t i = 1; i < point_array_.size(); ++i) {
      cv::line(draw_, point_array_[i - 1], point_array_[i], defaultColor(), 2, 8, 0);
    }
  } else if (mode_ == MODE_SELECT_FORE_AND_BACK) {
    std::lock_guard<std::mutex> lock(point_array_mutex_);
    for (size_t i = 1; i < point_fg_array_.size(); ++i) {
      cv::line(draw_, point_fg_array_[i - 1], point_fg_array_[i],
        cv::Scalar(0, 0, 255), 8, 8, 0);
    }
    for (size_t i = 1; i < point_bg_array_.size(); ++i) {
      cv::line(draw_, point_bg_array_[i - 1], point_bg_array_[i],
        cv::Scalar(0, 255, 0), 8, 8, 0);
    }
  } else if (mode_ == MODE_SELECT_FORE_AND_BACK_RECT) {
    std::lock_guard<std::mutex> lock(point_array_mutex_);
    if (rect_fg_.width != 0 && rect_fg_.height != 0) {
      cv::rectangle(draw_, rect_fg_, cv::Scalar(0, 0, 255), 4);
    }
    if (rect_bg_.width != 0 && rect_bg_.height != 0) {
      cv::rectangle(draw_, rect_bg_, cv::Scalar(0, 255, 0), 4);
    }
  } else if (mode_ == MODE_LINE) {
    std::lock_guard<std::mutex> lock(line_point_mutex_);
    if (line_selected_) {
      cv::line(draw_, line_start_point_, line_end_point_, cv::Scalar(0, 255, 0), 8, 8, 0);
    }
  } else if (mode_ == MODE_POLY) {
    std::lock_guard<std::mutex> lock(poly_point_mutex_);
    if (!poly_points_.empty()) {
      for (size_t i = 0; i + 1 < poly_points_.size(); ++i) {
        cv::line(draw_, poly_points_[i], poly_points_[i + 1], cv::Scalar(0, 0, 255), 8, 8, 0);
      }
      if (poly_selecting_done_) {
        cv::line(draw_, poly_points_.back(), poly_points_.front(),
          cv::Scalar(0, 0, 255), 8, 8, 0);
      } else {
        cv::line(draw_, poly_points_.back(), poly_selecting_point_,
          cv::Scalar(0, 255, 0), 8, 8, 0);
      }
    }
  }
}

void ImageView2::cropROI()
{
  if (mode_ != MODE_RECTANGLE) {return;}
  if (window_selection_.width <= 0 || window_selection_.height <= 0) {return;}
  cv::Rect screen_rect(
    cv::Point(window_selection_.x, window_selection_.y),
    cv::Point(window_selection_.x + window_selection_.width,
              window_selection_.y + window_selection_.height));
  screen_rect &= cv::Rect(0, 0, original_image_.cols, original_image_.rows);
  if (screen_rect.width <= 0 || screen_rect.height <= 0) {return;}
  cv::Mat cropped = original_image_(screen_rect);
  rectangle_img_pub_->publish(
    *cv_bridge::CvImage(last_msg_->header, "bgr8", cropped).toImageMsg());
}

void ImageView2::processLeftButtonDown(int x, int y)
{
  left_button_clicked_ = true;
  continuous_ready_ = false;
  window_selection_.x = x;
  window_selection_.y = y;
  window_selection_.width = window_selection_.height = 0;
  if (mode_ == MODE_SELECT_FORE_AND_BACK_RECT) {setRegionWindowPoint(x, y);}
}

void ImageView2::processMove(int x, int y)
{
  if (left_button_clicked_) {
    cv::Point2f p0(window_selection_.x, window_selection_.y);
    cv::Point2f p1(x, y);
    if (isValidMovement(p0, p1)) {
      if (mode_ == MODE_RECTANGLE) {
        window_selection_.width = x - window_selection_.x;
        window_selection_.height = y - window_selection_.y;
      } else if (mode_ == MODE_SERIES) {
        addPoint(x, y);
      } else if (mode_ == MODE_SELECT_FORE_AND_BACK) {
        addRegionPoint(x, y);
      } else if (mode_ == MODE_SELECT_FORE_AND_BACK_RECT) {
        updateRegionWindowSize(x, y);
      }
    }
    geometry_msgs::msg::PointStamped move_point;
    move_point.header.stamp = this->now();
    move_point.point.x = x; move_point.point.y = y; move_point.point.z = 0;
    move_point_pub_->publish(move_point);
  } else {
    if (mode_ == MODE_LINE) {
      std::lock_guard<std::mutex> lock(line_point_mutex_);
      if (!line_select_start_point_) {line_end_point_ = cv::Point(x, y);}
    } else if (mode_ == MODE_POLY) {
      std::lock_guard<std::mutex> lock(poly_point_mutex_);
      poly_selecting_point_ = cv::Point(x, y);
    }
  }
}

void ImageView2::processLeftButtonUp(int x, int y)
{
  if (!left_button_clicked_) {return;}
  if (mode_ == MODE_SERIES) {
    publishMouseInteractionResult();
    std::lock_guard<std::mutex> lock(point_array_mutex_);
    point_array_.clear();
  } else if (mode_ == MODE_SELECT_FORE_AND_BACK ||
    mode_ == MODE_SELECT_FORE_AND_BACK_RECT)
  {
    if (toggleSelection()) {
      publishMouseInteractionResult();
      continuous_ready_ = true;
    }
  } else if (mode_ == MODE_RECTANGLE) {
    button_up_pos_ = cv::Point2f(x, y);
    publishMouseInteractionResult();
  } else if (mode_ == MODE_LINE) {
    {
      std::lock_guard<std::mutex> lock(line_point_mutex_);
      if (line_select_start_point_) {
        line_start_point_ = cv::Point(x, y);
        line_end_point_ = cv::Point(x, y);
      } else {
        line_end_point_ = cv::Point(x, y);
      }
      line_select_start_point_ = !line_select_start_point_;
      line_selected_ = true;
    }
    bool selecting_start;
    {
      std::lock_guard<std::mutex> lock(line_point_mutex_);
      selecting_start = line_select_start_point_;
    }
    if (selecting_start) {
      publishMouseInteractionResult();
      continuous_ready_ = true;
    }
  } else if (mode_ == MODE_POLY) {
    bool first_time;
    {
      std::lock_guard<std::mutex> lock(poly_point_mutex_);
      first_time = poly_selecting_done_;
    }
    if (first_time) {
      continuous_ready_ = false;
      std::lock_guard<std::mutex> lock(poly_point_mutex_);
      poly_selecting_done_ = false;
      poly_points_.clear();
    }
    std::lock_guard<std::mutex> lock(poly_point_mutex_);
    poly_points_.push_back(cv::Point(x, y));
  }
  left_button_clicked_ = false;
}

void ImageView2::processMouseEvent(int event, int x, int y)
{
  checkMousePos(x, y);
  switch (event) {
    case cv::EVENT_MOUSEMOVE:
      processMove(x, y);
      break;
    case cv::EVENT_LBUTTONDOWN:
      processLeftButtonDown(x, y);
      break;
    case cv::EVENT_LBUTTONUP:
      processLeftButtonUp(x, y);
      break;
    case cv::EVENT_RBUTTONDOWN:
      if (mode_ == MODE_POLY) {
        {
          std::lock_guard<std::mutex> lock(poly_point_mutex_);
          poly_selecting_done_ = true;
        }
        publishPolyPoints();
        continuous_ready_ = true;
      } else {
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (!image_.empty()) {
          char buf[1024];
          std::snprintf(buf, sizeof(buf), filename_format_.c_str(), count_);
          cv::imwrite(buf, image_);
          RCLCPP_INFO(this->get_logger(), "Saved image %s", buf);
          count_++;
        } else {
          RCLCPP_WARN(this->get_logger(), "Couldn't save image, no data!");
        }
      }
      break;
    default:
      break;
  }
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (!original_image_.empty()) {redraw();}
}

void ImageView2::mouseCb(int event, int x, int y, int /*flags*/, void * param)
{
  auto * iv = reinterpret_cast<ImageView2 *>(param);
  iv->processMouseEvent(event, x, y);
}

void ImageView2::resetInteraction()
{
  if (mode_ == MODE_SELECT_FORE_AND_BACK) {
    std::lock_guard<std::mutex> lock(point_array_mutex_);
    point_fg_array_.clear(); point_bg_array_.clear(); selecting_fg_ = true;
  } else if (mode_ == MODE_SELECT_FORE_AND_BACK_RECT) {
    std::lock_guard<std::mutex> lock(point_array_mutex_);
    rect_fg_.width = rect_fg_.height = 0;
    rect_bg_.width = rect_bg_.height = 0;
    selecting_fg_ = true;
  } else if (mode_ == MODE_LINE) {
    std::lock_guard<std::mutex> lock(line_point_mutex_);
    line_select_start_point_ = true;
    line_selected_ = false;
    continuous_ready_ = false;
  } else if (mode_ == MODE_RECTANGLE) {
    button_up_pos_ = cv::Point2f(0, 0);
    window_selection_.width = 0;
    window_selection_.height = 0;
  } else if (mode_ == MODE_POLY) {
    std::lock_guard<std::mutex> lock(poly_point_mutex_);
    poly_selecting_done_ = false;
    poly_points_.clear();
  }
}

void ImageView2::changeModeCb(
  const std::shared_ptr<image_view2::srv::ChangeMode::Request> req,
  std::shared_ptr<image_view2::srv::ChangeMode::Response>)
{
  resetInteraction();
  try {
    setMode(stringToMode(req->mode));
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
  }
  resetInteraction();
}

void ImageView2::eventCb(const image_view2::msg::MouseEvent::ConstSharedPtr & msg)
{
  using MouseEvent = image_view2::msg::MouseEvent;
  if (msg->type == MouseEvent::KEY_PRESSED) {
    pressKey(msg->key);
    return;
  }
  int x, y;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!last_msg_) {
      RCLCPP_WARN(this->get_logger(), "Image is not yet available");
      return;
    }
    if (msg->width == 0 || msg->height == 0) {return;}
    x = (static_cast<float>(last_msg_->width) / msg->width) * msg->x;
    y = (static_cast<float>(last_msg_->height) / msg->height) * msg->y;
  }
  switch (msg->type) {
    case MouseEvent::MOUSE_LEFT_UP:
      processMouseEvent(cv::EVENT_LBUTTONUP, x, y);
      break;
    case MouseEvent::MOUSE_LEFT_DOWN:
      processMouseEvent(cv::EVENT_LBUTTONDOWN, x, y);
      break;
    case MouseEvent::MOUSE_MOVE:
      processMouseEvent(cv::EVENT_MOUSEMOVE, x, y);
      break;
    case MouseEvent::MOUSE_RIGHT_DOWN:
      processMouseEvent(cv::EVENT_RBUTTONDOWN, x, y);
      break;
    default:
      break;
  }
}

void ImageView2::infoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  info_msg_ = msg;
}

bool ImageView2::lookupTransform(
  const std::string & frame_id, const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & tf_out)
{
  try {
    // target = camera optical frame, source = marker frame
    tf_out = tf_buffer_->lookupTransform(
      cam_model_.tfFrame(), frame_id, stamp,
      rclcpp::Duration::from_seconds(tf_timeout_));
    tf_fail_[frame_id] = 0;
    return true;
  } catch (const tf2::TransformException & ex) {
    if (++tf_fail_[frame_id] < 5) {
      RCLCPP_ERROR(this->get_logger(), "[image_view2] TF exception:\n%s", ex.what());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "[image_view2] TF exception:\n%s", ex.what());
    }
    return false;
  }
}

cv::Point2d ImageView2::projectPoint(
  const geometry_msgs::msg::Point & p, const std::string & frame_id,
  const geometry_msgs::msg::TransformStamped & tf)
{
  geometry_msgs::msg::PointStamped in, out;
  in.header.frame_id = frame_id;
  in.point = p;
  tf2::doTransform(in, out, tf);
  return cam_model_.project3dToPixel(
    cv::Point3d(out.point.x, out.point.y, out.point.z));
}

void ImageView2::drawFrames(const MarkerConstPtr & marker)
{
  const rclcpp::Time stamp(last_msg_->header.stamp);
  for (const std::string & frame_id : marker->frames) {
    geometry_msgs::msg::TransformStamped tf;
    if (!lookupTransform(frame_id, stamp, tf)) {continue;}

    geometry_msgs::msg::Point origin, px, py, pz;
    px.x = 0.05; py.y = 0.05; pz.z = 0.05;
    cv::Point2d uv = projectPoint(origin, frame_id, tf);
    cv::Point2d uv0 = projectPoint(px, frame_id, tf);
    cv::Point2d uv1 = projectPoint(py, frame_id, tf);
    cv::Point2d uv2 = projectPoint(pz, frame_id, tf);

    static const int RADIUS = 3;
    cv::circle(draw_, uv, RADIUS, defaultColor(), -1);
    cv::line(draw_, uv, uv0, cv::Scalar(0, 0, 255), 2);  // x red
    cv::line(draw_, uv, uv1, cv::Scalar(0, 255, 0), 2);  // y green
    cv::line(draw_, uv, uv2, cv::Scalar(255, 0, 0), 2);  // z blue

    int baseline;
    cv::Size text_size = cv::getTextSize(frame_id, font_, 1.0, 1.0, &baseline);
    cv::Point origin_txt(uv.x - text_size.width / 2, uv.y - RADIUS - baseline - 3);
    cv::putText(draw_, frame_id, origin_txt, font_, 1.0, defaultColor(), 2);
  }
}

void ImageView2::drawLineStrip3D(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  const std::string frame_id = marker->points_3d.header.frame_id;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTransform(frame_id, rclcpp::Time(last_msg_->header.stamp), tf)) {return;}
  if (marker->points_3d.points.empty()) {return;}

  std::vector<cv::Point2d> pts;
  for (const auto & p : marker->points_3d.points) {
    pts.push_back(projectPoint(p, frame_id, tf));
  }
  cv::Point2d p0 = pts.front();
  for (size_t i = 1; i < pts.size(); ++i) {
    cv::line(draw_, p0, pts[i], *col_it,
      (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    p0 = pts[i];
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
}

void ImageView2::drawLineList3D(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  const std::string frame_id = marker->points_3d.header.frame_id;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTransform(frame_id, rclcpp::Time(last_msg_->header.stamp), tf)) {return;}

  std::vector<cv::Point2d> pts;
  for (const auto & p : marker->points_3d.points) {
    pts.push_back(projectPoint(p, frame_id, tf));
  }
  for (size_t i = 0; i + 1 < pts.size(); i += 2) {
    cv::line(draw_, pts[i], pts[i + 1], *col_it,
      (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
}

void ImageView2::drawPolygon3D(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  const std::string frame_id = marker->points_3d.header.frame_id;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTransform(frame_id, rclcpp::Time(last_msg_->header.stamp), tf)) {return;}
  if (marker->points_3d.points.empty()) {return;}

  std::vector<cv::Point2d> pts;
  std::vector<cv::Point> fill_pts;
  for (const auto & p : marker->points_3d.points) {
    cv::Point2d uv = projectPoint(p, frame_id, tf);
    pts.push_back(uv);
    if (marker->filled) {fill_pts.push_back(cv::Point(uv.x, uv.y));}
  }
  cv::Point2d p0 = pts.front();
  for (size_t i = 1; i < pts.size(); ++i) {
    cv::line(draw_, p0, pts[i], *col_it,
      (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    p0 = pts[i];
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
  cv::line(draw_, p0, pts.front(), *col_it,
    (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
  if (marker->filled && fill_pts.size() >= 3) {
    cv::fillConvexPoly(draw_, fill_pts.data(), fill_pts.size(), msgToRGB(marker->fill_color));
  }
}

void ImageView2::drawPoints3D(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  const std::string frame_id = marker->points_3d.header.frame_id;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTransform(frame_id, rclcpp::Time(last_msg_->header.stamp), tf)) {return;}

  for (const auto & p : marker->points_3d.points) {
    cv::Point2d uv = projectPoint(p, frame_id, tf);
    cv::circle(draw_, uv, (marker->scale == 0 ? 3 : marker->scale), *col_it, -1);
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
}

void ImageView2::drawText3D(const MarkerConstPtr & marker)
{
  const std::string frame_id = marker->position_3d.header.frame_id;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTransform(frame_id, rclcpp::Time(last_msg_->header.stamp), tf)) {return;}

  cv::Point2d uv = projectPoint(marker->position_3d.point, frame_id, tf);
  int baseline;
  float scale = marker->scale;
  if (scale == 0) {scale = 1.0;}
  cv::Size text_size = cv::getTextSize(marker->text, font_, scale, scale, &baseline);
  cv::Point origin(uv.x - text_size.width / 2, uv.y - baseline - 3);
  cv::putText(draw_, marker->text, origin, font_, scale, cv::Scalar(0, 255, 0), 3);
}

void ImageView2::drawCircle3D(const MarkerConstPtr & marker)
{
  const std::string frame_id = marker->pose.header.frame_id;
  geometry_msgs::msg::TransformStamped tf;
  if (!lookupTransform(frame_id, rclcpp::Time(last_msg_->header.stamp), tf)) {return;}

  geometry_msgs::msg::PoseStamped pose_in, pose_out;
  pose_in.header = marker->pose.header;
  pose_in.pose = marker->pose.pose;
  tf2::doTransform(pose_in, pose_out, tf);

  tf2::Quaternion q;
  tf2::fromMsg(pose_out.pose.orientation, q);
  tf2::Matrix3x3 rot(q);
  const double angle = (marker->arc == 0 ? 360.0 : marker->angle);
  const double scale = (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale);
  const int N = 100;
  std::vector<std::vector<cv::Point>> ptss(1);
  std::vector<cv::Point> & pts = ptss[0];
  for (int i = 0; i < N; ++i) {
    double th = angle * i / N * (M_PI / 180.0);
    tf2::Vector3 v = rot * tf2::Vector3(scale * std::cos(th), scale * std::sin(th), 0);
    cv::Point2d pt = cam_model_.project3dToPixel(
      cv::Point3d(pose_out.pose.position.x + v.x(),
                  pose_out.pose.position.y + v.y(),
                  pose_out.pose.position.z + v.z()));
    pts.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
  }
  cv::polylines(
    draw_, ptss, (marker->arc == 0), msgToRGB(marker->outline_color),
    (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
  if (marker->filled) {
    if (marker->arc != 0) {
      cv::Point2d c = cam_model_.project3dToPixel(
        cv::Point3d(pose_out.pose.position.x, pose_out.pose.position.y,
                    pose_out.pose.position.z));
      pts.push_back(cv::Point(static_cast<int>(c.x), static_cast<int>(c.y)));
    }
    cv::fillPoly(draw_, ptss, msgToRGB(marker->fill_color));
  }
}

void ImageView2::drawCircle(const MarkerConstPtr & marker)
{
  cv::Point2d uv = cv::Point2d(marker->position.x, marker->position.y);
  if (blurry_mode_) {
    int s0 = (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width);
    cv::Scalar co = msgToRGB(marker->outline_color);
    for (int s1 = s0 * 10; s1 >= s0; s1--) {
      double m = std::pow((1.0 - static_cast<double>(s1 - s0) / (s0 * 9)), 2);
      cv::circle(
        draw_, uv, (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale),
        cv::Scalar(co.val[0] * m, co.val[1] * m, co.val[2] * m), s1);
    }
  } else {
    cv::circle(
      draw_, uv, (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale),
      msgToRGB(marker->outline_color),
      (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    if (marker->filled) {
      cv::circle(
        draw_, uv,
        (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale) -
        (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width) / 2.0,
        msgToRGB(marker->fill_color), -1);
    }
  }
}

void ImageView2::drawLineStrip(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  if (marker->points.empty()) {return;}
  cv::Point2d p0, p1;
  auto it = marker->points.begin();
  auto end = marker->points.end();
  p0 = cv::Point2d(it->x, it->y); ++it;
  for (; it != end; ++it) {
    p1 = cv::Point2d(it->x, it->y);
    cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    p0 = p1;
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
}

void ImageView2::drawLineList(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  cv::Point2d p0, p1;
  auto it = marker->points.begin();
  auto end = marker->points.end();
  for (; it != end; ) {
    p0 = cv::Point2d(it->x, it->y); ++it;
    if (it != end) {p1 = cv::Point2d(it->x, it->y);}
    cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    ++it;
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
}

void ImageView2::drawPolygon(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  if (marker->points.empty()) {return;}
  cv::Point2d p0, p1;
  auto it = marker->points.begin();
  auto end = marker->points.end();
  std::vector<cv::Point> points;

  if (marker->filled) {points.push_back(cv::Point(it->x, it->y));}
  p0 = cv::Point2d(it->x, it->y); ++it;
  for (; it != end; ++it) {
    p1 = cv::Point2d(it->x, it->y);
    if (marker->filled) {points.push_back(cv::Point(it->x, it->y));}
    cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    p0 = p1;
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
  it = marker->points.begin();
  p1 = cv::Point2d(it->x, it->y);
  cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
  if (marker->filled && points.size() >= 3) {
    cv::fillConvexPoly(draw_, points.data(), points.size(), msgToRGB(marker->fill_color));
  }
}

void ImageView2::drawPoints(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> & colors,
  std::vector<cv::Scalar>::iterator & col_it)
{
  for (const auto & p : marker->points) {
    cv::Point2d uv = cv::Point2d(p.x, p.y);
    if (blurry_mode_) {
      int s0 = (marker->scale == 0 ? 3 : marker->scale);
      cv::Scalar co = (*col_it);
      for (int s1 = s0 * 2; s1 >= s0; s1--) {
        double m = std::pow((1.0 - static_cast<double>(s1 - s0) / s0), 2);
        cv::circle(
          draw_, uv, s1, cv::Scalar(co.val[0] * m, co.val[1] * m, co.val[2] * m), -1);
      }
    } else {
      cv::circle(draw_, uv, (marker->scale == 0 ? 3 : marker->scale), *col_it, -1);
    }
    if (++col_it == colors.end()) {col_it = colors.begin();}
  }
}

cv::Point ImageView2::ratioPoint(double x, double y)
{
  if (last_msg_) {
    return cv::Point(last_msg_->width * x, last_msg_->height * y);
  }
  return cv::Point(0, 0);
}

void ImageView2::drawText(
  const MarkerConstPtr & marker, std::vector<cv::Scalar> &,
  std::vector<cv::Scalar>::iterator &)
{
  int baseline;
  float scale = marker->scale;
  if (scale == 0) {scale = 1.0;}
  cv::Size text_size = cv::getTextSize(marker->text, font_, scale, scale, &baseline);
  if (marker->ratio_scale && last_msg_) {
    cv::Size a_size = cv::getTextSize("A", font_, 1.0, 1.0, &baseline);
    scale = (last_msg_->height * marker->scale) / a_size.height;
    text_size = cv::getTextSize(marker->text, font_, scale, scale, &baseline);
  }

  cv::Point origin;
  if (marker->left_up_origin) {
    origin = marker->ratio_scale ?
      ratioPoint(marker->position.x, marker->position.y) :
      cv::Point(marker->position.x, marker->position.y);
  } else {
    if (marker->ratio_scale) {
      cv::Point p = ratioPoint(marker->position.x, marker->position.y);
      origin = cv::Point(p.x - text_size.width / 2, p.y + baseline + 3);
    } else {
      origin = cv::Point(
        marker->position.x - text_size.width / 2, marker->position.y + baseline + 3);
    }
  }

  cv::putText(
    draw_, marker->text, origin, font_, scale, defaultColor(),
    marker->filled ? marker->filled : 1);
}

void ImageView2::createDistortGridImage()
{
  distort_grid_mask_ = cv::Mat::zeros(draw_.size(), draw_.type());
  cv::Mat mask(draw_.size(), CV_8UC1);
  mask.setTo(cv::Scalar(0));
  const float K = 341.0;
  const float r_divier = 1.0f / (draw_.cols / 2);
  const float center_x = mask.rows / 2.0f;
  const float center_y = mask.cols / 2.0f;
  for (int degree = -80; degree <= 80; degree += std::max(space_, 1)) {
    double c = draw_.cols / 2.0 * std::tan(degree * M_PI / 180.0);
    for (float theta = -1.57f; theta <= 1.57f; theta += 0.001f) {
      double sin_phi = c * r_divier / std::tan(theta);
      if (sin_phi > 1.0 || sin_phi < -1.0) {continue;}
      int x1 = K * theta * std::sqrt(1 - sin_phi * sin_phi) + center_x;
      int y1 = K * theta * sin_phi + center_y;
      int x2 = K * theta * sin_phi + center_x;
      int y2 = K * theta * std::sqrt(1 - sin_phi * sin_phi) + center_y;
      cv::circle(mask, cv::Point(y1, x1), 2, 1, grid_thickness_);
      cv::circle(mask, cv::Point(y2, x2), 2, 1, grid_thickness_);
    }
  }
  cv::Mat color(draw_.size(), draw_.type());
  color = cv::Scalar(grid_blue_, grid_green_, grid_red_);
  color.copyTo(distort_grid_mask_, mask);
}

void ImageView2::drawGrid()
{
  if (fisheye_mode_) {
    if (space_ != prev_space_ || prev_red_ != grid_red_ ||
      prev_green_ != grid_green_ || prev_blue_ != grid_blue_ ||
      prev_thickness_ != grid_thickness_)
    {
      createDistortGridImage();
      prev_space_ = space_; prev_red_ = grid_red_; prev_green_ = grid_green_;
      prev_blue_ = grid_blue_; prev_thickness_ = grid_thickness_;
    }
    if (!distort_grid_mask_.empty() && distort_grid_mask_.size() == draw_.size()) {
      cv::add(draw_, distort_grid_mask_, draw_);
    }
  } else {
    const double w = last_msg_->width;
    const double h = last_msg_->height;
    const cv::Scalar red(0, 0, 255);
    cv::line(draw_, cv::Point2d(0, h / 2.0), cv::Point2d(w, h / 2.0), red, DEFAULT_LINE_WIDTH);
    cv::line(draw_, cv::Point2d(w / 2.0, 0), cv::Point2d(w / 2.0, h), red, DEFAULT_LINE_WIDTH);
    for (int i = 1; i < div_u_; ++i) {
      double y = h * i * 1.0 / div_u_;
      cv::line(draw_, cv::Point2d(0, y), cv::Point2d(w, y), red, 1);
    }
    for (int i = 1; i < div_v_; ++i) {
      double x = w * i * 1.0 / div_v_;
      cv::line(draw_, cv::Point2d(x, 0), cv::Point2d(x, h), red, 1);
    }
  }
}

rcl_interfaces::msg::SetParametersResult ImageView2::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(grid_mutex_);
  for (const auto & p : params) {
    const std::string & name = p.get_name();
    if (name == "grid") {draw_grid_ = p.as_bool();
    } else if (name == "fisheye_mode") {fisheye_mode_ = p.as_bool();
    } else if (name == "div_u") {div_u_ = p.as_int();
    } else if (name == "div_v") {div_v_ = p.as_int();
    } else if (name == "grid_red") {grid_red_ = p.as_int();
    } else if (name == "grid_green") {grid_green_ = p.as_int();
    } else if (name == "grid_blue") {grid_blue_ = p.as_int();
    } else if (name == "grid_thickness") {grid_thickness_ = p.as_int();
    } else if (name == "grid_space") {space_ = p.as_int();
    } else if (name == "blurry") {blurry_mode_ = p.as_bool();
    } else if (name == "show_info") {show_info_ = p.as_bool();
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void ImageView2::resolveLocalMarkerQueue()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!marker_queue_.empty()) {
      auto new_msg = marker_queue_.begin();
      // remove any existing marker with the same namespace and id
      for (auto message_it = local_queue_.begin(); message_it != local_queue_.end(); ) {
        if ((*new_msg)->ns == (*message_it)->ns && (*new_msg)->id == (*message_it)->id) {
          message_it = local_queue_.erase(message_it);
        } else {
          ++message_it;
        }
      }
      local_queue_.push_back(*new_msg);
      const bool clear_all =
        ((*new_msg)->action == Marker::REMOVE && (*new_msg)->id == -1);
      marker_queue_.erase(new_msg);
      if (clear_all) {local_queue_.clear();}
    }
  }

  // drop REMOVE markers and expired markers
  const double now_sec = this->now().seconds();
  for (auto it = local_queue_.begin(); it != local_queue_.end(); ) {
    const double lifetime = durationToSec((*it)->lifetime);
    if ((*it)->action == Marker::REMOVE ||
      (lifetime != 0.0 && lifetime < now_sec))
    {
      it = local_queue_.erase(it);
    } else {
      ++it;
    }
  }
}

void ImageView2::drawMarkers()
{
  resolveLocalMarkerQueue();
  for (auto & marker : local_queue_) {
    std::vector<cv::Scalar> colors;
    for (const auto & color : marker->outline_colors) {
      colors.push_back(msgToRGB(color));
    }
    if (colors.empty()) {colors.push_back(defaultColor());}
    std::vector<cv::Scalar>::iterator col_it = colors.begin();

    // FRAMES and 3D marker types require the camera model.
    const bool needs_camera =
      marker->type == Marker::FRAMES ||
      marker->type == Marker::LINE_STRIP3D ||
      marker->type == Marker::LINE_LIST3D ||
      marker->type == Marker::POLYGON3D ||
      marker->type == Marker::POINTS3D ||
      marker->type == Marker::TEXT3D ||
      marker->type == Marker::CIRCLE3D;
    if (needs_camera) {
      std::lock_guard<std::mutex> lock(info_mutex_);
      if (!info_msg_) {
        if (!warned_3d_) {
          RCLCPP_WARN(this->get_logger(), "[image_view2] camera_info could not be found");
          warned_3d_ = true;
        }
        continue;
      }
      cam_model_.fromCameraInfo(info_msg_);
    }

    switch (marker->type) {
      case Marker::CIRCLE:
        drawCircle(marker);
        break;
      case Marker::LINE_STRIP:
        drawLineStrip(marker, colors, col_it);
        break;
      case Marker::LINE_LIST:
        drawLineList(marker, colors, col_it);
        break;
      case Marker::POLYGON:
        drawPolygon(marker, colors, col_it);
        break;
      case Marker::POINTS:
        drawPoints(marker, colors, col_it);
        break;
      case Marker::TEXT:
        drawText(marker, colors, col_it);
        break;
      case Marker::FRAMES:
        drawFrames(marker);
        break;
      case Marker::LINE_STRIP3D:
        drawLineStrip3D(marker, colors, col_it);
        break;
      case Marker::LINE_LIST3D:
        drawLineList3D(marker, colors, col_it);
        break;
      case Marker::POLYGON3D:
        drawPolygon3D(marker, colors, col_it);
        break;
      case Marker::POINTS3D:
        drawPoints3D(marker, colors, col_it);
        break;
      case Marker::TEXT3D:
        drawText3D(marker);
        break;
      case Marker::CIRCLE3D:
        drawCircle3D(marker);
        break;
      default:
        RCLCPP_WARN_ONCE(
          this->get_logger(), "unknown marker type %d; skipping", marker->type);
        break;
    }
  }
}

void ImageView2::redraw()
{
  if (original_image_.empty()) {
    return;
  }
  original_image_.copyTo(image_);
  if (blurry_mode_) {
    draw_ = cv::Mat(image_.size(), image_.type(), cv::Scalar(0, 0, 0));
  } else {
    draw_ = image_;
  }
  drawMarkers();
  drawInteraction();
  cropROI();
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    if (draw_grid_) {drawGrid();}
  }
  if (blurry_mode_) {
    cv::addWeighted(image_, 0.9, draw_, 1.0, 0.0, image_);
  }

  cv_bridge::CvImage out_msg;
  out_msg.header = last_msg_->header;
  out_msg.encoding = "bgr8";
  out_msg.image = image_;
  image_pub_.publish(out_msg.toImageMsg());
  local_image_pub_.publish(out_msg.toImageMsg());
}

void ImageView2::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->width == 0 && msg->height == 0) {
    return;
  }
  static int count = 0;
  if (count < skip_draw_rate_) {
    count++;
    return;
  }
  count = 0;

  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (msg->encoding.find("bayer") != std::string::npos) {
      original_image_ = cv::Mat(
        msg->height, msg->width, CV_8UC1,
        const_cast<uint8_t *>(&msg->data[0]), msg->step);
    } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      cv::Mat input_image = cv_bridge::toCvCopy(msg)->image;
      double min, max;
      cv::minMaxIdx(input_image, &min, &max);
      cv::convertScaleAbs(input_image, original_image_, 255 / max);
    } else {
      try {
        original_image_ =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(
          this->get_logger(), "Unable to convert '%s' image to bgr8: %s",
          msg->encoding.c_str(), e.what());
        return;
      }
    }
    last_msg_ = msg;
    redraw();
  }
  // Released the image lock: publishForegroundBackgroundMask re-locks it.
  if (region_continuous_publish_ && continuous_ready_) {
    publishMouseInteractionResult();
  }
}

void ImageView2::pressKey(int key)
{
  if (key == 27) {  // ESC resets the current interaction
    resetInteraction();
  }
}

void ImageView2::showImage()
{
  if (!use_window) {return;}
  if (!window_initialized_) {
    cv::namedWindow(window_name_, autosize_ ? cv::WINDOW_AUTOSIZE : cv::WINDOW_NORMAL);
    cv::setMouseCallback(window_name_, &ImageView2::mouseCb, this);
    window_initialized_ = true;
  }
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (!image_.empty()) {
    cv::imshow(window_name_, image_);
    cv::waitKey(3);
  }
}

}  // namespace image_view2
