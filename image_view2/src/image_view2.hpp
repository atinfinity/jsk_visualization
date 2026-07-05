// -*- mode: c++ -*-
// ROS 2 port of image_view2 (JSK Lab, BSD license).
// Phase 1: minimal viewer -- input image + 2D ImageMarker2 overlay +
// marked-image republication (+ optional HighGUI window).
// Phase 2: camera_info + tf2 + 3D markers (FRAMES, *_3D).
// Interaction modes, grid and grabcut are added in later phases.
#ifndef IMAGE_VIEW2__IMAGE_VIEW2_HPP_
#define IMAGE_VIEW2__IMAGE_VIEW2_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <image_view2/msg/image_marker2.hpp>
#include <image_view2/msg/mouse_event.hpp>
#include <image_view2/srv/change_mode.hpp>

namespace image_view2
{

// DEFAULT drawing color (red). OpenCV matrices are BGR.
inline cv::Scalar defaultColor() { return cv::Scalar(0, 0, 255); }

// Convert std_msgs/ColorRGBA (0-1) to an OpenCV BGR scalar, falling back
// to the default color when the message is all-zero (unset).
inline cv::Scalar msgToRGB(const std_msgs::msg::ColorRGBA & color)
{
  if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0) {
    return defaultColor();
  }
  return cv::Scalar(color.b * 255, color.g * 255, color.r * 255);
}

class ImageView2 : public rclcpp::Node
{
public:
  using Marker = image_view2::msg::ImageMarker2;
  using MarkerConstPtr = Marker::SharedPtr;
  using V_ImageMarkerMessage = std::vector<MarkerConstPtr>;

  enum KEY_MODE
  {
    MODE_RECTANGLE,
    MODE_SERIES,
    MODE_SELECT_FORE_AND_BACK,
    MODE_SELECT_FORE_AND_BACK_RECT,
    MODE_LINE,
    MODE_POLY,
    MODE_NONE
  };

  explicit ImageView2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImageView2() override;

  // Called from the GUI thread (main) when use_window is true.
  void pressKey(int key);
  void showImage();
  // Static OpenCV HighGUI mouse callback (local window interaction).
  static void mouseCb(int event, int x, int y, int flags, void * param);

  bool use_window;

private:
  // callbacks
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void markerCb(const Marker::ConstSharedPtr & marker);
  void eventCb(const image_view2::msg::MouseEvent::ConstSharedPtr & msg);
  void infoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // tf / projection helpers (Phase 2)
  bool lookupTransform(
    const std::string & frame_id, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & tf_out);
  cv::Point2d projectPoint(
    const geometry_msgs::msg::Point & p, const std::string & frame_id,
    const geometry_msgs::msg::TransformStamped & tf);

  // rendering
  void redraw();
  void drawMarkers();
  void resolveLocalMarkerQueue();

  // 2D drawing helpers
  void drawCircle(const MarkerConstPtr & marker);
  void drawLineStrip(const MarkerConstPtr & marker,
                     std::vector<cv::Scalar> & colors,
                     std::vector<cv::Scalar>::iterator & col_it);
  void drawLineList(const MarkerConstPtr & marker,
                    std::vector<cv::Scalar> & colors,
                    std::vector<cv::Scalar>::iterator & col_it);
  void drawPolygon(const MarkerConstPtr & marker,
                   std::vector<cv::Scalar> & colors,
                   std::vector<cv::Scalar>::iterator & col_it);
  void drawPoints(const MarkerConstPtr & marker,
                  std::vector<cv::Scalar> & colors,
                  std::vector<cv::Scalar>::iterator & col_it);
  void drawText(const MarkerConstPtr & marker,
                std::vector<cv::Scalar> & colors,
                std::vector<cv::Scalar>::iterator & col_it);
  cv::Point ratioPoint(double x, double y);

  // 3D drawing helpers (Phase 2). Points are given in a tf frame and
  // projected into the image using camera_info + tf2.
  void drawFrames(const MarkerConstPtr & marker);
  void drawLineStrip3D(const MarkerConstPtr & marker,
                       std::vector<cv::Scalar> & colors,
                       std::vector<cv::Scalar>::iterator & col_it);
  void drawLineList3D(const MarkerConstPtr & marker,
                      std::vector<cv::Scalar> & colors,
                      std::vector<cv::Scalar>::iterator & col_it);
  void drawPolygon3D(const MarkerConstPtr & marker,
                     std::vector<cv::Scalar> & colors,
                     std::vector<cv::Scalar>::iterator & col_it);
  void drawPoints3D(const MarkerConstPtr & marker,
                    std::vector<cv::Scalar> & colors,
                    std::vector<cv::Scalar>::iterator & col_it);
  void drawText3D(const MarkerConstPtr & marker);
  void drawCircle3D(const MarkerConstPtr & marker);

  // grid / fisheye (Phase 4)
  void drawGrid();
  void createDistortGridImage();
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // interaction (Phase 3)
  void drawInteraction();
  void cropROI();
  void processMouseEvent(int event, int x, int y);
  void processLeftButtonDown(int x, int y);
  void processMove(int x, int y);
  void processLeftButtonUp(int x, int y);
  void publishMouseInteractionResult();
  void publishPointArray();
  void publishLinePoints();
  void publishPolyPoints();
  void publishForegroundBackgroundMask();
  void publishRectFromMaskImage(
    const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr & pub,
    const cv::Mat & image, const std_msgs::msg::Header & header);
  void publishMonoImage(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
    const cv::Mat & image, const std_msgs::msg::Header & header);
  void pointArrayToMask(std::vector<cv::Point2d> & points, cv::Mat & mask);
  bool isValidMovement(const cv::Point2f & p0, const cv::Point2f & p1);
  void checkMousePos(int & x, int & y);
  void resetInteraction();
  void setMode(KEY_MODE mode);
  KEY_MODE getMode();
  bool toggleSelection();
  KEY_MODE stringToMode(const std::string & str);
  void addPoint(int x, int y);
  void addRegionPoint(int x, int y);
  void setRegionWindowPoint(int x, int y);
  void updateRegionWindowSize(int x, int y);
  // service handlers
  void changeModeCb(
    const std::shared_ptr<image_view2::srv::ChangeMode::Request> req,
    std::shared_ptr<image_view2::srv::ChangeMode::Response> res);

  // subscriptions / publications
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher local_image_pub_;
  rclcpp::Subscription<Marker>::SharedPtr marker_sub_;
  rclcpp::Subscription<image_view2::msg::MouseEvent>::SharedPtr event_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  // camera model + tf (Phase 2)
  std::mutex info_mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg_;
  image_geometry::PinholeCameraModel cam_model_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double tf_timeout_;
  std::map<std::string, int> tf_fail_;

  std::string marker_topic_;
  std::string window_name_;
  std::string transport_;

  // marker queues
  V_ImageMarkerMessage marker_queue_;
  V_ImageMarkerMessage local_queue_;
  std::mutex queue_mutex_;

  // image buffers
  std::mutex image_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr last_msg_;
  cv::Mat original_image_, image_, draw_;

  // parameters
  int skip_draw_rate_;
  bool autosize_;
  bool blurry_mode_;
  bool show_info_;

  bool window_initialized_;
  int font_;

  // warn only once when camera_info is missing for 3D markers
  bool warned_3d_;

  // ---- interaction (Phase 3) ----
  KEY_MODE mode_;
  std::mutex point_array_mutex_;
  std::vector<cv::Point2d> point_array_;
  std::vector<cv::Point2d> point_fg_array_;
  std::vector<cv::Point2d> point_bg_array_;
  bool selecting_fg_;
  cv::Rect rect_fg_;
  cv::Rect rect_bg_;
  cv::Rect window_selection_;
  cv::Point2f button_up_pos_;
  bool left_button_clicked_;
  bool continuous_ready_;
  bool region_continuous_publish_;
  double resize_x_, resize_y_;
  std::string filename_format_;
  int count_;

  // line-mode state
  std::mutex line_point_mutex_;
  bool line_select_start_point_;
  bool line_selected_;
  cv::Point line_start_point_;
  cv::Point line_end_point_;
  // poly-mode state
  std::mutex poly_point_mutex_;
  std::vector<cv::Point2d> poly_points_;
  cv::Point poly_selecting_point_;
  bool poly_selecting_done_;

  // interaction publishers (all under the camera namespace)
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr rectangle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rectangle_img_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr move_point_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr foreground_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr background_mask_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr foreground_rect_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr background_rect_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr line_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr poly_pub_;

  // services
  rclcpp::Service<image_view2::srv::ChangeMode>::SharedPtr change_mode_srv_;
  std::vector<rclcpp::ServiceBase::SharedPtr> mode_srvs_;

  // ---- grid / fisheye (Phase 4) ----
  std::mutex grid_mutex_;
  bool draw_grid_;
  bool fisheye_mode_;
  int div_u_, div_v_;
  int grid_red_, grid_green_, grid_blue_;
  int grid_thickness_;
  int space_;
  int prev_space_, prev_red_, prev_green_, prev_blue_, prev_thickness_;
  cv::Mat distort_grid_mask_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace image_view2

#endif  // IMAGE_VIEW2__IMAGE_VIEW2_HPP_
