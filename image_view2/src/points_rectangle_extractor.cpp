// -*- mode: c++ -*-
// ROS 2 port of image_view2/points_rectangle_extractor (JSK Lab, BSD).
//
// Extracts a rectangular (or point-neighborhood) region from an organized
// PointCloud2 using a screenrectangle / screenpoint published by
// image_view2. The region is copied out by raw point bytes, so no PCL
// dependency is needed (the ROS 1 CMake link against PCL was unnecessary).
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

namespace image_view2
{

class PointsRectExtractor : public rclcpp::Node
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PolygonStamped = geometry_msgs::msg::PolygonStamped;
  using PointStamped = geometry_msgs::msg::PointStamped;
  using PolygonSyncPolicy =
    message_filters::sync_policies::ApproximateTime<PointCloud2, PolygonStamped>;
  using PointSyncPolicy =
    message_filters::sync_policies::ApproximateTime<PointCloud2, PointStamped>;

public:
  explicit PointsRectExtractor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("points_rectangle_extractor", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 200);
    crop_width_ = this->declare_parameter("crop_width", 2);

    points_sub_.subscribe(this, "points");
    rect_sub_.subscribe(this, "rect");
    point_sub_.subscribe(this, "point");

    sync_polygon_ = std::make_shared<message_filters::Synchronizer<PolygonSyncPolicy>>(
      PolygonSyncPolicy(queue_size_), points_sub_, rect_sub_);
    sync_polygon_->registerCallback(
      std::bind(
        &PointsRectExtractor::callbackPolygon, this,
        std::placeholders::_1, std::placeholders::_2));

    sync_point_ = std::make_shared<message_filters::Synchronizer<PointSyncPolicy>>(
      PointSyncPolicy(queue_size_), points_sub_, point_sub_);
    sync_point_->registerCallback(
      std::bind(
        &PointsRectExtractor::callbackPoint, this,
        std::placeholders::_1, std::placeholders::_2));

    pub_ = this->create_publisher<PointCloud2>("output", 1);
  }

private:
  // Copy the organized-cloud pixels in [st_x, ed_x] x [st_y, ed_y] into a
  // new (smaller) organized cloud and publish it.
  void extractAndPublish(
    const PointCloud2::ConstSharedPtr & points, int st_x, int st_y, int ed_x, int ed_y)
  {
    const int wd = points->width;
    const int ht = points->height;
    const int rstep = points->row_step;
    const int pstep = points->point_step;

    st_x = std::max(st_x, 0);
    st_y = std::max(st_y, 0);
    ed_x = std::min(ed_x, wd - 1);
    ed_y = std::min(ed_y, ht - 1);
    if (ed_x < st_x || ed_y < st_y) {return;}

    PointCloud2 pt;
    pt.header = points->header;
    pt.width = ed_x - st_x + 1;
    pt.height = ed_y - st_y + 1;
    pt.row_step = pt.width * pstep;
    pt.point_step = pstep;
    pt.is_bigendian = false;
    pt.fields = points->fields;
    pt.is_dense = false;
    pt.data.resize(static_cast<size_t>(pt.row_step) * pt.height);

    unsigned char * dst_ptr = pt.data.data();
    for (int idx_y = st_y; idx_y <= ed_y; ++idx_y) {
      for (int idx_x = st_x; idx_x <= ed_x; ++idx_x) {
        const unsigned char * src_ptr = &points->data[idx_y * rstep + idx_x * pstep];
        std::memcpy(dst_ptr, src_ptr, pstep);
        dst_ptr += pstep;
      }
    }
    pub_->publish(pt);
  }

  void callbackPoint(
    const PointCloud2::ConstSharedPtr & points, const PointStamped::ConstSharedPtr & pt)
  {
    const int x = static_cast<int>(pt->point.x);
    const int y = static_cast<int>(pt->point.y);
    extractAndPublish(
      points, x - crop_width_, y - crop_width_, x + crop_width_, y + crop_width_);
  }

  void callbackPolygon(
    const PointCloud2::ConstSharedPtr & points, const PolygonStamped::ConstSharedPtr & poly)
  {
    if (poly->polygon.points.size() > 1) {
      extractAndPublish(
        points,
        static_cast<int>(poly->polygon.points[0].x),
        static_cast<int>(poly->polygon.points[0].y),
        static_cast<int>(poly->polygon.points[1].x),
        static_cast<int>(poly->polygon.points[1].y));
    }
  }

  int queue_size_;
  int crop_width_;
  message_filters::Subscriber<PointCloud2> points_sub_;
  message_filters::Subscriber<PolygonStamped> rect_sub_;
  message_filters::Subscriber<PointStamped> point_sub_;
  std::shared_ptr<message_filters::Synchronizer<PolygonSyncPolicy>> sync_polygon_;
  std::shared_ptr<message_filters::Synchronizer<PointSyncPolicy>> sync_point_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
};

}  // namespace image_view2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<image_view2::PointsRectExtractor>());
  rclcpp::shutdown();
  return 0;
}
