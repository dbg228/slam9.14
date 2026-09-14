#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace bg = boost::geometry;

namespace inspection_geometry_tools
{
class PolygonOffsetNode : public rclcpp::Node
{
public:
  PolygonOffsetNode()
  : Node("polygon_offset_node")
  {
    offset_distance_ = declare_parameter<double>("offset_distance", 1.0);
    path_spacing_ = declare_parameter<double>("path_spacing", 0.10);
    direction_marker_spacing_ = declare_parameter<double>("direction_marker_spacing", 1.0);
    corner_points_ = declare_parameter<int>("corner_points", 16);
    reverse_path_ = declare_parameter<bool>("reverse_path", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    polygon_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/inspection_polygon", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PolygonOffsetNode::polygonCallback, this, std::placeholders::_1));
    offset_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/inspection_offset_polygon", rclcpp::QoS(1).reliable().transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/inspection_path", rclcpp::QoS(1).reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/inspection_path_markers", rclcpp::QoS(1).reliable().transient_local());
  }

private:
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  using MultiPolygon = bg::model::multi_polygon<Polygon>;

  static double distance(const Point & a, const Point & b)
  {
    return std::hypot(bg::get<0>(b) - bg::get<0>(a), bg::get<1>(b) - bg::get<1>(a));
  }

  static geometry_msgs::msg::Point toRosPoint(const Point & point)
  {
    geometry_msgs::msg::Point output;
    output.x = bg::get<0>(point);
    output.y = bg::get<1>(point);
    output.z = 0.05;
    return output;
  }

  Polygon inputPolygon(const geometry_msgs::msg::PolygonStamped & message) const
  {
    Polygon polygon;
    for (const auto & point : message.polygon.points) {
      bg::append(polygon.outer(), Point(point.x, point.y));
    }
    bg::correct(polygon);
    return polygon;
  }

  const Polygon * largestPolygon(const MultiPolygon & polygons) const
  {
    const Polygon * result = nullptr;
    double largest_area = -1.0;
    for (const auto & polygon : polygons) {
      const double area = std::abs(bg::area(polygon));
      if (area > largest_area) {
        largest_area = area;
        result = &polygon;
      }
    }
    return result;
  }

  std::vector<Point> resampleOuterRing(const Polygon & polygon) const
  {
    std::vector<Point> vertices(polygon.outer().begin(), polygon.outer().end());
    if (vertices.size() > 1U && bg::equals(vertices.front(), vertices.back())) {
      vertices.pop_back();
    }
    if (vertices.size() < 3U) {
      return {};
    }
    if (reverse_path_) {
      std::reverse(vertices.begin(), vertices.end());
    }
    const double spacing = std::max(0.02, path_spacing_);
    std::vector<Point> path;
    for (size_t i = 0; i < vertices.size(); ++i) {
      const Point & start = vertices[i];
      const Point & end = vertices[(i + 1U) % vertices.size()];
      const double length = distance(start, end);
      const int samples = std::max(1, static_cast<int>(std::ceil(length / spacing)));
      for (int sample = 0; sample < samples; ++sample) {
        const double t = static_cast<double>(sample) / samples;
        path.emplace_back(
          bg::get<0>(start) + t * (bg::get<0>(end) - bg::get<0>(start)),
          bg::get<1>(start) + t * (bg::get<1>(end) - bg::get<1>(start)));
      }
    }
    return path;
  }

  void publishOffsetPolygon(const Polygon & polygon, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PolygonStamped message;
    message.header.frame_id = frame_id_;
    message.header.stamp = stamp;
    for (const auto & point : polygon.outer()) {
      geometry_msgs::msg::Point32 output;
      output.x = static_cast<float>(bg::get<0>(point));
      output.y = static_cast<float>(bg::get<1>(point));
      output.z = 0.0F;
      message.polygon.points.push_back(output);
    }
    if (message.polygon.points.size() > 1U) {
      message.polygon.points.pop_back();  // PolygonStamped implies closure.
    }
    offset_polygon_pub_->publish(message);
  }

  void publishPath(const std::vector<Point> & path, const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = frame_id_;
    message.header.stamp = stamp;
    for (size_t i = 0; i < path.size(); ++i) {
      const Point & point = path[i];
      const Point & next = path[(i + 1U) % path.size()];
      const double yaw = std::atan2(
        bg::get<1>(next) - bg::get<1>(point), bg::get<0>(next) - bg::get<0>(point));
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position = toRosPoint(point);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      message.poses.push_back(pose);
    }
    path_pub_->publish(message);
  }

  void publishMarkers(const std::vector<Point> & path, const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = stamp;
    line.ns = "inspection_offset_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.07;
    line.color.r = 0.95F;
    line.color.g = 0.15F;
    line.color.b = 0.80F;
    line.color.a = 1.0F;
    for (const auto & point : path) {
      line.points.push_back(toRosPoint(point));
    }
    if (!line.points.empty()) {
      line.points.push_back(line.points.front());
    }
    markers.markers.push_back(line);

    // The Path itself carries a tangent yaw at every pose.  Draw sparse arrows
    // so operators can see that ordering directly in RViz before FollowPath is
    // sent to MPPI.
    const size_t arrow_step = std::max<size_t>(1U, static_cast<size_t>(std::round(
          std::max(0.10, direction_marker_spacing_) / std::max(0.02, path_spacing_))));
    int arrow_id = 0;
    for (size_t i = 0; i < path.size(); i += arrow_step) {
      const Point & point = path[i];
      const Point & next = path[(i + 1U) % path.size()];
      const double yaw = std::atan2(
        bg::get<1>(next) - bg::get<1>(point), bg::get<0>(next) - bg::get<0>(point));
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = stamp;
      arrow.ns = "inspection_offset_direction";
      arrow.id = arrow_id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.position = toRosPoint(point);
      arrow.pose.position.z = 0.14;
      arrow.pose.orientation.z = std::sin(yaw / 2.0);
      arrow.pose.orientation.w = std::cos(yaw / 2.0);
      arrow.scale.x = 0.45;
      arrow.scale.y = 0.12;
      arrow.scale.z = 0.12;
      arrow.color.r = 0.90F;
      arrow.color.g = 0.05F;
      arrow.color.b = 0.75F;
      arrow.color.a = 1.0F;
      markers.markers.push_back(arrow);
    }
    marker_pub_->publish(markers);
  }

  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr message)
  {
    if (message->polygon.points.size() < 3U) {
      RCLCPP_WARN(get_logger(), "Ignoring inspection polygon with fewer than three vertices.");
      return;
    }
    if (!message->header.frame_id.empty() && message->header.frame_id != frame_id_) {
      RCLCPP_WARN(get_logger(), "Ignoring polygon in frame '%s'; expected '%s'.",
        message->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    if (offset_distance_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "offset_distance must be positive for outward buffer.");
      return;
    }

    const Polygon source = inputPolygon(*message);
    if (!bg::is_valid(source)) {
      RCLCPP_WARN(get_logger(), "Inspection polygon is self-intersecting or invalid; buffer was not generated.");
      return;
    }
    MultiPolygon buffered;
    const bg::strategy::buffer::distance_symmetric<double> distance_strategy(offset_distance_);
    const bg::strategy::buffer::side_straight side_strategy;
    const bg::strategy::buffer::join_round join_strategy(std::max(4, corner_points_));
    const bg::strategy::buffer::end_round end_strategy(std::max(4, corner_points_));
    const bg::strategy::buffer::point_circle circle_strategy(std::max(4, corner_points_));
    bg::buffer(source, buffered, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
    const Polygon * offset = largestPolygon(buffered);
    if (offset == nullptr || offset->outer().size() < 4U) {
      RCLCPP_WARN(get_logger(), "Polygon buffer produced no usable outer ring.");
      return;
    }
    const auto path = resampleOuterRing(*offset);
    if (path.size() < 3U) {
      RCLCPP_WARN(get_logger(), "Offset path contains too few samples.");
      return;
    }
    const auto stamp = now();
    publishOffsetPolygon(*offset, stamp);
    publishPath(path, stamp);
    publishMarkers(path, stamp);
    RCLCPP_INFO(get_logger(), "Published %.2f m outward offset with %zu path poses.",
      offset_distance_, path.size());
  }

  double offset_distance_;
  double path_spacing_;
  double direction_marker_spacing_;
  int corner_points_;
  bool reverse_path_;
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr offset_polygon_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
}  // namespace inspection_geometry_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_geometry_tools::PolygonOffsetNode>());
  rclcpp::shutdown();
  return 0;
}
