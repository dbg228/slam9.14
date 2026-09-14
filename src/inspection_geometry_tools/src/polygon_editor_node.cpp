#include <memory>
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace inspection_geometry_tools
{
class PolygonEditorNode : public rclcpp::Node
{
public:
  PolygonEditorNode()
  : Node("polygon_editor_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    close_radius_ = declare_parameter<double>("close_radius", 0.25);
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/snapped_feature_point", 10,
      std::bind(&PolygonEditorNode::pointCallback, this, std::placeholders::_1));
    polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "/inspection_polygon", rclcpp::QoS(1).reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/inspection_markers", rclcpp::QoS(1).reliable().transient_local());
    close_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_polygon/close",
      std::bind(&PolygonEditorNode::close, this, std::placeholders::_1, std::placeholders::_2));
    undo_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_polygon/undo",
      std::bind(&PolygonEditorNode::undo, this, std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_polygon/reset",
      std::bind(&PolygonEditorNode::reset, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point)
  {
    if (!point->header.frame_id.empty() && point->header.frame_id != frame_id_) {
      RCLCPP_WARN(get_logger(), "Ignoring feature point in frame '%s'; expected '%s'.",
        point->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    if (closed_) {
      RCLCPP_INFO(get_logger(), "Starting a new polygon after a closed polygon.");
      points_.clear();
      closed_ = false;
    }
    if (points_.size() >= 3U && nearFirstVertex(point->point)) {
      closed_ = true;
      publishPolygon();
      publishMarkers();
      RCLCPP_INFO(get_logger(), "Polygon auto-closed by clicking near P1.");
      return;
    }
    points_.push_back(point->point);
    publishMarkers();
  }

  bool nearFirstVertex(const geometry_msgs::msg::Point & point) const
  {
    const auto & first = points_.front();
    return std::hypot(point.x - first.x, point.y - first.y) <= close_radius_;
  }

  void close(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (points_.size() < 3U) {
      response->success = false;
      response->message = "At least three snapped points are required to close a polygon.";
      return;
    }
    closed_ = true;
    publishPolygon();
    publishMarkers();
    response->success = true;
    response->message = "Inspection polygon closed and published.";
  }

  void undo(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (points_.empty()) {
      response->success = false;
      response->message = "No point to undo.";
      return;
    }
    points_.pop_back();
    closed_ = false;
    publishMarkers();
    response->success = true;
    response->message = "Last point removed.";
  }

  void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    points_.clear();
    closed_ = false;
    publishMarkers();
    response->success = true;
    response->message = "Polygon editor reset.";
  }

  void publishPolygon()
  {
    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = frame_id_;
    polygon.header.stamp = now();
    polygon.polygon.points.reserve(points_.size());
    for (const auto & point : points_) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(point.x);
      p.y = static_cast<float>(point.y);
      p.z = static_cast<float>(point.z);
      polygon.polygon.points.push_back(p);
    }
    polygon_pub_->publish(polygon);
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    for (size_t i = 0; i < points_.size(); ++i) {
      visualization_msgs::msg::Marker point_marker;
      point_marker.header.frame_id = frame_id_;
      point_marker.header.stamp = now();
      point_marker.ns = "inspection_vertices";
      point_marker.id = static_cast<int>(i);
      point_marker.type = visualization_msgs::msg::Marker::SPHERE;
      point_marker.action = visualization_msgs::msg::Marker::ADD;
      point_marker.pose.position = points_[i];
      point_marker.pose.orientation.w = 1.0;
      point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = 0.12;
      point_marker.color.r = 0.1F;
      point_marker.color.g = 0.85F;
      point_marker.color.b = 1.0F;
      point_marker.color.a = 1.0F;
      array.markers.push_back(point_marker);

      visualization_msgs::msg::Marker label = point_marker;
      label.ns = "inspection_labels";
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.z += 0.18;
      label.scale.z = 0.14;
      label.color.r = label.color.g = label.color.b = 1.0F;
      label.text = "P" + std::to_string(i + 1U);
      array.markers.push_back(label);
    }

    if (points_.size() >= 2U) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = frame_id_;
      line.header.stamp = now();
      line.ns = "inspection_polygon_edges";
      line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.045;
      line.color.r = closed_ ? 0.1F : 1.0F;
      line.color.g = closed_ ? 1.0F : 0.65F;
      line.color.b = 0.1F;
      line.color.a = 1.0F;
      line.points = points_;
      if (closed_) {
        line.points.push_back(points_.front());
      }
      array.markers.push_back(line);
    }
    marker_pub_->publish(array);
  }

  std::string frame_id_;
  double close_radius_;
  bool closed_{false};
  std::vector<geometry_msgs::msg::Point> points_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};
}  // namespace inspection_geometry_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_geometry_tools::PolygonEditorNode>());
  rclcpp::shutdown();
  return 0;
}
