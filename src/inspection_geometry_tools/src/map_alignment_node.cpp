#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace inspection_geometry_tools
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeHalfPi(double angle)
{
  // A line is unchanged by pi.  Return the correction needed to reach its
  // nearest horizontal or vertical representation.
  return std::remainder(angle, kPi / 2.0);
}
}  // namespace

class MapAlignmentNode : public rclcpp::Node
{
public:
  MapAlignmentNode()
  : Node("map_alignment_node")
  {
    alignment_method_ = declare_parameter<std::string>("alignment_method", "pca");
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    min_line_length_ = declare_parameter<double>("min_line_length", 0.30);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.10);
    map_padding_ = declare_parameter<double>("map_padding", 1.00);
    map_canvas_scale_ = declare_parameter<double>("map_canvas_scale", 2.00);
    // Nav2's costmaps and localization consume the original /map coordinates.
    // Rotating samples while retaining frame_id="map" would create a second,
    // incompatible coordinate system with the same frame name.  Keep the
    // geometry pipeline in the original navigation coordinates by default;
    // the feature-aligned 10 cm grid remains available as an RViz overlay.
    // A rotated map is only appropriate for offline visualization or when a
    // separate aligned frame and matching Nav2 map/TF are also supplied.
    rotate_occupancy_grid_ = declare_parameter<bool>("rotate_occupancy_grid", false);
    output_frame_ = declare_parameter<std::string>("frame_id", "map");

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      std::bind(&MapAlignmentNode::mapCallback, this, std::placeholders::_1));
    aligned_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/aligned_map", map_qos);
    aligned_display_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/aligned_map_display", map_qos);
    alignment_grid_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/alignment_grid", rclcpp::QoS(1).reliable().transient_local());
    theta_map_pub_ = create_publisher<std_msgs::msg::Float64>("/theta_map", 10);
    theta_correction_pub_ = create_publisher<std_msgs::msg::Float64>("/theta_correction", 10);
  }

private:
  using Grid = nav_msgs::msg::OccupancyGrid;

  bool occupied(const Grid & map, int x, int y) const
  {
    if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) ||
      y >= static_cast<int>(map.info.height))
    {
      return false;
    }
    return map.data[static_cast<size_t>(y) * map.info.width + x] >= occupied_threshold_;
  }

  std::vector<std::pair<int, int>> boundaryCells(const Grid & map) const
  {
    std::vector<std::pair<int, int>> result;
    static constexpr int dx[] = {1, -1, 0, 0};
    static constexpr int dy[] = {0, 0, 1, -1};
    for (int y = 0; y < static_cast<int>(map.info.height); ++y) {
      for (int x = 0; x < static_cast<int>(map.info.width); ++x) {
        if (!occupied(map, x, y)) {
          continue;
        }
        for (int i = 0; i < 4; ++i) {
          if (!occupied(map, x + dx[i], y + dy[i])) {
            result.emplace_back(x, y);
            break;
          }
        }
      }
    }
    return result;
  }

  double estimateMainDirection(const Grid & map, const std::vector<std::pair<int, int>> & boundary) const
  {
    if (boundary.size() < 3U) {
      return 0.0;
    }

    // Each boundary point contributes the principal component of a local
    // neighbourhood.  A building axis is unchanged by 90 degrees: horizontal
    // and vertical walls belong to one orthogonal coordinate frame. Therefore
    // vote in 4*theta space (not 2*theta), then recover theta / 4.
    const int radius = std::max(2, static_cast<int>(std::ceil(
          min_line_length_ / std::max(1e-3F, map.info.resolution) / 2.0)));
    std::vector<uint8_t> is_boundary(static_cast<size_t>(map.info.width) * map.info.height, 0U);
    for (const auto & cell : boundary) {
      is_boundary[static_cast<size_t>(cell.second) * map.info.width + cell.first] = 1U;
    }

    double sum_cos = 0.0;
    double sum_sin = 0.0;
    for (const auto & center : boundary) {
      std::vector<std::pair<double, double>> neighbours;
      for (int y = std::max(0, center.second - radius);
        y <= std::min(static_cast<int>(map.info.height) - 1, center.second + radius); ++y)
      {
        for (int x = std::max(0, center.first - radius);
          x <= std::min(static_cast<int>(map.info.width) - 1, center.first + radius); ++x)
        {
          if (is_boundary[static_cast<size_t>(y) * map.info.width + x] != 0U) {
            neighbours.emplace_back(x, y);
          }
        }
      }
      if (neighbours.size() < 3U) {
        continue;
      }

      double mean_x = 0.0;
      double mean_y = 0.0;
      for (const auto & point : neighbours) {
        mean_x += point.first;
        mean_y += point.second;
      }
      mean_x /= neighbours.size();
      mean_y /= neighbours.size();
      double cxx = 0.0, cyy = 0.0, cxy = 0.0;
      for (const auto & point : neighbours) {
        const double x = point.first - mean_x;
        const double y = point.second - mean_y;
        cxx += x * x;
        cyy += y * y;
        cxy += x * y;
      }
      const double trace = cxx + cyy;
      const double discriminant = std::hypot(cxx - cyy, 2.0 * cxy);
      if (trace <= 1e-9 || discriminant / trace < 0.20) {
        continue;  // Corner/blob: not a reliable local line direction.
      }
      const double angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
      const double weight = discriminant;
      sum_cos += weight * std::cos(4.0 * angle);
      sum_sin += weight * std::sin(4.0 * angle);
    }

    if (std::hypot(sum_cos, sum_sin) < 1e-9) {
      RCLCPP_WARN(get_logger(), "No reliable local line direction; using zero map rotation.");
      return 0.0;
    }
    return 0.25 * std::atan2(sum_sin, sum_cos);
  }

  Grid rotateGrid(
    const Grid & input, double correction,
    const std::vector<std::pair<int, int>> & boundary) const
  {
    Grid output;
    output.header = input.header;
    output.header.frame_id = output_frame_;
    output.info.resolution = input.info.resolution;
    output.info.origin.orientation.w = 1.0;

    const double resolution = input.info.resolution;
    const double x0 = input.info.origin.position.x;
    const double y0 = input.info.origin.position.y;
    const double x1 = x0 + input.info.width * resolution;
    const double y1 = y0 + input.info.height * resolution;
    const double cx = (x0 + x1) / 2.0;
    const double cy = (y0 + y1) / 2.0;
    const double c = std::cos(correction);
    const double s = std::sin(correction);
    auto rotate = [cx, cy, c, s](double x, double y) {
        const double dx = x - cx;
        const double dy = y - cy;
        return std::pair<double, double>{cx + c * dx - s * dy, cy + s * dx + c * dy};
      };

    // Do not retain the complete source canvas. It is only a mapping work
    // area and is often much larger than the actual building (which is why it
    // becomes an awkward white diamond after rotation). Fit the new white map
    // extents to real obstacle features plus an explicit safety margin.
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    if (!boundary.empty()) {
      for (const auto & cell : boundary) {
        const auto point = rotate(
          x0 + (cell.first + 0.5) * resolution,
          y0 + (cell.second + 0.5) * resolution);
        min_x = std::min(min_x, point.first);
        min_y = std::min(min_y, point.second);
        max_x = std::max(max_x, point.first);
        max_y = std::max(max_y, point.second);
      }
      const double padding = std::max(0.0, map_padding_);
      min_x -= padding;
      min_y -= padding;
      max_x += padding;
      max_y += padding;
    } else {
      for (const auto & corner : {rotate(x0, y0), rotate(x0, y1), rotate(x1, y0), rotate(x1, y1)}) {
        min_x = std::min(min_x, corner.first);
        min_y = std::min(min_y, corner.second);
        max_x = std::max(max_x, corner.first);
        max_y = std::max(max_y, corner.second);
      }
    }
    // Align output bounds to cell edges; this prevents one-cell clipping due
    // to floating point round-off at a feature boundary.
    min_x = std::floor(min_x / resolution) * resolution;
    min_y = std::floor(min_y / resolution) * resolution;
    max_x = std::ceil(max_x / resolution) * resolution;
    max_y = std::ceil(max_y / resolution) * resolution;
    // Expand around the aligned feature-map centre without scaling walls,
    // occupancy cells, or the 10 cm grid.  Outside source coverage remains
    // unknown (-1), rather than being incorrectly declared free space.
    const double canvas_scale = std::max(1.0, map_canvas_scale_);
    const double center_x = (min_x + max_x) / 2.0;
    const double center_y = (min_y + max_y) / 2.0;
    const double half_width = (max_x - min_x) * canvas_scale / 2.0;
    const double half_height = (max_y - min_y) * canvas_scale / 2.0;
    min_x = std::floor((center_x - half_width) / resolution) * resolution;
    min_y = std::floor((center_y - half_height) / resolution) * resolution;
    max_x = std::ceil((center_x + half_width) / resolution) * resolution;
    max_y = std::ceil((center_y + half_height) / resolution) * resolution;
    output.info.width = static_cast<uint32_t>(std::ceil((max_x - min_x) / resolution));
    output.info.height = static_cast<uint32_t>(std::ceil((max_y - min_y) / resolution));
    output.info.origin.position.x = min_x;
    output.info.origin.position.y = min_y;
    output.info.origin.position.z = input.info.origin.position.z;
    output.data.assign(static_cast<size_t>(output.info.width) * output.info.height, -1);

    // Inverse-map output cell centres into the source grid. Nearest-neighbour
    // assignment deliberately avoids interpolation halos and thickened walls.
    for (uint32_t y = 0; y < output.info.height; ++y) {
      for (uint32_t x = 0; x < output.info.width; ++x) {
        const double wx = min_x + (static_cast<double>(x) + 0.5) * resolution;
        const double wy = min_y + (static_cast<double>(y) + 0.5) * resolution;
        const double dx = wx - cx;
        const double dy = wy - cy;
        const double sx = cx + c * dx + s * dy;  // inverse rotation R(-theta)
        const double sy = cy - s * dx + c * dy;
        const int source_x = static_cast<int>(std::floor((sx - x0) / resolution));
        const int source_y = static_cast<int>(std::floor((sy - y0) / resolution));
        if (source_x >= 0 && source_y >= 0 && source_x < static_cast<int>(input.info.width) &&
          source_y < static_cast<int>(input.info.height))
        {
          output.data[static_cast<size_t>(y) * output.info.width + x] =
            input.data[static_cast<size_t>(source_y) * input.info.width + source_x];
        }
      }
    }
    return output;
  }

  Grid makeDisplayMap(const Grid & safety_map) const
  {
    // This is deliberately a display-only companion topic. RViz renders free
    // cells white, so filling unknown canvas cells makes the requested aligned
    // rectangular white canvas visible. `/aligned_map` is left untouched for
    // snapping and navigation safety: unknown remains unknown there.
    Grid display_map = safety_map;
    for (auto & cell : display_map.data) {
      if (cell < 0) {
        cell = 0;
      }
    }
    return display_map;
  }

  geometry_msgs::msg::Point fromAlignedCoordinates(double u, double v, double theta) const
  {
    // u is along the detected main wall direction, v is its normal direction.
    geometry_msgs::msg::Point point;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    point.x = c * u - s * v;
    point.y = s * u + c * v;
    point.z = 0.03;
    return point;
  }

  void publishAlignmentGrid(
    const Grid & map, const std::vector<std::pair<int, int>> & boundary, double theta)
  {
    if (grid_resolution_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "grid_resolution must be positive; alignment grid is disabled.");
      return;
    }
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    auto to_aligned = [c, s](double x, double y) {
        return std::pair<double, double>{c * x + s * y, -s * x + c * y};
      };
    const double x0 = map.info.origin.position.x;
    const double y0 = map.info.origin.position.y;
    const double x1 = x0 + map.info.width * map.info.resolution;
    const double y1 = y0 + map.info.height * map.info.resolution;
    double min_u = std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const auto & corner : {to_aligned(x0, y0), to_aligned(x0, y1),
      to_aligned(x1, y0), to_aligned(x1, y1)})
    {
      min_u = std::min(min_u, corner.first);
      min_v = std::min(min_v, corner.second);
      max_u = std::max(max_u, corner.first);
      max_v = std::max(max_v, corner.second);
    }

    // Anchor a gridline to a real obstacle-boundary feature. This makes the
    // grid phase repeatable from the map rather than arbitrary world origin.
    double anchor_u = min_u;
    double anchor_v = min_v;
    if (!boundary.empty()) {
      anchor_u = std::numeric_limits<double>::infinity();
      anchor_v = std::numeric_limits<double>::infinity();
      for (const auto & cell : boundary) {
        const double x = x0 + (cell.first + 0.5) * map.info.resolution;
        const double y = y0 + (cell.second + 0.5) * map.info.resolution;
        const auto aligned = to_aligned(x, y);
        anchor_u = std::min(anchor_u, aligned.first);
        anchor_v = std::min(anchor_v, aligned.second);
      }
    }

    const int first_u = static_cast<int>(std::ceil((min_u - anchor_u) / grid_resolution_));
    const int last_u = static_cast<int>(std::floor((max_u - anchor_u) / grid_resolution_));
    const int first_v = static_cast<int>(std::ceil((min_v - anchor_v) / grid_resolution_));
    const int last_v = static_cast<int>(std::floor((max_v - anchor_v) / grid_resolution_));
    constexpr int kMaxLines = 4000;
    if ((last_u - first_u + 1) + (last_v - first_v + 1) > kMaxLines) {
      RCLCPP_WARN(get_logger(), "Alignment grid would exceed %d lines; increase grid_resolution.", kMaxLines);
      return;
    }

    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);
    auto new_marker = [this](const std::string & name, int id, double width,
        float r, float g, float b, float alpha) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = output_frame_;
        marker.header.stamp = now();
        marker.ns = name;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = width;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = alpha;
        return marker;
      };
    auto fine = new_marker("alignment_grid_fine", 0, 0.008, 0.20F, 0.75F, 1.00F, 0.45F);
    auto major = new_marker("alignment_grid_major", 0, 0.018, 0.10F, 0.95F, 0.95F, 0.85F);
    auto is_major = [](int index) { return ((index % 10) + 10) % 10 == 0; };
    for (int index = first_u; index <= last_u; ++index) {
      auto & marker = is_major(index) ? major : fine;
      const double u = anchor_u + index * grid_resolution_;
      marker.points.push_back(fromAlignedCoordinates(u, min_v, theta));
      marker.points.push_back(fromAlignedCoordinates(u, max_v, theta));
    }
    for (int index = first_v; index <= last_v; ++index) {
      auto & marker = is_major(index) ? major : fine;
      const double v = anchor_v + index * grid_resolution_;
      marker.points.push_back(fromAlignedCoordinates(min_u, v, theta));
      marker.points.push_back(fromAlignedCoordinates(max_u, v, theta));
    }
    output.markers.push_back(fine);
    output.markers.push_back(major);
    alignment_grid_pub_->publish(output);
  }

  void mapCallback(const Grid::SharedPtr input)
  {
    if (input->info.width == 0U || input->info.height == 0U || input->info.resolution <= 0.0F ||
      input->data.size() != static_cast<size_t>(input->info.width) * input->info.height)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid OccupancyGrid.");
      return;
    }
    if (alignment_method_ != "pca") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "alignment_method '%s' is not implemented; using local PCA.", alignment_method_.c_str());
    }
    const auto boundary = boundaryCells(*input);
    const double theta_map = estimateMainDirection(*input, boundary);
    const double theta_correction = -normalizeHalfPi(theta_map);

    std_msgs::msg::Float64 angle_msg;
    angle_msg.data = theta_map;
    theta_map_pub_->publish(angle_msg);
    angle_msg.data = theta_correction;
    theta_correction_pub_->publish(angle_msg);
    if (rotate_occupancy_grid_) {
      const Grid aligned = rotateGrid(*input, theta_correction, boundary);
      aligned_map_pub_->publish(aligned);
      aligned_display_map_pub_->publish(makeDisplayMap(aligned));
      // In the aligned map frame feature axes are X/Y, so the 10 cm grid and
      // the cropped white OccupancyGrid canvas share exactly the same axes.
      publishAlignmentGrid(aligned, boundaryCells(aligned), 0.0);
    } else {
      Grid passthrough = *input;
      passthrough.header.frame_id = output_frame_;
      aligned_map_pub_->publish(passthrough);
      aligned_display_map_pub_->publish(makeDisplayMap(passthrough));
      publishAlignmentGrid(*input, boundary, theta_map);
    }
    RCLCPP_INFO(
      get_logger(),
      "Detected %zu boundary cells: theta_map=%.5f rad, correction=%.5f rad, grid=%.3f m",
      boundary.size(), theta_map, theta_correction, grid_resolution_);
  }

  std::string alignment_method_;
  int occupied_threshold_;
  double min_line_length_;
  double grid_resolution_;
  double map_padding_;
  double map_canvas_scale_;
  bool rotate_occupancy_grid_;
  std::string output_frame_;
  rclcpp::Subscription<Grid>::SharedPtr map_sub_;
  rclcpp::Publisher<Grid>::SharedPtr aligned_map_pub_;
  rclcpp::Publisher<Grid>::SharedPtr aligned_display_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr alignment_grid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_correction_pub_;
};
}  // namespace inspection_geometry_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_geometry_tools::MapAlignmentNode>());
  rclcpp::shutdown();
  return 0;
}
