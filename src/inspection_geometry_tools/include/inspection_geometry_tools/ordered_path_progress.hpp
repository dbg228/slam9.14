#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace inspection_geometry_tools
{
struct RoutePoint {double x; double y;};

// Only consecutive route points may be consumed. A nearby END cannot skip
// START, the far side of a loop, or an unvisited crossing of the path.
class OrderedPathProgress
{
public:
  void setPath(const std::vector<RoutePoint> & points, double radius)
  {
    clear();
    if (!std::isfinite(radius) || radius <= 0.0) {
      throw std::invalid_argument("route progress radius must be positive");
    }
    std::vector<RoutePoint> checked;
    for (const auto & point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw std::invalid_argument("route contains a nonfinite point");
      }
      if (checked.empty() || std::hypot(point.x - checked.back().x,
          point.y - checked.back().y) > 1e-6)
      {
        checked.push_back(point);
      }
    }
    if (checked.size() < 2U) {
      throw std::invalid_argument("route needs two distinct points");
    }
    points_ = std::move(checked);
    radius_ = radius;
  }

  void clear() {points_.clear(); next_ = 0U;}

  bool update(double x, double y)
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {return false;}
    while (next_ < points_.size() &&
      std::hypot(x - points_[next_].x, y - points_[next_].y) <= radius_)
    {
      ++next_;
    }
    return !points_.empty() && next_ == points_.size();
  }

  bool ready() const {return !points_.empty();}
  size_t visited() const {return next_;}
  size_t size() const {return points_.size();}

private:
  std::vector<RoutePoint> points_;
  size_t next_{0U};
  double radius_{0.35};
};
}  // namespace inspection_geometry_tools
