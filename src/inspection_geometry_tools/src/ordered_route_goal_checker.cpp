#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "inspection_geometry_tools/ordered_path_progress.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace inspection_geometry_tools
{
class OrderedRouteGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    nav2_controller::SimpleGoalChecker::initialize(parent, plugin_name, costmap);
    node_ = parent;
    auto node = parent.lock();
    costmap_frame_ = costmap->getGlobalFrameID();
    tf_ = costmap->getTfBuffer();
    const auto radius_name = plugin_name + ".route_progress_tolerance";
    if (!node->has_parameter(radius_name)) {node->declare_parameter(radius_name, 0.35);}
    radius_ = node->get_parameter(radius_name).as_double();
    if (!std::isfinite(radius_) || radius_ <= 0.0) {
      throw std::invalid_argument("route_progress_tolerance must be positive");
    }
    // Volatile: never accept a retained path from a previous action. The
    // executor republishes the immutable active path with a fresh header.
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "/inspection_route/active_reference_path", rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (progress_.ready() || rclcpp::Time(msg->header.stamp) < reset_stamp_ ||
          msg->header.frame_id.empty()) {return;}
        std::vector<RoutePoint> points;
        for (const auto & pose : msg->poses) {
          if (!pose.header.frame_id.empty() && pose.header.frame_id != msg->header.frame_id) {
            return;
          }
          points.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        try {progress_.setPath(points, radius_);} catch (const std::invalid_argument &) {return;}
        path_frame_ = msg->header.frame_id;
        endpoint_ = points.back();
      });
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nav2_controller::SimpleGoalChecker::reset();
    progress_.clear();
    reset_stamp_ = node_.lock()->get_clock()->now();
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query,
    const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!progress_.ready()) {return false;}
    auto map_query = query;
    auto map_goal = goal;
    if (path_frame_ != costmap_frame_) {
      try {
        const auto transform = tf_->lookupTransform(path_frame_, costmap_frame_, tf2::TimePointZero);
        tf2::doTransform(query, map_query, transform);
        tf2::doTransform(goal, map_goal, transform);
      } catch (const tf2::TransformException &) {return false;}
    }
    // A path for another goal must never release this goal checker.
    if (std::hypot(map_goal.position.x - endpoint_.x,
      map_goal.position.y - endpoint_.y) > 0.05) {return false;}
    if (!progress_.update(map_query.position.x, map_query.position.y)) {return false;}
    return nav2_controller::SimpleGoalChecker::isGoalReached(query, goal, velocity);
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::mutex mutex_;
  OrderedPathProgress progress_;
  RoutePoint endpoint_{};
  std::string costmap_frame_;
  std::string path_frame_;
  double radius_{0.35};
  rclcpp::Time reset_stamp_{0, 0, RCL_ROS_TIME};
};
}  // namespace inspection_geometry_tools

PLUGINLIB_EXPORT_CLASS(inspection_geometry_tools::OrderedRouteGoalChecker, nav2_core::GoalChecker)
