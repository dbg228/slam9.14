#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace inspection_geometry_tools
{
class InspectionPathExecutorNode : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

  InspectionPathExecutorNode()
  : Node("inspection_path_executor_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    follow_path_action_ = declare_parameter<std::string>("follow_path_action", "/follow_path");
    // The approach and the purple segment are concatenated into ONE fixed
    // FollowPath goal.  In particular, START is not a controller goal: this
    // prevents MPPI from slowing down and searching for a pose at START.
    controller_id_ = declare_parameter<std::string>("controller_id", "InspectionStrict");
    // Nav2 Humble does not treat an empty FollowPath goal_checker_id as
    // "use the default": it tries to find a checker literally named "" and
    // aborts before publishing any velocity command.  Select it explicitly.
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "goal_checker");
    route_selection_file_ = declare_parameter<std::string>(
      "route_selection_file", "/home/zkam/test_nav/robot_data/inspection_route_selection.txt");

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/inspection_path", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&InspectionPathExecutorNode::pathCallback, this, std::placeholders::_1));
    start_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/inspection_route_start", 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr point) {
        selectionCallback(point, Selection::Start);
      });
    middle_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/inspection_route_middle", 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr point) {
        selectionCallback(point, Selection::Middle);
      });
    end_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/inspection_route_end", 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr point) {
        selectionCallback(point, Selection::End);
      });
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/inspection_route_markers", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/inspection_route/status", rclcpp::QoS(1).reliable().transient_local());
    follow_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/follow",
      std::bind(&InspectionPathExecutorNode::follow, this, std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/cancel",
      std::bind(&InspectionPathExecutorNode::cancel, this, std::placeholders::_1, std::placeholders::_2));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/save",
      std::bind(&InspectionPathExecutorNode::save, this, std::placeholders::_1, std::placeholders::_2));
    load_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/load",
      std::bind(&InspectionPathExecutorNode::load, this, std::placeholders::_1, std::placeholders::_2));
    action_client_ = rclcpp_action::create_client<FollowPath>(this, follow_path_action_);
    compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/compute_path_to_pose");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(250),
      std::bind(&InspectionPathExecutorNode::publishStatus, this));
    publishStatus();
  }

private:
  enum class Selection {Start, Middle, End};

  void pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    if (path->poses.size() < 3U) {
      RCLCPP_WARN(get_logger(), "Ignoring inspection path with fewer than three poses.");
      return;
    }
    path_ = *path;
    projectSavedSelections();
    publishMarkers();
  }

  std::optional<size_t> nearestPathIndex(const geometry_msgs::msg::Point & point) const
  {
    if (path_.poses.empty()) {
      return std::nullopt;
    }
    size_t best_index = 0U;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_.poses.size(); ++i) {
      const auto & candidate = path_.poses[i].pose.position;
      const double dx = candidate.x - point.x;
      const double dy = candidate.y - point.y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_index = i;
      }
    }
    return best_index;
  }

  void selectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr point, Selection selection)
  {
    if (!point->header.frame_id.empty() && point->header.frame_id != frame_id_) {
      RCLCPP_WARN(get_logger(), "Ignoring route selection in frame '%s'; expected '%s'.",
        point->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    clickFor(selection) = point->point;
    const auto index = nearestPathIndex(point->point);
    switch (selection) {
      case Selection::Start: start_index_ = index; break;
      case Selection::Middle: middle_index_ = index; break;
      case Selection::End: end_index_ = index; break;
    }
    if (!index.has_value()) {
      RCLCPP_WARN(get_logger(), "Saved %s click; it will attach automatically when /inspection_path becomes available.",
        selectionName(selection).c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s saved on inspection path at pose %zu.",
        selectionName(selection).c_str(), *index);
    }
    publishMarkers();
  }

  std::string selectionName(Selection selection) const
  {
    switch (selection) {
      case Selection::Start: return "START";
      case Selection::Middle: return "MIDDLE";
      case Selection::End: return "END";
    }
    return "UNKNOWN";
  }

  std::optional<geometry_msgs::msg::Point> & clickFor(Selection selection)
  {
    switch (selection) {
      case Selection::Start: return start_click_;
      case Selection::Middle: return middle_click_;
      case Selection::End: return end_click_;
    }
    return start_click_;
  }

  void projectSavedSelections()
  {
    start_index_ = start_click_.has_value() ? nearestPathIndex(*start_click_) : std::nullopt;
    middle_index_ = middle_click_.has_value() ? nearestPathIndex(*middle_click_) : std::nullopt;
    end_index_ = end_click_.has_value() ? nearestPathIndex(*end_click_) : std::nullopt;
  }

  void appendClosedSegment(nav_msgs::msg::Path & route, size_t from, size_t to) const
  {
    const size_t count = path_.poses.size();
    size_t index = from;
    while (true) {
      if (route.poses.empty() || index != from) {
        route.poses.push_back(path_.poses[index]);
      }
      if (index == to) {
        break;
      }
      index = (index + 1U) % count;
    }
  }

  std::optional<nav_msgs::msg::Path> selectedRoute() const
  {
    if (!start_index_.has_value() || !end_index_.has_value()) {
      return std::nullopt;
    }
    nav_msgs::msg::Path route;
    route.header = path_.header;
    appendClosedSegment(route, *start_index_, middle_index_.value_or(*end_index_));
    if (middle_index_.has_value()) {
      // The middle pose is already the last pose from the first segment.
      const size_t count = path_.poses.size();
      size_t index = (*middle_index_ + 1U) % count;
      while (true) {
        route.poses.push_back(path_.poses[index]);
        if (index == *end_index_) {
          break;
        }
        index = (index + 1U) % count;
      }
    }
    return route.poses.size() >= 2U ? std::optional<nav_msgs::msg::Path>(route) : std::nullopt;
  }

  std::optional<geometry_msgs::msg::Point> selectedPoint(
    const std::optional<size_t> & index, const std::optional<geometry_msgs::msg::Point> & click) const
  {
    if (index.has_value() && *index < path_.poses.size()) {
      return path_.poses[*index].pose.position;
    }
    return click;
  }

  static double distance2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static double pathLength(const nav_msgs::msg::Path & path)
  {
    double result = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      result += distance2d(path.poses[i - 1].pose.position, path.poses[i].pose.position);
    }
    return result;
  }

  static void setPathTangentOrientations(nav_msgs::msg::Path & path)
  {
    if (path.poses.size() < 2U) {
      return;
    }
    // ComputePathToPose supplies reliable positions, but planners do not all
    // populate each intermediate pose orientation.  InspectionStrict uses
    // path orientations, so leaving those poses at the identity quaternion
    // makes a DiffDrive robot turn while trying to follow a perfectly valid
    // positional path.  Derive every heading from the next path segment.
    for (size_t index = 0; index + 1U < path.poses.size(); ++index) {
      const auto & current = path.poses[index].pose.position;
      const auto & next = path.poses[index + 1U].pose.position;
      const double dx = next.x - current.x;
      const double dy = next.y - current.y;
      if (std::hypot(dx, dy) < 1e-5) {
        continue;
      }
      const double half_yaw = 0.5 * std::atan2(dy, dx);
      auto & orientation = path.poses[index].pose.orientation;
      orientation.x = 0.0;
      orientation.y = 0.0;
      orientation.z = std::sin(half_yaw);
      orientation.w = std::cos(half_yaw);
    }
    // This is an open execution path, not a closed polygon.  END must retain
    // the incoming tangent (last segment), rather than the closed-loop
    // direction that the original purple Path stored for its final sample.
    path.poses.back().pose.orientation = path.poses[path.poses.size() - 2U].pose.orientation;
  }

  bool writeSelection(const char * name, const std::optional<geometry_msgs::msg::Point> & point,
    std::ofstream & output) const
  {
    if (!point.has_value()) {
      return false;
    }
    output << name << " " << std::setprecision(17) << point->x << " " << point->y << " " << point->z << "\n";
    return true;
  }

  void save(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto start = selectedPoint(start_index_, start_click_);
    const auto middle = selectedPoint(middle_index_, middle_click_);
    const auto end = selectedPoint(end_index_, end_click_);
    if (!start.has_value() || !end.has_value()) {
      response->success = false;
      response->message = "Select and attach both START and END before saving.";
      return;
    }
    std::ofstream output(route_selection_file_, std::ios::trunc);
    if (!output) {
      response->success = false;
      response->message = "Cannot write route selection file: " + route_selection_file_;
      return;
    }
    output << "frame_id " << frame_id_ << "\n";
    writeSelection("start", start, output);
    writeSelection("middle", middle, output);
    writeSelection("end", end, output);
    output.close();
    response->success = true;
    response->message = "START/END saved to " + route_selection_file_;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    publishStatus();
  }

  void load(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::ifstream input(route_selection_file_);
    if (!input) {
      response->success = false;
      response->message = "No saved route selection file: " + route_selection_file_;
      return;
    }
    std::string key;
    std::string saved_frame;
    std::optional<geometry_msgs::msg::Point> loaded_start;
    std::optional<geometry_msgs::msg::Point> loaded_middle;
    std::optional<geometry_msgs::msg::Point> loaded_end;
    while (input >> key) {
      if (key == "frame_id") {
        input >> saved_frame;
        continue;
      }
      geometry_msgs::msg::Point point;
      if (!(input >> point.x >> point.y >> point.z)) {
        response->success = false;
        response->message = "Saved route selection file is invalid.";
        return;
      }
      if (key == "start") {loaded_start = point;}
      if (key == "middle") {loaded_middle = point;}
      if (key == "end") {loaded_end = point;}
    }
    if (saved_frame != frame_id_ || !loaded_start.has_value() || !loaded_end.has_value()) {
      response->success = false;
      response->message = "Saved route is incomplete or belongs to a different frame.";
      return;
    }
    start_click_ = loaded_start;
    middle_click_ = loaded_middle;
    end_click_ = loaded_end;
    projectSavedSelections();
    publishMarkers();
    response->success = true;
    response->message = "Saved START/END loaded and attached to the current purple path.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    publishStatus();
  }

  void publishStatus()
  {
    const auto start = selectedPoint(start_index_, start_click_);
    const auto end = selectedPoint(end_index_, end_click_);
    const auto route = selectedRoute();
    std::ostringstream text;
    text << std::fixed << std::setprecision(2);
    text << "START: " << (start_index_.has_value() ? "已选" : "未选")
         << " | END: " << (end_index_.has_value() ? "已选" : "未选");
    if (route.has_value()) {
      text << "\n紫线路程 START→END: " << pathLength(*route) << " m";
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(frame_id_, "base_link", tf2::TimePointZero);
      geometry_msgs::msg::Point robot;
      robot.x = transform.transform.translation.x;
      robot.y = transform.transform.translation.y;
      if (start.has_value()) {
        text << "\n到 START（直线）: " << distance2d(robot, *start) << " m";
      }
      if (end.has_value() && !active_goal_) {
        text << "\nSTART 后到 END（紫线）: " << (route.has_value() ? pathLength(*route) : 0.0) << " m";
      }
    } catch (const tf2::TransformException &) {
      text << "\n等待 map → base_link 定位 TF";
    }
    if (approach_planning_) {
      text << "\n正在一次性计算固定全程路径…";
    } else if (active_goal_) {
      text << "\n固定路径跟踪中，距 END（Nav2）: " << follow_distance_remaining_ << " m";
    } else if (start.has_value() && end.has_value()) {
      text << "\n可保存后点击“开始路线”。";
    }
    std_msgs::msg::String status;
    status.data = text.str();
    status_pub_->publish(status);
  }

  void sendFollowPath(const nav_msgs::msg::Path & route)
  {
    FollowPath::Goal goal;
    goal.path = route;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;
    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandleFollowPath::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Nav2 rejected the inspection FollowPath goal.");
          return;
        }
        active_goal_ = goal_handle;
        RCLCPP_INFO(get_logger(), "Nav2 accepted %zu-pose inspection route.", selected_route_size_);
      };
    options.feedback_callback = [this](GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback) {
        follow_distance_remaining_ = feedback->distance_to_goal;
      };
    options.result_callback = [this](const GoalHandleFollowPath::WrappedResult & result) {
        active_goal_.reset();
        follow_distance_remaining_ = 0.0;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Inspection FollowPath route completed.");
        } else {
          RCLCPP_WARN(get_logger(), "Inspection FollowPath finished with action result code %d.",
            static_cast<int>(result.code));
        }
    };
    selected_route_size_ = route.poses.size();
    action_client_->async_send_goal(goal, options);
  }

  // Concatenate the one-shot global path and the selected purple segment,
  // then give the controller one immutable Path.  START is therefore only a
  // waypoint inside the path, never a terminal goal with goal tolerance or a
  // terminal-yaw search.
  void sendFixedCombinedPath(nav_msgs::msg::Path approach_path)
  {
    if (!pending_route_.has_value() || approach_path.poses.size() < 2U) {
      pending_route_.reset();
      RCLCPP_ERROR(get_logger(), "Static global path is empty; inspection route was not started.");
      return;
    }

    // The global planner finishes at the START position.  Append the
    // remaining purple poses, then derive the tangents over the *whole open
    // execution path* (including END).
    // Do not append the first pose twice: it is already the final pose of the
    // global path and duplicate consecutive poses can make a path angle ill
    // defined for a differential-drive controller.
    for (size_t index = 1U; index < pending_route_->poses.size(); ++index) {
      approach_path.poses.push_back(pending_route_->poses[index]);
    }
    setPathTangentOrientations(approach_path);
    pending_route_.reset();
    RCLCPP_INFO(get_logger(),
      "Following one fixed combined path (%zu poses): global path joins purple path at START; no replanning, stop, or START goal search.",
      approach_path.poses.size());
    sendFollowPath(approach_path);
  }

  void follow(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto route = selectedRoute();
    if (!route.has_value()) {
      response->success = false;
      response->message = "Select both /inspection_route_start and /inspection_route_end on /inspection_path first.";
      return;
    }
    if (active_goal_ || approach_planning_) {
      response->success = false;
      response->message = "An inspection static approach or FollowPath goal is already active.";
      return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      response->success = false;
      response->message = "Nav2 FollowPath action server is unavailable.";
      return;
    }
    if (!compute_path_client_->wait_for_action_server(std::chrono::seconds(2))) {
      response->success = false;
      response->message = "Nav2 ComputePathToPose action server is unavailable.";
      return;
    }

    pending_route_ = *route;
    approach_planning_ = true;
    ComputePathToPose::Goal plan_goal;
    plan_goal.goal = pending_route_->poses.front();
    plan_goal.use_start = false;  // use current robot pose exactly once
    rclcpp_action::Client<ComputePathToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<ComputePathToPose>::SharedPtr goal_handle) {
        if (!goal_handle) {
          approach_planning_ = false;
          pending_route_.reset();
          RCLCPP_ERROR(get_logger(), "Nav2 rejected the one-shot static START planning request.");
          return;
        }
        RCLCPP_INFO(get_logger(), "Computing one static path to START (no replanning while moving).");
      };
    options.result_callback = [this](const rclcpp_action::ClientGoalHandle<ComputePathToPose>::WrappedResult & result) {
        approach_planning_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !pending_route_.has_value()) {
          RCLCPP_WARN(get_logger(), "Static planning to START failed; inspection FollowPath was not started.");
          pending_route_.reset();
          return;
        }
        sendFixedCombinedPath(result.result->path);
      };
    compute_path_client_->async_send_goal(plan_goal, options);
    response->success = true;
    response->message = "Computing one fixed global-plus-purple path; START is a waypoint, not a stop goal.";
  }

  void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (approach_planning_) {
      compute_path_client_->async_cancel_all_goals();
      approach_planning_ = false;
      pending_route_.reset();
      response->success = true;
      response->message = "Inspection static START planning cancellation requested.";
      return;
    }
    if (!active_goal_) {
      response->success = false;
      response->message = "No active inspection FollowPath goal.";
      return;
    }
    action_client_->async_cancel_goal(active_goal_);
    response->success = true;
    response->message = "FollowPath cancellation requested.";
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    const auto add_selection = [this, &markers](const std::optional<size_t> & index,
        const std::string & name, int id, float r, float g, float b) {
        if (!index.has_value() || *index >= path_.poses.size()) {
          return;
        }
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = frame_id_;
        sphere.header.stamp = now();
        sphere.ns = "inspection_route_selection";
        sphere.id = id;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose = path_.poses[*index].pose;
        sphere.pose.position.z = 0.16;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25;
        sphere.color.r = r;
        sphere.color.g = g;
        sphere.color.b = b;
        sphere.color.a = 1.0F;
        markers.markers.push_back(sphere);
        visualization_msgs::msg::Marker label = sphere;
        label.ns = "inspection_route_selection_label";
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.pose.position.z = 0.36;
        label.scale.z = 0.20;
        label.color.r = label.color.g = label.color.b = 1.0F;
        label.text = name;
        markers.markers.push_back(label);

        // Show the stored tangent orientation explicitly: a sphere alone
        // cannot tell the operator which way START will enter the loop.
        visualization_msgs::msg::Marker arrow = sphere;
        arrow.ns = "inspection_route_selection_direction";
        arrow.id = id;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.pose.position.z = 0.20;
        arrow.scale.x = 0.45;
        arrow.scale.y = 0.10;
        arrow.scale.z = 0.10;
        arrow.color.r = r;
        arrow.color.g = g;
        arrow.color.b = b;
        arrow.color.a = 1.0F;
        markers.markers.push_back(arrow);
      };
    add_selection(start_index_, "START", 0, 0.1F, 1.0F, 0.1F);
    add_selection(middle_index_, "MIDDLE", 1, 1.0F, 0.85F, 0.0F);
    add_selection(end_index_, "END", 2, 1.0F, 0.15F, 0.15F);
    marker_pub_->publish(markers);
  }

  std::string frame_id_;
  std::string follow_path_action_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string route_selection_file_;
  size_t selected_route_size_{0U};
  double follow_distance_remaining_{0.0};
  nav_msgs::msg::Path path_;
  std::optional<size_t> start_index_;
  std::optional<size_t> middle_index_;
  std::optional<size_t> end_index_;
  // Original clicks are retained even before the purple path exists, then
  // projected to the closest sampled route pose as soon as it arrives.
  std::optional<geometry_msgs::msg::Point> start_click_;
  std::optional<geometry_msgs::msg::Point> middle_click_;
  std::optional<geometry_msgs::msg::Point> end_click_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
  GoalHandleFollowPath::SharedPtr active_goal_;
  bool approach_planning_{false};
  std::optional<nav_msgs::msg::Path> pending_route_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr middle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr end_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr follow_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace inspection_geometry_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_geometry_tools::InspectionPathExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
