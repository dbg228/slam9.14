#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fcntl.h>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "inspection_geometry_tools/ordered_path_progress.hpp"

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
    // A second executor has the same ROS name, topics and FollowPath action.
    // ROS 2 permits that situation, but the two instances can preempt each
    // other's goals without an obvious RViz error.  Hold an OS lock for the
    // node lifetime so a duplicate launch fails immediately and visibly.
    executor_lock_fd_ = open("/tmp/inspection_path_executor.lock", O_CREAT | O_RDWR, 0644);
    if (executor_lock_fd_ < 0 || flock(executor_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      const std::string reason = executor_lock_fd_ < 0 ? std::strerror(errno) : "another executor is running";
      if (executor_lock_fd_ >= 0) {
        close(executor_lock_fd_);
        executor_lock_fd_ = -1;
      }
      RCLCPP_FATAL(get_logger(),
        "Only one inspection_path_executor_node may run (%s). Stop the older "
        "inspection_geometry launch before starting another one.", reason.c_str());
      throw std::runtime_error("inspection path executor is already running");
    }
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    follow_path_action_ = declare_parameter<std::string>("follow_path_action", "/follow_path");
    // The approach and the purple segment are concatenated into ONE fixed
    // FollowPath goal.  In particular, START is not a controller goal: this
    // prevents MPPI from slowing down and searching for a pose at START.
    controller_id_ = declare_parameter<std::string>("controller_id", "InspectionStrict");
    // Nav2 Humble does not treat an empty FollowPath goal_checker_id as
    // "use the default": it tries to find a checker literally named "" and
    // aborts before publishing any velocity command.  Select it explicitly.
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "inspection_route_goal_checker");
    route_selection_file_ = declare_parameter<std::string>(
      "route_selection_file", "/home/zkam/test_nav/robot_data/inspection_route_selection.txt");
    // /Odometry is only the sampling clock.  The localization binary labels
    // its raw LiDAR pose as base_link, while Nav2 drives the corrected TF
    // base_link.  Record that TF so reference and actual paths use one point.
    tracking_odom_topic_ = declare_parameter<std::string>("tracking_odom_topic", "/Odometry");
    tracking_base_frame_ = declare_parameter<std::string>("tracking_base_frame", "base_link");
    experiment_output_dir_ = declare_parameter<std::string>(
      "experiment_output_dir", "/home/zkam/test_nav/robot_data/experiments");
    experiment_label_ = declare_parameter<std::string>("experiment_label", "mppi");
    follow_feedback_timeout_sec_ = declare_parameter<double>("follow_feedback_timeout_sec", 5.0);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/inspection_path", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&InspectionPathExecutorNode::pathCallback, this, std::placeholders::_1));
    active_reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/inspection_route/active_reference_path", rclcpp::QoS(1).reliable());
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
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/inspection_route/reference_path", rclcpp::QoS(1).reliable().transient_local());
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/inspection_route/actual_path", rclcpp::QoS(1).reliable().transient_local());
    tracking_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      tracking_odom_topic_, rclcpp::QoS(100),
      std::bind(&InspectionPathExecutorNode::trackingOdomCallback, this, std::placeholders::_1));
    follow_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/follow",
      std::bind(&InspectionPathExecutorNode::follow, this, std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/cancel",
      std::bind(&InspectionPathExecutorNode::cancel, this, std::placeholders::_1, std::placeholders::_2));
    pause_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/pause",
      std::bind(&InspectionPathExecutorNode::pause, this, std::placeholders::_1, std::placeholders::_2));
    resume_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/resume",
      std::bind(&InspectionPathExecutorNode::resume, this, std::placeholders::_1, std::placeholders::_2));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/save",
      std::bind(&InspectionPathExecutorNode::save, this, std::placeholders::_1, std::placeholders::_2));
    load_service_ = create_service<std_srvs::srv::Trigger>(
      "/inspection_route/load",
      std::bind(&InspectionPathExecutorNode::load, this, std::placeholders::_1, std::placeholders::_2));
    action_client_ = rclcpp_action::create_client<FollowPath>(this, follow_path_action_);
    compute_path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/compute_path_to_pose");
    // A stale FollowPath action endpoint can remain visible while Nav2 is
    // already deactivating. Query controller_server itself: its lifecycle
    // manager service can disappear during heavy FAST-LIO load even while the
    // controller is healthy and active.
    nav_ready_client_ = create_client<lifecycle_msgs::srv::GetState>(
      "/controller_server/get_state");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(250),
      std::bind(&InspectionPathExecutorNode::publishStatus, this));
    nav_health_timer_ = create_wall_timer(std::chrono::seconds(1),
      std::bind(&InspectionPathExecutorNode::refreshNavHealth, this));
    refreshNavHealth();
    publishStatus();
  }

  ~InspectionPathExecutorNode() override
  {
    if (executor_lock_fd_ >= 0) {
      flock(executor_lock_fd_, LOCK_UN);
      close(executor_lock_fd_);
    }
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
    checkFollowHealth();
    if (active_goal_ && recording_) {
      // The ordered goal checker only accepts a path published after its
      // action reset. Repeating the same immutable path also removes the
      // goal-acceptance/reset callback race; pose geometry never changes.
      auto reference = active_follow_route_;
      reference.header.stamp = now();
      active_reference_path_pub_->publish(reference);
    }
    const auto start = selectedPoint(start_index_, start_click_);
    const auto end = selectedPoint(end_index_, end_click_);
    const auto route = selectedRoute();
    std::ostringstream text;
    text << std::fixed << std::setprecision(2);
    text << "START: " << (start_index_.has_value() ? "已选" : "未选")
         << " | END: " << (end_index_.has_value() ? "已选" : "未选");
    text << "\nNav2: " << (nav_active_ ? "控制器已就绪" : nav_health_message_);
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
    } else if (paused_ || pause_requested_) {
      text << "\n巡检已暂停，保留原路线进度；纠偏后从剩余紫线继续。";
    } else if (active_goal_) {
      text << "\n固定路径跟踪中，距 END（Nav2）: " << follow_distance_remaining_ << " m";
      text << "\n终点判定：需按顺序经过 START 和完整巡检路线。";
    } else if (start.has_value() && end.has_value() && nav_active_) {
      text << "\n可保存后点击“开始路线”。";
    } else if (start.has_value() && end.has_value()) {
      text << "\n等待 Nav2 控制器恢复 active 后再开始路线。";
    }
    std_msgs::msg::String status;
    status.data = text.str();
    status_pub_->publish(status);
  }

  void refreshNavHealth()
  {
    const auto now_wall = std::chrono::steady_clock::now();
    if (nav_health_request_in_flight_ &&
      now_wall - nav_health_request_time_ < std::chrono::seconds(2))
    {
      return;
    }
    if (!nav_ready_client_->service_is_ready()) {
      nav_active_ = false;
      nav_health_message_ = "导航控制器未启动/已停止";
      nav_health_request_in_flight_ = false;
      return;
    }
    nav_health_request_in_flight_ = true;
    nav_health_request_time_ = now_wall;
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    nav_ready_client_->async_send_request(request,
      [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
        nav_health_request_in_flight_ = false;
        try {
          const auto response = future.get();
          nav_active_ = response->current_state.id ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
          nav_health_message_ = nav_active_ ? "控制器已就绪" : "导航生命周期未 active";
        } catch (const std::exception &) {
          nav_active_ = false;
          nav_health_message_ = "导航控制器健康检查失败";
        }
      });
  }

  void checkFollowHealth()
  {
    if (!active_goal_ || follow_feedback_timeout_sec_ <= 0.0 || follow_fault_reported_) {
      return;
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_follow_feedback_time_).count();
    if (elapsed <= follow_feedback_timeout_sec_) {
      return;
    }
    follow_fault_reported_ = true;
    nav_active_ = false;
    nav_health_message_ = "FollowPath 无反馈，已判定控制器失效";
    RCLCPP_ERROR(get_logger(),
      "FollowPath was accepted but produced no feedback for %.1f s. "
      "Canceling it; restart navigation.sh before retrying.", elapsed);
    action_client_->async_cancel_goal(active_goal_);
    active_goal_.reset();
    follow_distance_remaining_ = 0.0;
    exportExperiment("controller_no_feedback");
  }

  struct TrackingSample
  {
    double time_s{};
    double x{};
    double y{};
    double yaw{};
    double vx{};
    double vy{};
    double wz{};
  };

  struct PathProjection
  {
    double distance{std::numeric_limits<double>::infinity()};
    double along_track{};
    size_t segment{};
  };

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static PathProjection projectToPath(const nav_msgs::msg::Path & path, double x, double y)
  {
    PathProjection best;
    double cumulative = 0.0;
    for (size_t index = 1U; index < path.poses.size(); ++index) {
      const auto & a = path.poses[index - 1U].pose.position;
      const auto & b = path.poses[index].pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length = std::hypot(dx, dy);
      if (length < 1e-6) {
        continue;
      }
      const double ratio = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / (length * length), 0.0, 1.0);
      const double error = std::hypot(x - (a.x + ratio * dx), y - (a.y + ratio * dy));
      if (error < best.distance) {
        best.distance = error;
        best.along_track = cumulative + ratio * length;
        best.segment = index - 1U;
      }
      cumulative += length;
    }
    return best;
  }

  static std::string timestampForFile()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t raw_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&raw_time, &local_time);
    std::ostringstream text;
    text << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return text.str();
  }

  void trackingOdomCallback(const nav_msgs::msg::Odometry::SharedPtr)
  {
    if (!recording_) {
      return;
    }
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
      map_to_base = tf_buffer_->lookupTransform(frame_id_, tracking_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Tracking sample skipped: cannot transform %s to %s: %s",
        tracking_base_frame_.c_str(), frame_id_.c_str(), exception.what());
      return;
    }
    const auto sample_stamp = now();
    TrackingSample sample;
    sample.time_s = sample_stamp.seconds() - experiment_start_stamp_.seconds();
    sample.x = map_to_base.transform.translation.x;
    sample.y = map_to_base.transform.translation.y;
    sample.yaw = yawFromQuaternion(map_to_base.transform.rotation);
    if (!tracking_samples_.empty() && sample.time_s <= tracking_samples_.back().time_s) {
      return;
    }
    if (!tracking_samples_.empty()) {
      const auto & previous = tracking_samples_.back();
      const double delta_time = sample.time_s - previous.time_s;
      if (delta_time > 1e-4) {
        sample.vx = (sample.x - previous.x) / delta_time;
        sample.vy = (sample.y - previous.y) / delta_time;
        sample.wz = std::atan2(std::sin(sample.yaw - previous.yaw),
          std::cos(sample.yaw - previous.yaw)) / delta_time;
      }
    }
    tracking_samples_.push_back(sample);
    if (!paused_ && !pause_requested_ && !resume_pending_) {
      route_progress_.update(sample.x, sample.y);
      const size_t next = std::min(route_progress_.visited(), progress_indices_.size() - 1U);
      const size_t first = next > 12U ? next - 12U : 0U;
      double best = std::numeric_limits<double>::infinity();
      for (size_t i = first; i <= next; ++i) {
        const auto index = progress_indices_[i];
        const auto & point = tracking_reference_.poses[index].pose.position;
        const double separation = std::hypot(sample.x - point.x, sample.y - point.y);
        if (separation < best) {best = separation; resume_index_ = index;}
      }
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = sample_stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = sample.x;
    pose.pose.position.y = sample.y;
    pose.pose.position.z = map_to_base.transform.translation.z;
    pose.pose.orientation = map_to_base.transform.rotation;
    actual_path_.header = pose.header;
    actual_path_.header.frame_id = frame_id_;
    actual_path_.poses.push_back(pose);
    actual_path_pub_->publish(actual_path_);
  }

  void beginExperiment(const nav_msgs::msg::Path & reference)
  {
    tracking_reference_ = reference;
    tracking_reference_.header.frame_id = frame_id_;
    tracking_reference_.header.stamp = now();
    tracking_samples_.clear();
    progress_indices_.clear();
    std::vector<RoutePoint> points;
    for (size_t i = 0U; i < reference.poses.size(); ++i) {
      const auto & p = reference.poses[i].pose.position;
      if (points.empty() || std::hypot(p.x - points.back().x, p.y - points.back().y) > 1e-6) {
        points.push_back({p.x, p.y}); progress_indices_.push_back(i);
      }
    }
    route_progress_.setPath(points, 0.35);
    resume_index_ = 0U;
    actual_path_.poses.clear();
    actual_path_.header.frame_id = frame_id_;
    experiment_start_stamp_ = now();
    recording_ = true;
    reference_path_pub_->publish(tracking_reference_);
    RCLCPP_INFO(get_logger(), "Tracking experiment started: label=%s, reference=%.2f m.",
      experiment_label_.c_str(), pathLength(tracking_reference_));
  }

  static void writeTimeSeriesSvg(const std::filesystem::path & file,
    const std::string & title, const std::string & y_label, const std::string & color,
    const std::vector<std::pair<double, double>> & values, const std::string & result)
  {
    std::ofstream output(file);
    output << std::fixed << std::setprecision(3)
      << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1120\" height=\"860\" viewBox=\"0 0 1120 860\">\n"
      << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n"
      << "<text x=\"100\" y=\"42\" font-family=\"sans-serif\" font-size=\"24\">" << title << "</text>\n"
      << "<text x=\"100\" y=\"70\" font-family=\"sans-serif\" font-size=\"16\">result=" << result << "</text>\n";
    if (values.empty()) {
      output << "<text x=\"100\" y=\"130\" font-family=\"sans-serif\" font-size=\"20\" fill=\"#d32f2f\">"
             << "No /Odometry samples were received during this trial.</text>\n</svg>\n";
      return;
    }
    const double plot_left = 110.0;
    const double plot_top = 110.0;
    const double plot_width = 900.0;
    const double plot_height = 620.0;
    const double max_time = std::max(1.0, values.back().first);
    double max_value = 0.01;
    for (const auto & value : values) {
      max_value = std::max(max_value, value.second);
    }
    max_value *= 1.10;
    const auto x = [plot_left, plot_width, max_time](double value) {
        return plot_left + value / max_time * plot_width;
      };
    const auto y = [plot_top, plot_height, max_value](double value) {
        return plot_top + plot_height - value / max_value * plot_height;
      };
    output << "<line x1=\"" << plot_left << "\" y1=\"" << plot_top + plot_height
           << "\" x2=\"" << plot_left + plot_width << "\" y2=\"" << plot_top + plot_height
           << "\" stroke=\"#333\" stroke-width=\"2\"/>\n"
           << "<line x1=\"" << plot_left << "\" y1=\"" << plot_top << "\" x2=\"" << plot_left
           << "\" y2=\"" << plot_top + plot_height << "\" stroke=\"#333\" stroke-width=\"2\"/>\n"
           << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\"3\" points=\"";
    for (const auto & value : values) {
      output << x(value.first) << ',' << y(value.second) << ' ';
    }
    output << "\"/>\n"
           << "<text x=\"" << plot_left + plot_width - 140.0 << "\" y=\"" << plot_top + plot_height + 42.0
           << "\" font-family=\"sans-serif\" font-size=\"18\">time (s), max=" << max_time << "</text>\n"
           << "<text x=\"" << plot_left << "\" y=\"" << plot_top - 12.0
           << "\" font-family=\"sans-serif\" font-size=\"18\">" << y_label << ", max=" << max_value / 1.10
           << "</text>\n</svg>\n";
  }

  void exportExperiment(const std::string & result)
  {
    if (!recording_) {
      return;
    }
    recording_ = false;
    if (tracking_reference_.poses.size() < 2U) {
      RCLCPP_WARN(get_logger(), "Tracking experiment has no reference path; nothing exported.");
      return;
    }
    const std::filesystem::path directory = std::filesystem::path(experiment_output_dir_) /
      (timestampForFile() + "_" + experiment_label_);
    std::filesystem::create_directories(directory);
    const double reference_length = pathLength(tracking_reference_);
    const bool has_samples = !tracking_samples_.empty();
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    double actual_length = 0.0;
    double absolute_sum = 0.0;
    double squared_sum = 0.0;
    double maximum_error = 0.0;
    std::vector<PathProjection> projections;
    projections.reserve(tracking_samples_.size());
    for (size_t index = 0U; index < tracking_samples_.size(); ++index) {
      const auto & sample = tracking_samples_[index];
      const auto projection = projectToPath(tracking_reference_, sample.x, sample.y);
      projections.push_back(projection);
      absolute_sum += projection.distance;
      squared_sum += projection.distance * projection.distance;
      maximum_error = std::max(maximum_error, projection.distance);
      if (index > 0U) {
        actual_length += std::hypot(sample.x - tracking_samples_[index - 1U].x,
          sample.y - tracking_samples_[index - 1U].y);
      }
    }
    const double sample_count = static_cast<double>(tracking_samples_.size());
    const double mae = has_samples ? absolute_sum / sample_count : unavailable;
    const double rmse = has_samples ? std::sqrt(squared_sum / sample_count) : unavailable;
    const double duration = has_samples ? tracking_samples_.back().time_s : 0.0;
    const double final_error = has_samples ? projections.back().distance : unavailable;

    // SVG is a vector figure: it can be inserted into a paper directly or
    // converted to PNG/PDF later without losing line quality.
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    const auto expand_bounds = [&min_x, &max_x, &min_y, &max_y](double x, double y) {
        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
      };
    for (const auto & pose : tracking_reference_.poses) {
      expand_bounds(pose.pose.position.x, pose.pose.position.y);
    }
    for (const auto & sample : tracking_samples_) {
      expand_bounds(sample.x, sample.y);
    }
    const double border = 0.5;
    min_x -= border; max_x += border; min_y -= border; max_y += border;
    const double plot_width = 920.0;
    const double plot_height = 680.0;
    const double scale = std::min(plot_width / std::max(0.1, max_x - min_x),
      plot_height / std::max(0.1, max_y - min_y));
    const double offset_x = 100.0 + 0.5 * (plot_width - (max_x - min_x) * scale);
    const double offset_y = 110.0 + 0.5 * (plot_height - (max_y - min_y) * scale);
    const auto svg_x = [min_x, scale, offset_x](double x) {return offset_x + (x - min_x) * scale;};
    const auto svg_y = [max_y, scale, offset_y](double y) {return offset_y + (max_y - y) * scale;};
    std::ofstream figure_file(directory / "trajectory.svg");
    figure_file << std::fixed << std::setprecision(2)
      << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1120\" height=\"860\" viewBox=\"0 0 1120 860\">\n"
      << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n"
      << "<text x=\"100\" y=\"42\" font-family=\"sans-serif\" font-size=\"24\">Tracking experiment: "
      << experiment_label_ << " (" << controller_id_ << ")</text>\n"
      << "<text x=\"100\" y=\"70\" font-family=\"sans-serif\" font-size=\"16\">"
      << "result=" << result << ", RMSE=" << rmse << " m, MAE=" << mae << " m, max=" << maximum_error
      << " m, final=" << final_error << " m, duration=" << duration << " s, ref=" << reference_length
      << " m, actual=" << actual_length << " m</text>\n";
    const auto write_polyline = [&figure_file](const char * color, double width, const auto & points) {
        figure_file << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\"" << width
                    << "\" points=\"";
        for (const auto & point : points) {
          figure_file << point.first << ',' << point.second << ' ';
        }
        figure_file << "\"/>\n";
      };
    std::vector<std::pair<double, double>> reference_points;
    std::vector<std::pair<double, double>> actual_points;
    for (const auto & pose : tracking_reference_.poses) {
      reference_points.emplace_back(svg_x(pose.pose.position.x), svg_y(pose.pose.position.y));
    }
    for (const auto & sample : tracking_samples_) {
      actual_points.emplace_back(svg_x(sample.x), svg_y(sample.y));
    }
    write_polyline("#bf00bf", 4.0, reference_points);
    write_polyline("#e53935", 2.5, actual_points);
    figure_file << "<circle cx=\"" << reference_points.front().first << "\" cy=\""
                << reference_points.front().second << "\" r=\"6\" fill=\"#18a558\"/>\n"
                << "<circle cx=\"" << reference_points.back().first << "\" cy=\""
                << reference_points.back().second << "\" r=\"6\" fill=\"#d32f2f\"/>\n"
                << "<text x=\"100\" y=\"825\" font-family=\"sans-serif\" font-size=\"16\" fill=\"#bf00bf\">Purple: reference FollowPath</text>\n"
                << "<text x=\"400\" y=\"825\" font-family=\"sans-serif\" font-size=\"16\" fill=\"#e53935\">Red: estimated robot trajectory</text>\n"
                << "</svg>\n";
    std::vector<std::pair<double, double>> error_values;
    std::vector<std::pair<double, double>> speed_values;
    error_values.reserve(tracking_samples_.size());
    speed_values.reserve(tracking_samples_.size());
    for (size_t index = 0U; index < tracking_samples_.size(); ++index) {
      const auto & sample = tracking_samples_[index];
      error_values.emplace_back(sample.time_s, projections[index].distance);
      speed_values.emplace_back(sample.time_s, std::hypot(sample.vx, sample.vy));
    }
    writeTimeSeriesSvg(directory / "cross_track_error.svg",
      "Cross-track error: " + experiment_label_, "error (m)", "#e53935", error_values, result);
    writeTimeSeriesSvg(directory / "speed_profile.svg",
      "Estimated speed: " + experiment_label_, "speed (m/s)", "#1976d2", speed_values, result);
    if (has_samples) {
      RCLCPP_INFO(get_logger(),
        "Tracking experiment exported: %s (result=%s, RMSE=%.3f m, MAE=%.3f m, max=%.3f m).",
        directory.c_str(), result.c_str(), rmse, mae, maximum_error);
    } else {
      RCLCPP_WARN(get_logger(),
        "Tracking experiment exported without odometry samples: %s (result=%s).",
        directory.c_str(), result.c_str());
    }
  }

  void sendFollowPath(const nav_msgs::msg::Path & route, bool continuing = false)
  {
    FollowPath::Goal goal;
    goal.path = route;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;
    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = [this, route, continuing](GoalHandleFollowPath::SharedPtr goal_handle) {
        resume_pending_ = false;
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Nav2 rejected the inspection FollowPath goal.");
          return;
        }
        active_goal_ = goal_handle;
        active_follow_route_ = route;
        paused_ = false;
        last_follow_feedback_time_ = std::chrono::steady_clock::now();
        follow_fault_reported_ = false;
        if (!continuing) {beginExperiment(route);}
        RCLCPP_INFO(get_logger(), "Nav2 accepted %zu-pose inspection route.", selected_route_size_);
      };
    options.feedback_callback = [this](GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback) {
        follow_distance_remaining_ = feedback->distance_to_goal;
        last_follow_feedback_time_ = std::chrono::steady_clock::now();
      };
    options.result_callback = [this](const GoalHandleFollowPath::WrappedResult & result) {
      active_goal_.reset();
      follow_distance_remaining_ = 0.0;
        if (pause_requested_ && result.code == rclcpp_action::ResultCode::CANCELED) {
          pause_requested_ = false; paused_ = true;
          RCLCPP_INFO(get_logger(), "Inspection paused at original pose %zu; tracking retained.", resume_index_);
          publishStatus();
          return;
        }
        pause_requested_ = false; paused_ = false;
        if (follow_fault_reported_) {
          RCLCPP_WARN(get_logger(),
            "Inspection FollowPath result received after the no-feedback watchdog; result ignored.");
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Inspection FollowPath route completed.");
          exportExperiment("succeeded");
        } else {
          RCLCPP_WARN(get_logger(), "Inspection FollowPath finished with action result code %d.",
            static_cast<int>(result.code));
          exportExperiment(result.code == rclcpp_action::ResultCode::CANCELED ? "canceled" : "failed");
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
    if (active_goal_ || approach_planning_ || paused_ || resume_pending_) {
      response->success = false;
      response->message = "An inspection static approach or FollowPath goal is already active.";
      return;
    }
    if (!nav_active_) {
      response->success = false;
      response->message = "Nav2 controller is not active. Restart navigation.sh and wait for Nav2 ready before starting.";
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

  void pause(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!active_goal_ || pause_requested_ || !recording_) {
      response->success = false; response->message = "No active inspection route to pause."; return;
    }
    pause_requested_ = true;
    action_client_->async_cancel_goal(active_goal_);
    response->success = true;
    response->message = "Pause requested; wait for FollowPath cancellation and a stationary chassis before correction.";
  }

  void resume(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!paused_ || active_goal_ || resume_pending_ || !recording_ || !nav_active_) {
      response->success = false; response->message = "No idle paused route with an active controller."; return;
    }
    if (resume_index_ + 1U >= tracking_reference_.poses.size()) {
      response->success = false; response->message = "No remaining route poses."; return;
    }
    auto remaining = tracking_reference_;
    remaining.poses.erase(remaining.poses.begin(), remaining.poses.begin() + resume_index_);
    remaining.header.stamp = now();
    resume_pending_ = true;
    sendFollowPath(remaining, true);
    response->success = true;
    response->message = "Resuming the unchanged remaining reference poses; no START replanning.";
  }

  void cancel(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (paused_ && !active_goal_) {
      paused_ = false; pause_requested_ = false;
      exportExperiment("canceled");
      response->success = true; response->message = "Paused route canceled."; return;
    }
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
    pause_requested_ = false;
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_reference_path_pub_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string route_selection_file_;
  std::string tracking_odom_topic_;
  std::string tracking_base_frame_;
  std::string experiment_output_dir_;
  std::string experiment_label_;
  int executor_lock_fd_{-1};
  size_t selected_route_size_{0U};
  double follow_distance_remaining_{0.0};
  double follow_feedback_timeout_sec_{5.0};
  bool follow_fault_reported_{false};
  std::chrono::steady_clock::time_point last_follow_feedback_time_{std::chrono::steady_clock::now()};
  bool nav_active_{false};
  bool nav_health_request_in_flight_{false};
  std::chrono::steady_clock::time_point nav_health_request_time_{std::chrono::steady_clock::now()};
  std::string nav_health_message_{"等待 Nav2 控制器"};
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
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr nav_ready_client_;
  GoalHandleFollowPath::SharedPtr active_goal_;
  bool approach_planning_{false};
  std::optional<nav_msgs::msg::Path> pending_route_;
  bool recording_{false};
  rclcpp::Time experiment_start_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path tracking_reference_;
  nav_msgs::msg::Path active_follow_route_;
  OrderedPathProgress route_progress_;
  std::vector<size_t> progress_indices_;
  size_t resume_index_{0U};
  bool pause_requested_{false};
  bool paused_{false};
  bool resume_pending_{false};
  nav_msgs::msg::Path actual_path_;
  std::vector<TrackingSample> tracking_samples_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracking_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr middle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr end_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr follow_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr nav_health_timer_;
};
}  // namespace inspection_geometry_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_geometry_tools::InspectionPathExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
