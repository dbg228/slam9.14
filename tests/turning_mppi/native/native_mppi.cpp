// Offline adapter to the installed, unmodified Nav2 MPPI controller plugin.
// It never creates a velocity publisher or starts an executor/costmap thread.
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <xtensor/xrandom.hpp>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_loader.hpp"

namespace {
thread_local std::string last_error;
std::mutex api_mutex;
rclcpp::Context::SharedPtr offline_context;

rclcpp::NodeOptions node_options() {
  if (!offline_context) {
    // All nodes live on a private ROS domain, even if the shell is sourced for
    // the real robot. No real /cmd_vel publisher exists in this adapter.
    // ROS_LOCALHOST_ONLY=1 makes this RMW inject a UDP transport even when XML
    // requests SHM only. The explicit SHM profile therefore provides isolation.
    setenv("ROS_LOCALHOST_ONLY", "0", 1);
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    setenv("FASTRTPS_DEFAULT_PROFILES_FILE", OFFLINE_FASTDDS_PROFILE, 1);
    const std::string log_dir = "/tmp/turning_mppi_native_logs";
    std::filesystem::create_directories(log_dir);
    setenv("ROS_LOG_DIR", log_dir.c_str(), 1);
    rclcpp::InitOptions init;
    init.set_domain_id(231);
    offline_context = std::make_shared<rclcpp::Context>();
    const char * args[] = {"offline_mppi", "--ros-args", "--log-level", "error",
      "--disable-external-lib-logs", "--disable-rosout-logs"};
    offline_context->init(6, args, init);
  }
  return rclcpp::NodeOptions().context(offline_context).use_global_arguments(false)
    .enable_rosout(false).start_parameter_services(false)
    .start_parameter_event_publisher(false);
}

// Construct only the in-memory costmap required by the real plugin. Calling
// Costmap2DROS::on_configure/on_activate would start sensors and ROS timers.
class OfflineCostmap : public nav2_costmap_2d::Costmap2DROS {
 public:
  explicit OfflineCostmap(const YAML::Node & config)
    : Costmap2DROS(node_options()) {
    global_frame_ = "odom";
    robot_base_frame_ = "base_link";
    transform_tolerance_ = config["transform_tolerance"].as<double>(0.5);
    resolution_ = config["resolution"].as<double>(0.05);
    const double width = config["width"].as<double>(10.0);
    const double height = config["height"].as<double>(10.0);
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      global_frame_, true, false);
    layered_costmap_->resizeMap(
      static_cast<unsigned>(std::ceil(width / resolution_)),
      static_cast<unsigned>(std::ceil(height / resolution_)), resolution_,
      -width / 2.0, -height / 2.0);
    footprint_padding_ = config["footprint_padding"].as<double>(0.03);
    std::vector<geometry_msgs::msg::Point> footprint;
    auto points = YAML::Load(config["footprint"].as<std::string>(
      "[[0.45,0.355],[0.45,-0.355],[-0.45,-0.355],[-0.45,0.355]]"));
    for (const auto & xy : points) {
      geometry_msgs::msg::Point p;
      p.x = xy[0].as<double>(); p.y = xy[1].as<double>();
      footprint.push_back(p);
    }
    setRobotFootprint(footprint);
  }
  void center(double x, double y) {
    auto * map = getCostmap();
    map->updateOrigin(x - map->getSizeInMetersX() / 2,
                     y - map->getSizeInMetersY() / 2);
    map->resetMap(0, 0, map->getSizeInCellsX(), map->getSizeInCellsY());
  }
};

void add_parameters(const YAML::Node & node, const std::string & prefix,
                    std::vector<rclcpp::Parameter> & out) {
  for (const auto & kv : node) {
    auto key = kv.first.as<std::string>();
    auto name = prefix.empty() ? key : prefix + "." + key;
    const auto & value = kv.second;
    if (value.IsMap()) {
      add_parameters(value, name, out);
    } else if (value.IsSequence()) {
      // Controller and critic arrays in the frozen source YAML are strings.
      out.emplace_back(name, value.as<std::vector<std::string>>());
    } else if (value.IsScalar()) {
      const auto scalar = value.Scalar();
      if (scalar == "true" || scalar == "True" || scalar == "false" || scalar == "False") {
        out.emplace_back(name, value.as<bool>());
      } else {
        std::size_t consumed = 0;
        bool added = false;
        try {
          auto integer = std::stoll(scalar, &consumed);
          if (consumed == scalar.size()) {
            out.emplace_back(name, static_cast<int64_t>(integer)); added = true;
          }
        } catch (const std::exception &) {}
        if (!added) {
          try {
            auto real = std::stod(scalar, &consumed);
            if (consumed == scalar.size()) {
              out.emplace_back(name, real); added = true;
            }
          } catch (const std::exception &) {}
        }
        if (!added) out.emplace_back(name, scalar);
      }
    }
  }
}

class OfflineGoalChecker : public nav2_core::GoalChecker {
 public:
  explicit OfflineGoalChecker(double xy) : xy_(xy) {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override { return false; }
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & vel) override {
    pose.position.x = xy_; pose.position.y = xy_;
    pose.orientation.z = std::sin(3.14 / 2); pose.orientation.w = std::cos(3.14 / 2);
    vel.linear.x = vel.linear.y = vel.angular.z = std::numeric_limits<double>::lowest();
    return true;
  }
 private: double xy_;
};

struct Harness {
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<OfflineCostmap> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  // The installed MPPI binary uses private xtensor/xsimd compile flags. Its
  // plugin factory must allocate and construct the complete object itself.
  pluginlib::ClassLoader<nav2_core::Controller> loader{
    "nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr controller;
  std::unique_ptr<OfflineGoalChecker> goal_checker;
  bool configured{false};
  bool plan_set{false};
  Harness(const char * yaml_path, const char * plugin_name, unsigned seed) {
    const auto root = YAML::LoadFile(yaml_path);
    const auto params = root["controller_server"]["ros__parameters"];
    const auto plugin = params[plugin_name];
    if (!plugin || plugin["plugin"].as<std::string>() !=
      "nav2_mppi_controller::MPPIController") {
      throw std::runtime_error("Selected YAML plugin is not original Nav2 MPPIController");
    }
    // Seed xtensor's engine used by the original shared library; no scoring,
    // optimization, sampling, path handling or output logic is reimplemented.
    xt::random::seed(seed);
    std::vector<rclcpp::Parameter> overrides;
    overrides.emplace_back("controller_frequency", params["controller_frequency"].as<double>());
    add_parameters(plugin, plugin_name, overrides);
    auto options = node_options();
    options.parameter_overrides(overrides);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("offline_mppi_baseline", options);
    costmap = std::make_shared<OfflineCostmap>(
      root["local_costmap"]["local_costmap"]["ros__parameters"]);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    goal_checker = std::make_unique<OfflineGoalChecker>(
      params["goal_checker"]["xy_goal_tolerance"].as<double>(0.2));
    controller = loader.createSharedInstance("nav2_mppi_controller::MPPIController");
    controller->configure(node, plugin_name, tf, costmap);
    configured = true;
  }
  ~Harness() {
    if (configured) controller->cleanup();
  }
};

template<class Callable> int guarded(Callable call) {
  try { std::lock_guard<std::mutex> lock(api_mutex); call(); last_error.clear(); return 0; }
  catch (const std::exception & e) { last_error = e.what(); return -1; }
  catch (...) { last_error = "Unknown native MPPI exception"; return -1; }
}
}  // namespace

extern "C" {
const char * mppi_last_error() { return last_error.c_str(); }

void * mppi_create(const char * yaml_path, const char * plugin_name, unsigned seed) {
  Harness * result = nullptr;
  guarded([&] { result = new Harness(yaml_path, plugin_name, seed); });
  return result;
}

int mppi_set_path(void * opaque, const double * xy_yaw, int count) {
  return guarded([&] {
    if (!opaque || !xy_yaw || count < 2) throw std::runtime_error("Invalid path");
    auto & h = *static_cast<Harness *>(opaque);
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom"; path.header.stamp = h.node->now();
    for (int i = 0; i < count; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = xy_yaw[i * 3]; pose.pose.position.y = xy_yaw[i * 3 + 1];
      pose.pose.orientation.z = std::sin(xy_yaw[i * 3 + 2] / 2);
      pose.pose.orientation.w = std::cos(xy_yaw[i * 3 + 2] / 2);
      path.poses.push_back(pose);
    }
    h.controller->setPlan(path); h.plan_set = true;
  });
}

int mppi_step(void * opaque, double x, double y, double yaw, double vx, double wz,
              double * out_vw) {
  return guarded([&] {
    if (!opaque || !out_vw) throw std::runtime_error("Invalid MPPI handle/output");
    auto & h = *static_cast<Harness *>(opaque);
    if (!h.plan_set) throw std::runtime_error("Call set_path before step");
    h.costmap->center(x, y);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom"; pose.header.stamp = h.node->now();
    pose.pose.position.x = x; pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2); pose.pose.orientation.w = std::cos(yaw / 2);
    geometry_msgs::msg::Twist speed;
    speed.linear.x = vx; speed.angular.z = wz;
    const auto result = h.controller->computeVelocityCommands(pose, speed, h.goal_checker.get());
    out_vw[0] = result.twist.linear.x; out_vw[1] = result.twist.angular.z;
    if (!std::isfinite(out_vw[0]) || !std::isfinite(out_vw[1]))
      throw std::runtime_error("Original MPPI returned non-finite output");
  });
}

int mppi_reset(void * opaque) {
  return guarded([&] {
    if (!opaque) throw std::runtime_error("Invalid MPPI handle");
    auto * controller = dynamic_cast<nav2_mppi_controller::MPPIController *>(
      static_cast<Harness *>(opaque)->controller.get());
    if (!controller) throw std::runtime_error("Native plugin type mismatch");
    controller->reset();
  });
}

int mppi_destroy(void * opaque) {
  return guarded([&] { delete static_cast<Harness *>(opaque); });
}
}
