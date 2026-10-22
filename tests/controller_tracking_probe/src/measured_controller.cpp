#include <chrono>
#include <memory>
#include <string>
#include "nav2_core/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"

namespace controller_tracking {
class MeasuredController : public nav2_core::Controller {
  // Loader must outlive the delegated object. No MPPI code or parameters are copied.
  pluginlib::ClassLoader<nav2_core::Controller> loader_{"nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr inner_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr speed_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr timing_;
  uint64_t sequence_{0};
  void timing(int64_t call_ns, int64_t duration, bool ok) {
    std_msgs::msg::String m;
    m.data = "{\"sequence\":" + std::to_string(sequence_++) +
      ",\"call_ros_ns\":" + std::to_string(call_ns) +
      ",\"compute_steady_ns\":" + std::to_string(duration) +
      ",\"ok\":" + (ok ? "true" : "false") + "}";
    timing_->publish(m);
  }
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_ = parent.lock();
    plan_ = node_->create_publisher<nav_msgs::msg::Path>("received_plan", rclcpp::QoS(1).transient_local());
    pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("input_pose", 50);
    speed_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("input_speed", 50);
    timing_ = node_->create_publisher<std_msgs::msg::String>("compute_timing", 50);
    inner_ = loader_.createSharedInstance("nav2_mppi_controller::MPPIController");
    inner_->configure(parent, name, tf, costmap);
  }
  void activate() override {
    inner_->activate(); plan_->on_activate(); pose_->on_activate(); speed_->on_activate(); timing_->on_activate();
  }
  void deactivate() override {
    inner_->deactivate(); plan_->on_deactivate(); pose_->on_deactivate(); speed_->on_deactivate(); timing_->on_deactivate();
  }
  void cleanup() override {
    inner_->cleanup(); inner_.reset(); plan_.reset(); pose_.reset(); speed_.reset(); timing_.reset(); node_.reset();
  }
  void setPlan(const nav_msgs::msg::Path & path) override {
    inner_->setPlan(path); plan_->publish(path);
  }
  void setSpeedLimit(const double & limit, const bool & percentage) override {
    inner_->setSpeedLimit(limit, percentage);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override {
    const auto call = node_->now();
    // These are precisely the arguments passed to MPPI (after Nav2 velocity deadbands).
    pose_->publish(pose);
    geometry_msgs::msg::TwistStamped speed;
    speed.header.frame_id = "base_link";
    speed.header.stamp = call;  // invocation time, NOT a velocity measurement timestamp
    speed.twist = velocity; speed_->publish(speed);
    auto start = std::chrono::steady_clock::now();
    try {
      auto command = inner_->computeVelocityCommands(pose, velocity, checker);
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
      timing(call.nanoseconds(), ns, true);
      return command;
    } catch (...) {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
      timing(call.nanoseconds(), ns, false); throw;
    }
  }
};
}  // namespace controller_tracking
PLUGINLIB_EXPORT_CLASS(controller_tracking::MeasuredController, nav2_core::Controller)
