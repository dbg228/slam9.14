#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "static_map_localization/msg/ndt_match_diagnostic.hpp"
#include "static_map_localization/msg/ndt_match_diagnostic_detail.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace static_map_localization
{
namespace
{
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using NdtMatchDiagnostic = static_map_localization::msg::NdtMatchDiagnostic;
using NdtMatchDiagnosticDetail = static_map_localization::msg::NdtMatchDiagnosticDetail;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Isometry3f poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaternionf q(
    static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  if (q.norm() < 1e-4F) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3f(
    static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
    static_cast<float>(pose.position.z));
  return transform;
}

Eigen::Isometry3f transformToEigen(const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return poseToEigen(pose);
}

geometry_msgs::msg::Quaternion eigenToQuaternion(const Eigen::Matrix3f & rotation)
{
  Eigen::Quaternionf q(rotation);
  q.normalize();
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3f & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation = eigenToQuaternion(transform.rotation());
  return pose;
}

bool finiteTransform(const Eigen::Isometry3f & transform)
{
  return transform.matrix().allFinite();
}

Cloud::Ptr voxelDownsample(const Cloud::ConstPtr & input, const float leaf)
{
  auto output = std::make_shared<Cloud>();
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*output);
  return output;
}
}  // namespace

class StaticMapLocalizer final : public rclcpp::Node
{
public:
  StaticMapLocalizer()
  : Node("static_map_localizer")
  {
    map_pcd_path_ = declare_parameter<std::string>(
      "map_pcd_path", "/home/zkam/test_nav/robot_data/maps/data/scans.pcd");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    raw_tf_topic_ = declare_parameter<std::string>("raw_tf_topic", "/tf_lio_raw");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    raw_parent_frame_ = declare_parameter<std::string>("raw_parent_frame", "camera_init");
    raw_child_frame_ = declare_parameter<std::string>("raw_child_frame", "body");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    ready_topic_ = declare_parameter<std::string>("ready_topic", "/static_localization/ready");
    // User-confirmed base->sensor mount, 2026-10-16; not the lidar->IMU extrinsic.
    sensor_x_ = declare_parameter<double>("sensor_x", 0.37);
    sensor_y_ = declare_parameter<double>("sensor_y", 0.28);
    sensor_z_ = declare_parameter<double>("sensor_z", 0.15);
    sensor_yaw_ = declare_parameter<double>("sensor_yaw", 0.8229227423153264);
    // FAST-LIO remains responsible for high-rate odometry.  NDT is only a
    // slow global correction, so keep its clouds intentionally compact.
    // The previous 16k/2.7k-point registration took 6--15 seconds and
    // starved the real-time LIO process on the onboard computer.
    map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.40);
    target_voxel_size_ = declare_parameter<double>("target_voxel_size", 0.60);
    source_voxel_size_ = declare_parameter<double>("source_voxel_size", 0.40);
    submap_radius_ = declare_parameter<double>("submap_radius", 12.0);
    match_period_s_ = declare_parameter<double>("match_period_s", 2.0);
    ndt_resolution_ = declare_parameter<double>("ndt_resolution", 1.20);
    ndt_max_iterations_ = declare_parameter<int>("ndt_max_iterations", 15);
    ndt_step_size_ = declare_parameter<double>("ndt_step_size", 0.25);
    ndt_transformation_epsilon_ = declare_parameter<double>("ndt_transformation_epsilon", 0.01);
    max_fitness_score_ = declare_parameter<double>("max_fitness_score", 1.5);
    max_update_translation_ = declare_parameter<double>("max_update_translation", 2.5);
    max_update_yaw_ = declare_parameter<double>("max_update_yaw", 0.70);
    min_source_points_ = declare_parameter<int>("min_source_points", 250);
    // This only controls whether a diagnostic pose can be evaluated at the
    // cloud measurement time.  It never changes the NDT initial guess or any
    // acceptance decision.
    diagnostic_pose_max_skew_s_ = declare_parameter<double>("diagnostic_pose_max_skew_s", 0.10);
    diagnostic_pose_cache_s_ = declare_parameter<double>("diagnostic_pose_cache_s", 10.0);
    diagnostic_overlap_max_distance_m_ = declare_parameter<double>(
      "diagnostic_overlap_max_distance_m", ndt_resolution_);
    if (diagnostic_pose_max_skew_s_ <= 0.0 || diagnostic_pose_cache_s_ <= 0.0 ||
      ndt_step_size_ <= 0.0 || ndt_transformation_epsilon_ <= 0.0 ||
      diagnostic_overlap_max_distance_m_ <= 0.0)
    {
      throw std::runtime_error("Diagnostic pose cache parameters must be positive.");
    }

    loadStaticMap();
    base_to_sensor_ = Eigen::Isometry3f::Identity();
    base_to_sensor_.translation() = Eigen::Vector3f(sensor_x_, sensor_y_, sensor_z_);
    base_to_sensor_.linear() = Eigen::AngleAxisf(sensor_yaw_, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&StaticMapLocalizer::cloudCallback, this, std::placeholders::_1));
    raw_tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(raw_tf_topic_, 50,
      std::bind(&StaticMapLocalizer::rawTfCallback, this, std::placeholders::_1));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      std::bind(&StaticMapLocalizer::initialPoseCallback, this, std::placeholders::_1));
    initialpose_user_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose_user", 10,
      std::bind(&StaticMapLocalizer::initialPoseCallback, this, std::placeholders::_1));
    initialpose_checked_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose_checked", 10,
      std::bind(&StaticMapLocalizer::initialPoseCallback, this, std::placeholders::_1));

    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    map_to_odom_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
      "/static_localization/map_to_odom", rclcpp::QoS(1).reliable().transient_local());
    // camera_init is FAST-LIO's local-world frame.  In this stack it is the
    // same coordinate system as odom, but it must exist on /tf so RViz can
    // transform /cloud_registered (whose header frame is camera_init).
    publishOdomToCameraInit();
    // This latched topic is visualization-only.  It exposes the exact
    // downsampled PCD used as the NDT target, so an operator can seed an
    // initial pose against the fixed map in RViz.  It is never subscribed by
    // the matcher and does not affect localization, TF, or acceptance logic.
    map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/static_localization/map_cloud", rclcpp::QoS(1).reliable().transient_local());
    publishStaticMapCloud();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/static_localization/pose", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/static_localization/odometry", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/static_localization/status", 10);
    ndt_diagnostic_pub_ = create_publisher<NdtMatchDiagnostic>(
      "/static_localization/ndt_match_diagnostic", rclcpp::QoS(50).reliable());
    ndt_diagnostic_detail_pub_ = create_publisher<NdtMatchDiagnosticDetail>(
      "/static_localization/ndt_match_diagnostic_detail", rclcpp::QoS(50).reliable());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      ready_topic_, rclcpp::QoS(1).reliable().transient_local());
    // Publish an initial identity correction.  A dedicated, independent TF
    // broadcaster republishes this at high rate while NDT is calculating.
    publishMapToOdom(Eigen::Isometry3f::Identity());
    publishReady(false);
    timer_ = create_wall_timer(std::chrono::duration<double>(match_period_s_),
      std::bind(&StaticMapLocalizer::matchTimerCallback, this));

    publishStatus("等待 RViz 2D Pose Estimate；固定 PCD 已加载，不会写入地图。");
  }

private:
  struct StampedOdomBase
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Isometry3f transform{Eigen::Isometry3f::Identity()};
  };

  std::optional<StampedOdomBase> closestOdomToCloudLocked(const rclcpp::Time & cloud_stamp) const
  {
    if (cloud_stamp.nanoseconds() <= 0 || odom_to_base_history_.empty()) {
      return std::nullopt;
    }
    const auto closest = std::min_element(
      odom_to_base_history_.begin(), odom_to_base_history_.end(),
      [&cloud_stamp](const StampedOdomBase & a, const StampedOdomBase & b) {
        return std::abs((a.stamp - cloud_stamp).seconds()) <
               std::abs((b.stamp - cloud_stamp).seconds());
      });
    if (std::abs((closest->stamp - cloud_stamp).seconds()) > diagnostic_pose_max_skew_s_) {
      return std::nullopt;
    }
    return *closest;
  }

  NdtMatchDiagnosticDetail newDiagnostic(
    const uint64_t match_id, const rclcpp::Time & cloud_stamp,
    const rclcpp::Time & started_stamp) const
  {
    NdtMatchDiagnosticDetail detail;
    auto & diagnostic = detail.base;
    diagnostic.match_id = match_id;
    diagnostic.cloud_stamp = cloud_stamp;
    diagnostic.matching_started_stamp = started_stamp;
    diagnostic.fitness_score = std::numeric_limits<double>::quiet_NaN();
    diagnostic.cloud_to_odom_skew_sec = std::numeric_limits<double>::quiet_NaN();
    diagnostic.fitness_score_definition =
      "PCL NormalDistributionsTransform::getFitnessScore(): mean squared nearest-neighbor "
      "distance after the final transform; lower is better; not covariance or absolute error";
    diagnostic.score_lower_is_better = true;
    diagnostic.configured_max_fitness_score = max_fitness_score_;
    diagnostic.covariance_available = false;
    diagnostic.covariance_note =
      "Not estimated: this node does not compute an NDT covariance; unknown is not encoded as zeros.";
    detail.configured_ndt_max_iterations = ndt_max_iterations_;
    detail.configured_ndt_transformation_epsilon = ndt_transformation_epsilon_;
    detail.configured_ndt_step_size = ndt_step_size_;
    detail.configured_ndt_resolution = ndt_resolution_;
    detail.ndt_align_duration_sec = std::numeric_limits<double>::quiet_NaN();
    detail.init_to_ndt_translation_m = std::numeric_limits<double>::quiet_NaN();
    detail.init_to_ndt_yaw_rad = std::numeric_limits<double>::quiet_NaN();
    detail.nearest_neighbor_inlier_ratio = std::numeric_limits<double>::quiet_NaN();
    detail.nearest_neighbor_distance_threshold_m = diagnostic_overlap_max_distance_m_;
    detail.nearest_neighbor_residual_rms_m = std::numeric_limits<double>::quiet_NaN();
    detail.nearest_neighbor_metrics_definition =
      "Unavailable before a finite NDT result. When available: each finite post-NDT downsampled "
      "source point is assigned its nearest point in the same downsampled target cloud passed to NDT; "
      "inlier_count is nearest-neighbor distance <= nearest_neighbor_distance_threshold_m; "
      "ratio=inlier_count/evaluated_source_count; residual RMS uses all evaluated nearest-neighbor distances. "
      "This is a post-alignment diagnostic proxy, not PCL NDT internal correspondences.";
    detail.hessian_condition_available = false;
    detail.hessian_condition_number = std::numeric_limits<double>::quiet_NaN();
    detail.hessian_condition_note =
      "Unavailable: the installed PCL NormalDistributionsTransform does not expose its optimization Hessian "
      "or a condition number through public API. Unknown is not encoded as zero.";
    return detail;
  }

  void setCloudTimePoseContext(
    NdtMatchDiagnostic & diagnostic, const rclcpp::Time & cloud_stamp,
    const std::optional<StampedOdomBase> & odom_at_cloud) const
  {
    if (!odom_at_cloud.has_value()) {
      return;
    }
    diagnostic.evaluation_base_stamp = odom_at_cloud->stamp;
    diagnostic.cloud_to_odom_skew_sec = (odom_at_cloud->stamp - cloud_stamp).seconds();
    diagnostic.cloud_to_odom_sync_valid = true;
  }

  void setFinalPoseAtCloud(
    NdtMatchDiagnostic & diagnostic, const Eigen::Isometry3f & map_to_odom,
    const std::optional<StampedOdomBase> & odom_at_cloud, const std::string & source) const
  {
    if (!odom_at_cloud.has_value()) {
      diagnostic.final_pose_source = "unavailable_no_odom_near_cloud_stamp";
      return;
    }
    const Eigen::Isometry3f pose = map_to_odom * odom_at_cloud->transform;
    if (!finiteTransform(pose)) {
      diagnostic.final_pose_source = "unavailable_nonfinite_transform";
      return;
    }
    diagnostic.final_map_to_base_at_cloud = eigenToPose(pose);
    diagnostic.final_map_to_base_at_cloud_valid = true;
    diagnostic.final_pose_source = source;
  }

  void setInitialPoseAtCloud(
    NdtMatchDiagnosticDetail & detail, const Eigen::Isometry3f & map_to_odom,
    const std::optional<StampedOdomBase> & odom_at_cloud) const
  {
    if (!odom_at_cloud.has_value()) {
      detail.initial_pose_at_cloud_source = "unavailable_no_odom_near_cloud_stamp";
      return;
    }
    const Eigen::Isometry3f pose = map_to_odom * odom_at_cloud->transform;
    if (!finiteTransform(pose)) {
      detail.initial_pose_at_cloud_source = "unavailable_nonfinite_transform";
      return;
    }
    detail.initial_map_to_base_at_cloud = eigenToPose(pose);
    detail.initial_map_to_base_at_cloud_valid = true;
    detail.initial_pose_at_cloud_source = "pre_ndt_map_to_odom_times_odom_to_base_at_cloud";
  }

  void setCorrectionAtCloud(
    NdtMatchDiagnosticDetail & detail, const Eigen::Isometry3f & initial_map_to_odom,
    const Eigen::Isometry3f & candidate, const std::optional<StampedOdomBase> & odom_at_cloud) const
  {
    const Eigen::Isometry3f map_odom_correction = initial_map_to_odom.inverse() * candidate;
    if (finiteTransform(map_odom_correction)) {
      detail.map_odom_correction = eigenToPose(map_odom_correction);
      detail.map_odom_correction_valid = true;
    }
    if (!odom_at_cloud.has_value()) {
      return;
    }
    const Eigen::Isometry3f initial = initial_map_to_odom * odom_at_cloud->transform;
    const Eigen::Isometry3f ndt = candidate * odom_at_cloud->transform;
    if (!finiteTransform(initial) || !finiteTransform(ndt)) {
      return;
    }
    const Eigen::Isometry3f correction = initial.inverse() * ndt;
    if (!finiteTransform(correction)) {
      return;
    }
    detail.initial_to_ndt_base_at_cloud = eigenToPose(correction);
    detail.init_to_ndt_correction_valid = true;
    detail.init_to_ndt_translation_m = std::hypot(
      correction.translation().x(), correction.translation().y());
    detail.init_to_ndt_yaw_rad = normalizeAngle(
      std::atan2(correction.rotation()(1, 0), correction.rotation()(0, 0)));
  }

  void setNearestNeighborMetrics(
    NdtMatchDiagnosticDetail & detail, const Cloud & aligned, const Cloud::ConstPtr & target) const
  {
    if (!target || target->empty() || aligned.empty()) {
      return;
    }
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    const float distance_limit_squared = static_cast<float>(
      diagnostic_overlap_max_distance_m_ * diagnostic_overlap_max_distance_m_);
    uint32_t evaluated = 0U;
    uint32_t inliers = 0U;
    double squared_distance_sum = 0.0;
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    for (const auto & point : aligned) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (tree.nearestKSearch(point, 1, indices, squared_distances) != 1) {
        continue;
      }
      ++evaluated;
      squared_distance_sum += static_cast<double>(squared_distances[0]);
      if (squared_distances[0] <= distance_limit_squared) {
        ++inliers;
      }
    }
    if (evaluated == 0U) {
      return;
    }
    detail.nearest_neighbor_metrics_available = true;
    detail.nearest_neighbor_evaluated_source_count = evaluated;
    detail.nearest_neighbor_inlier_count = inliers;
    detail.nearest_neighbor_inlier_ratio = static_cast<double>(inliers) / evaluated;
    detail.nearest_neighbor_residual_rms_m = std::sqrt(squared_distance_sum / evaluated);
  }

  void publishDiagnostic(
    NdtMatchDiagnosticDetail & detail, const std::chrono::steady_clock::time_point & started_wall)
  {
    auto & diagnostic = detail.base;
    const auto completed = get_clock()->now();
    diagnostic.matching_completed_stamp = completed;
    if (diagnostic.cloud_stamp.sec > 0 || diagnostic.cloud_stamp.nanosec > 0) {
      diagnostic.header.stamp = diagnostic.cloud_stamp;
    } else {
      diagnostic.header.stamp = completed;
    }
    diagnostic.header.frame_id = map_frame_;
    diagnostic.computation_duration_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started_wall).count();
    ndt_diagnostic_pub_->publish(diagnostic);
    ndt_diagnostic_detail_pub_->publish(detail);
  }

  void loadStaticMap()
  {
    auto raw_map = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile<Point>(map_pcd_path_, *raw_map) != 0 || raw_map->empty()) {
      throw std::runtime_error("Cannot load static PCD map: " + map_pcd_path_);
    }
    static_map_ = voxelDownsample(raw_map, static_cast<float>(map_voxel_size_));
    if (static_map_->size() < 2U) {
      throw std::runtime_error("Static PCD has fewer than two usable points after voxel filtering: " + map_pcd_path_);
    }
    RCLCPP_INFO(get_logger(), "Loaded static PCD: raw=%zu, voxel=%zu, file=%s",
      raw_map->size(), static_map_->size(), map_pcd_path_.c_str());
  }

  void publishStaticMapCloud()
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*static_map_, message);
    message.header.stamp = get_clock()->now();
    message.header.frame_id = map_frame_;
    map_cloud_pub_->publish(message);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != raw_parent_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Ignoring cloud in frame '%s'; expected '%s'.", message->header.frame_id.c_str(), raw_parent_frame_.c_str());
      return;
    }
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*message, *cloud);
    if (cloud->empty()) {
      return;
    }
    std::scoped_lock lock(data_mutex_);
    latest_cloud_ = cloud;
    latest_cloud_stamp_ = rclcpp::Time(message->header.stamp);
  }

  void rawTfCallback(const tf2_msgs::msg::TFMessage::SharedPtr message)
  {
    for (const auto & tf : message->transforms) {
      if (tf.header.frame_id != raw_parent_frame_ || tf.child_frame_id != raw_child_frame_) {
        continue;
      }
      std::scoped_lock lock(data_mutex_);
      // FAST-LIO body is the LiDAR/IMU body.  Convert it to the chassis centre.
      odom_to_base_ = transformToEigen(tf.transform) * base_to_sensor_.inverse();
      odom_to_base_stamp_ = rclcpp::Time(tf.header.stamp);
      have_odom_to_base_ = true;
      // Keep a short timestamped history solely to express diagnostic poses at
      // the input point-cloud time.  NDT continues to use odom_to_base_ above,
      // exactly as it did before this diagnostic feature existed.
      if (odom_to_base_stamp_.nanoseconds() > 0) {
        odom_to_base_history_.push_back({odom_to_base_stamp_, odom_to_base_});
        while (odom_to_base_history_.size() > 1U &&
          (odom_to_base_history_.back().stamp - odom_to_base_history_.front().stamp).seconds() >
          diagnostic_pose_cache_s_)
        {
          odom_to_base_history_.pop_front();
        }
      }
    }
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(get_logger(), "Ignoring initial pose in '%s'; expected '%s'.",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const auto & p = message->pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite initial pose.");
      return;
    }
    {
      std::scoped_lock lock(data_mutex_);
      pending_seed_ = poseToEigen(message->pose.pose);
    }
    // map->odom is provisional until NDT accepts a registration after this seed.
    publishReady(false);
    RCLCPP_INFO(get_logger(), "Received operator seed: x=%+.2f y=%+.2f yaw=%+.1f deg.",
      p.x, p.y, yawFromQuaternion(message->pose.pose.orientation) * 180.0 / M_PI);
  }

  void matchTimerCallback()
  {
    const auto started_wall = std::chrono::steady_clock::now();
    const auto started_stamp = get_clock()->now();
    const uint64_t match_id = ++match_counter_;
    Cloud::Ptr cloud;
    rclcpp::Time cloud_stamp{0, 0, RCL_ROS_TIME};
    Eigen::Isometry3f odom_to_base;
    rclcpp::Time odom_to_base_stamp{0, 0, RCL_ROS_TIME};
    std::optional<StampedOdomBase> odom_at_cloud;
    std::optional<Eigen::Isometry3f> seed;
    Eigen::Isometry3f map_to_odom;
    bool initialized;
    bool have_odom_to_base;
    {
      std::scoped_lock lock(data_mutex_);
      cloud = latest_cloud_;
      cloud_stamp = latest_cloud_stamp_;
      odom_to_base = odom_to_base_;
      odom_to_base_stamp = odom_to_base_stamp_;
      seed = pending_seed_;
      initialized = initialized_;
      have_odom_to_base = have_odom_to_base_;
      map_to_odom = map_to_odom_;
      if (seed.has_value() && have_odom_to_base_) {
        // T_map_odom = T_map_base(seed) * inverse(T_odom_base).
        map_to_odom = *seed * odom_to_base.inverse();
        map_to_odom_ = map_to_odom;
        initialized_ = true;
        pending_seed_.reset();
        initialized = true;
        publishStatus("已接收 2D 初始位姿，开始固定地图 NDT 匹配。");
      }
      odom_at_cloud = closestOdomToCloudLocked(cloud_stamp);
    }

    auto detail = newDiagnostic(match_id, cloud_stamp, started_stamp);
    auto & diagnostic = detail.base;
    setCloudTimePoseContext(diagnostic, cloud_stamp, odom_at_cloud);
    // This is the only initial pose suitable for comparison to a ground-truth
    // pose at cloud_stamp.  The legacy field below intentionally remains
    // unchanged for backwards compatibility with existing bags.
    setInitialPoseAtCloud(detail, map_to_odom, odom_at_cloud);
    if (have_odom_to_base && finiteTransform(map_to_odom) && finiteTransform(odom_to_base)) {
      diagnostic.initial_prediction_valid = true;
      diagnostic.initial_map_to_odom = eigenToPose(map_to_odom);
      diagnostic.initial_map_to_base = eigenToPose(map_to_odom * odom_to_base);
      diagnostic.initial_odom_base_stamp = odom_to_base_stamp;
    }

    if (!initialized) {
      diagnostic.stage = "waiting_for_initialization";
      diagnostic.decision_reason = have_odom_to_base ?
        "waiting_for_operator_seed" : "waiting_for_operator_seed_and_odom_to_base";
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "not_initialized_identity_or_previous");
      publishDiagnostic(detail, started_wall);
      publishMapToOdom(Eigen::Isometry3f::Identity());
      return;
    }
    if (!cloud || cloud->size() < static_cast<size_t>(min_source_points_)) {
      diagnostic.stage = "no_input_cloud";
      diagnostic.decision_reason = !cloud ? "no_cloud_received" : "input_cloud_below_minimum";
      diagnostic.input_point_count = cloud ? static_cast<uint32_t>(cloud->size()) : 0U;
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      publishMapToOdom(map_to_odom);
      return;
    }
    diagnostic.input_point_count = static_cast<uint32_t>(cloud->size());

    const auto source = voxelDownsample(cloud, static_cast<float>(source_voxel_size_));
    diagnostic.source_point_count = static_cast<uint32_t>(source->size());
    if (source->size() < static_cast<size_t>(min_source_points_)) {
      diagnostic.stage = "source_below_minimum";
      diagnostic.decision_reason = "voxel_filtered_source_below_minimum";
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      publishMapToOdom(map_to_odom);
      return;
    }

    const Eigen::Vector3f predicted_base = (map_to_odom * odom_to_base).translation();
    auto target_raw = std::make_shared<Cloud>();
    const float radius_squared = static_cast<float>(submap_radius_ * submap_radius_);
    target_raw->reserve(static_map_->size() / 4U);
    for (const auto & point : *static_map_) {
      const float dx = point.x - predicted_base.x();
      const float dy = point.y - predicted_base.y();
      const float dz = point.z - predicted_base.z();
      if (dx * dx + dy * dy + dz * dz <= radius_squared) {
        target_raw->push_back(point);
      }
    }
    diagnostic.target_raw_point_count = static_cast<uint32_t>(target_raw->size());
    if (target_raw->size() < static_cast<size_t>(min_source_points_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Static-map submap is too small (%zu points); keep the last valid transform.", target_raw->size());
      diagnostic.stage = "submap_below_minimum";
      diagnostic.decision_reason = "target_submap_below_minimum";
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      publishMapToOdom(map_to_odom);
      return;
    }
    const auto target = voxelDownsample(target_raw, static_cast<float>(target_voxel_size_));
    diagnostic.target_point_count = static_cast<uint32_t>(target->size());

    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt.setMaximumIterations(ndt_max_iterations_);
    ndt.setInputTarget(target);
    ndt.setInputSource(source);
    Cloud aligned;
    const auto ndt_align_started = std::chrono::steady_clock::now();
    try {
      ndt.align(aligned, map_to_odom.matrix());
    } catch (const std::exception & error) {
      detail.ndt_align_duration_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - ndt_align_started).count();
      diagnostic.stage = "ndt_exception";
      diagnostic.decision_reason = std::string("ndt_exception: ") + error.what();
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      RCLCPP_ERROR(get_logger(), "NDT exception: %s", error.what());
      publishMapToOdom(map_to_odom);
      return;
    }
    detail.ndt_align_duration_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - ndt_align_started).count();

    diagnostic.converged = ndt.hasConverged();
    diagnostic.iteration_count = ndt.getFinalNumIteration();
    diagnostic.fitness_score = ndt.getFitnessScore();
    diagnostic.fitness_score_valid = std::isfinite(diagnostic.fitness_score);
    const Eigen::Isometry3f candidate(ndt.getFinalTransformation());
    if (finiteTransform(candidate)) {
      diagnostic.raw_ndt_result_valid = true;
      diagnostic.raw_ndt_map_to_odom = eigenToPose(candidate);
      if (have_odom_to_base && finiteTransform(odom_to_base)) {
        diagnostic.raw_ndt_map_to_base = eigenToPose(candidate * odom_to_base);
      }
      if (odom_at_cloud.has_value()) {
        const Eigen::Isometry3f at_cloud = candidate * odom_at_cloud->transform;
        if (finiteTransform(at_cloud)) {
          diagnostic.raw_ndt_map_to_base_at_cloud = eigenToPose(at_cloud);
          diagnostic.raw_ndt_map_to_base_at_cloud_valid = true;
        }
      }
      setCorrectionAtCloud(detail, map_to_odom, candidate, odom_at_cloud);
      setNearestNeighborMetrics(detail, aligned, target);
    }

    if (!diagnostic.converged) {
      diagnostic.stage = "ndt_not_converged";
      diagnostic.decision_reason = "ndt_has_not_converged";
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "NDT rejected: converged=%d fitness=%.3f (limit %.3f).",
        ndt.hasConverged(), ndt.getFitnessScore(), max_fitness_score_);
      publishMapToOdom(map_to_odom);
      return;
    }
    if (!diagnostic.fitness_score_valid) {
      diagnostic.stage = "invalid_fitness";
      diagnostic.decision_reason = "nonfinite_fitness_score";
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      publishMapToOdom(map_to_odom);
      return;
    }
    if (diagnostic.fitness_score > max_fitness_score_) {
      diagnostic.stage = "fitness_above_limit";
      diagnostic.decision_reason = "fitness_score_above_limit";
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "NDT rejected: converged=%d fitness=%.3f (limit %.3f).",
      ndt.hasConverged(), ndt.getFitnessScore(), max_fitness_score_);
      publishMapToOdom(map_to_odom);
      return;
    }

    const Eigen::Isometry3f update = map_to_odom.inverse() * candidate;
    const double update_xy = std::hypot(update.translation().x(), update.translation().y());
    const double update_yaw = std::abs(std::atan2(update.rotation()(1, 0), update.rotation()(0, 0)));
    diagnostic.correction_translation_m = update_xy;
    diagnostic.correction_yaw_rad = update_yaw;
    if (update_xy > max_update_translation_ || update_yaw > max_update_yaw_) {
      diagnostic.stage = update_xy > max_update_translation_ ?
        "correction_translation_above_limit" : "correction_yaw_above_limit";
      diagnostic.decision_reason = update_xy > max_update_translation_ ?
        "correction_translation_above_limit" : "correction_yaw_above_limit";
      setFinalPoseAtCloud(diagnostic, map_to_odom, odom_at_cloud, "last_accepted_before_attempt");
      publishDiagnostic(detail, started_wall);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "NDT correction rejected: %.2fm / %.1fdeg exceeds guard.",
        update_xy, update_yaw * 180.0 / M_PI);
      publishMapToOdom(map_to_odom);
      return;
    }

    {
      std::scoped_lock lock(data_mutex_);
      map_to_odom_ = candidate;
    }
    diagnostic.stage = "accepted";
    diagnostic.accepted = true;
    diagnostic.decision_reason = "accepted";
    setFinalPoseAtCloud(diagnostic, candidate, odom_at_cloud, "accepted_this_match");
    // T_final_at_cloud was stored above.  Separately retain the exact pose
    // emitted by the low-rate pose topic, whose stamp is publication time.
    const auto published_pose = publishGlobalPose(candidate * odom_to_base);
    detail.final_map_to_base_at_publish_valid = true;
    detail.final_map_to_base_at_publish = published_pose.pose;
    detail.final_pose_publish_stamp = published_pose.header.stamp;
    publishDiagnostic(detail, started_wall);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "NDT accepted: fitness=%.3f, update=%.2fm / %.1fdeg, target=%zu, source=%zu.",
      ndt.getFitnessScore(), update_xy, update_yaw * 180.0 / M_PI, target->size(), source->size());
    publishMapToOdom(candidate);
    publishReady(true);
  }

  void publishMapToOdom(const Eigen::Isometry3f & transform)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = transform.translation().x();
    tf.transform.translation.y = transform.translation().y();
    tf.transform.translation.z = transform.translation().z();
    tf.transform.rotation = eigenToQuaternion(transform.rotation());
    map_to_odom_pub_->publish(tf);
  }

  void publishOdomToCameraInit()
  {
    if (odom_frame_ == raw_parent_frame_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = raw_parent_frame_;
    tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf);
  }

  geometry_msgs::msg::PoseStamped publishGlobalPose(const Eigen::Isometry3f & map_to_base)
  {
    const auto now = get_clock()->now();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = map_frame_;
    pose.pose = eigenToPose(map_to_base);
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose.pose;
    odom_pub_->publish(odom);
    return pose;
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void publishReady(const bool ready)
  {
    if (ready_published_ && map_ready_ == ready) {
      return;
    }
    map_ready_ = ready;
    ready_published_ = true;
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "Global PCD localization ready: %s", ready ? "true" : "false");
  }

  std::string map_pcd_path_;
  std::string cloud_topic_;
  std::string raw_tf_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string raw_parent_frame_;
  std::string raw_child_frame_;
  std::string base_frame_;
  std::string ready_topic_;
  double sensor_x_{};
  double sensor_y_{};
  double sensor_z_{};
  double sensor_yaw_{};
  double map_voxel_size_{};
  double target_voxel_size_{};
  double source_voxel_size_{};
  double submap_radius_{};
  double match_period_s_{};
  double ndt_resolution_{};
  int ndt_max_iterations_{};
  double ndt_step_size_{};
  double ndt_transformation_epsilon_{};
  double max_fitness_score_{};
  double max_update_translation_{};
  double max_update_yaw_{};
  int min_source_points_{};
  double diagnostic_pose_max_skew_s_{};
  double diagnostic_pose_cache_s_{};
  double diagnostic_overlap_max_distance_m_{};

  std::mutex data_mutex_;
  Cloud::Ptr static_map_;
  Cloud::Ptr latest_cloud_;
  rclcpp::Time latest_cloud_stamp_{0, 0, RCL_ROS_TIME};
  Eigen::Isometry3f base_to_sensor_{Eigen::Isometry3f::Identity()};
  Eigen::Isometry3f odom_to_base_{Eigen::Isometry3f::Identity()};
  rclcpp::Time odom_to_base_stamp_{0, 0, RCL_ROS_TIME};
  std::deque<StampedOdomBase> odom_to_base_history_;
  Eigen::Isometry3f map_to_odom_{Eigen::Isometry3f::Identity()};
  std::optional<Eigen::Isometry3f> pending_seed_;
  bool have_odom_to_base_{false};
  bool initialized_{false};
  bool map_ready_{false};
  bool ready_published_{false};
  uint64_t match_counter_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr raw_tf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_user_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_checked_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr map_to_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<NdtMatchDiagnostic>::SharedPtr ndt_diagnostic_pub_;
  rclcpp::Publisher<NdtMatchDiagnosticDetail>::SharedPtr ndt_diagnostic_detail_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace static_map_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<static_map_localization::StaticMapLocalizer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("static_map_localizer"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
