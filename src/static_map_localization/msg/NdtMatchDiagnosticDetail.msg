# Versioned, additive diagnostic record for one fixed-map NDT attempt.
#
# ``base`` deliberately embeds the stable V1 record rather than modifying it:
# old rosbag2 files can therefore still be deserialized with
# NdtMatchDiagnostic, while new bags can use this topic for chain analysis.
NdtMatchDiagnostic base

# Exact pre-NDT prediction at the input-cloud measurement timestamp:
# T_init = T_map_odom(before NDT) * T_odom_base(near cloud_stamp).
# This is intentionally distinct from base.initial_map_to_base, whose legacy
# timestamp is the latest FAST-LIO pose when the timer callback began.
bool initial_map_to_base_at_cloud_valid
geometry_msgs/Pose initial_map_to_base_at_cloud
string initial_pose_at_cloud_source

# PCL configuration actually supplied to this attempt.  These are recorded,
# not adapted online by diagnostics.
int32 configured_ndt_max_iterations
float64 configured_ndt_transformation_epsilon
float64 configured_ndt_step_size
float64 configured_ndt_resolution

# Duration of the PCL ndt.align() call only.  base.computation_duration_sec
# remains full callback diagnostic duration, including post-alignment metrics.
float64 ndt_align_duration_sec

# T_init^-1 * T_ndt at cloud time, and the equivalent map->odom update.
# init_to_ndt_yaw_rad is signed and normalized to [-pi, pi].
bool init_to_ndt_correction_valid
geometry_msgs/Pose initial_to_ndt_base_at_cloud
float64 init_to_ndt_translation_m
float64 init_to_ndt_yaw_rad
bool map_odom_correction_valid
geometry_msgs/Pose map_odom_correction

# The actual low-rate pose publication exists only after an accepted match.
# It is recorded separately because its header time is publication time, not
# cloud_stamp; use base.final_map_to_base_at_cloud for fair ground-truth
# comparison at the measurement time.
bool final_map_to_base_at_publish_valid
geometry_msgs/Pose final_map_to_base_at_publish
builtin_interfaces/Time final_pose_publish_stamp

# Post-alignment nearest-neighbor diagnostics computed against the same
# downsampled target cloud supplied to PCL NDT.  They are diagnostic proxies,
# not PCL NDT's private Gaussian correspondence set or an NDT covariance.
bool nearest_neighbor_metrics_available
uint32 nearest_neighbor_evaluated_source_count
uint32 nearest_neighbor_inlier_count
float64 nearest_neighbor_inlier_ratio
float64 nearest_neighbor_distance_threshold_m
float64 nearest_neighbor_residual_rms_m
string nearest_neighbor_metrics_definition

# PCL's NDT Hessian/condition number is not public API in the installed PCL
# version.  It remains explicitly unavailable unless a future, independently
# validated implementation provides it; unknown numeric values are NaN.
bool hessian_condition_available
float64 hessian_condition_number
string hessian_condition_note
