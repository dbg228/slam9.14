# One record is published for every scheduled fixed-map localization attempt,
# including skipped, failed, non-converged, and rejected attempts.
#
# Time semantics:
# - cloud_stamp is the header stamp of the /cloud_registered input used here.
# - matching_* stamps are ROS clock times at callback execution, NOT measurement
#   times.  They must not be used to pair this result with ground truth.
# - *_at_cloud poses are valid only when a raw FAST-LIO odom->base_link pose
#   close enough to cloud_stamp was available.  They are the poses intended for
#   offline error evaluation.

std_msgs/Header header
uint64 match_id
builtin_interfaces/Time cloud_stamp
builtin_interfaces/Time matching_started_stamp
builtin_interfaces/Time matching_completed_stamp
builtin_interfaces/Time initial_odom_base_stamp
builtin_interfaces/Time evaluation_base_stamp
float64 computation_duration_sec

# stage is one of: waiting_for_initialization, no_input_cloud,
# source_below_minimum, submap_below_minimum, ndt_exception,
# ndt_not_converged, invalid_fitness, fitness_above_limit,
# correction_translation_above_limit, correction_yaw_above_limit, accepted.
string stage

bool initial_prediction_valid
geometry_msgs/Pose initial_map_to_odom
geometry_msgs/Pose initial_map_to_base

# PCL NDT's direct output.  The transform maps the local FAST-LIO odom
# (camera_init) frame into map; it is present even when the result is later
# rejected, unless NDT threw an exception.
bool raw_ndt_result_valid
geometry_msgs/Pose raw_ndt_map_to_odom
geometry_msgs/Pose raw_ndt_map_to_base
bool raw_ndt_map_to_base_at_cloud_valid
geometry_msgs/Pose raw_ndt_map_to_base_at_cloud

# The map->odom correction which remains active after this attempt, evaluated
# at the input-cloud time when an odom->base pose is available.  It equals the
# raw candidate for accepted attempts and the preceding accepted correction for
# rejected attempts.
bool final_map_to_base_at_cloud_valid
geometry_msgs/Pose final_map_to_base_at_cloud
string final_pose_source

# PCL NormalDistributionsTransform::getFitnessScore(): mean squared nearest-
# neighbor distance for the final transform.  Smaller is better.  It is not a
# covariance and is not an absolute localization error.
bool fitness_score_valid
float64 fitness_score
string fitness_score_definition
bool score_lower_is_better
bool converged
int32 iteration_count
bool accepted
string decision_reason

uint32 input_point_count
uint32 source_point_count
uint32 target_raw_point_count
uint32 target_point_count
float64 configured_max_fitness_score
float64 correction_translation_m
float64 correction_yaw_rad

# No covariance is estimated by this node.  Unknown covariance is represented
# by covariance_available=false, never by an all-zero matrix.
bool covariance_available
string covariance_note

# Absolute difference between the cached odom->base timestamp used for the
# *_at_cloud poses and cloud_stamp.  It is NaN when no cached pose exists.
float64 cloud_to_odom_skew_sec
bool cloud_to_odom_sync_valid
