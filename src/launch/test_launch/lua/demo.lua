
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_to_tf = false,
  publish_tracked_pose = true,
}

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 9999
POSE_GRAPH.constraint_builder.max_constraint_distance = 0.001
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 12
POSE_GRAPH.constraint_builder.min_score = 0.52
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.min_z = 0.15

POSE_GRAPH.optimization_problem.odometry_translation_weight = 1000
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 10

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1

POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 30.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window =  math.rad(45.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 30.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)

return options
