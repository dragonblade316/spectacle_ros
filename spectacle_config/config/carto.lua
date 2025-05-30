include("map_builder.lua")
include("trajectory_builder.lua")

options = {
	map_builder = MAP_BUILDER,
	trajectory_builder = TRAJECTORY_BUILDER,

	map_frame = "map",
	tracking_frame = "base_link",
	published_frame = "base_link",
	odom_frame = "odom",

	provide_odom_frame = true,
	publish_frame_projected_to_2d = false,

	use_odometry = false,
	use_nav_sat = false,
	use_landmarks = false,

	num_laser_scans = 0,
	num_multi_echo_laser_scans = 0,
	num_subdivisions_per_laser_scan = 1,
	num_point_clouds = 1, -- Set to the number of 3D point cloud topics

	lookup_transform_timeout_sec = 0.2,
	submap_publish_period_sec = 0.3,
	pose_publish_period_sec = 5e-3,
	trajectory_publish_period_sec = 30e-3,

	rangefinder_sampling_ratio = 1.,
	odometry_sampling_ratio = 1.,
	fixed_frame_pose_sampling_ratio = 1.,
	imu_sampling_ratio = 1.,
	landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.use_imu_data = false

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.min_score = 0.62

return options
