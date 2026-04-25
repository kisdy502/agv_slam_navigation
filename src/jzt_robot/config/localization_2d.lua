include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 核心坐标系
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,  --仿真，用1,真机雷达畸变，改大一些
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 激光雷达配置
TRAJECTORY_BUILDER_2D = {
  use_online_correlative_scan_matching = true, -- 是否启用相关匹配
  use_imu_data = true,  -- 🔴 在2D builder中也明确启用
  imu_gravity_time_constant = 10.,  
  ceres_scan_matcher = {
    occupied_space_weight = 1.0,    -- 占据空间残差权重
    translation_weight = 10.0,      -- 平移正则权重
    rotation_weight = 40.0,         -- 旋转正则权重
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,      -- 最大迭代次数
      num_threads = 1,              -- 求解线程数
    }
  },
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,     -- 线性搜索窗(m)
    angular_search_window = math.rad(1.), -- 角度搜索窗(rad)
    translation_delta_cost_weight = 1e-1, -- 平移代价权重
    rotation_delta_cost_weight = 1e-1,    -- 旋转代价权重
  }
}


-- POSE图优化配置
POSE_GRAPH.optimize_every_n_nodes = 15  -- 每15个节点优化一次
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

return options
