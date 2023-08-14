/*
 * ***********************************************************************
 * Copyright (c) 2021, by Cyngn. All rights reserved.
 *
 * Proprietary and confidential.
 *
 * ***********************************************************************
 */

#pragma once

#include <array>
#include <iostream>
#include <vector>

#include "common.h"

namespace cyngn::localization {

struct PreprocessingParams {
  std::array<double, 3> crop_min_distances = {0.0, 0.0, 0.0};
  std::array<double, 3> crop_max_distances = {100.0, 100.0, 100.0};

  bool use_close_point_filter = true;
  bool use_far_point_filter = true;
};

struct FeatureParams {
  double feature_curv_threshold;
  int largest_picked_num_threshold;

  std::array<size_t, 2> curvature_calc_range_right;
  std::array<size_t, 2> curvature_calc_range_left;

  double partial_curvature_threshold_right;
  double partial_curvature_threshold_left;

  uint32_t min_num_ring_points;
  uint32_t num_sectors;
};

struct PubSubParams {
  std::vector<std::string> pointcloud_sub_topic_name;
  std::string se_pose_topic_name;

  std::string imu_sub_topic_name;
  std::string output_topic_name;
  std::string initial_pose_guess_topic_name;

  uint32_t pointcloud_sub_queue_size;
  uint32_t imu_sub_queue_size;
  uint32_t pc_imu_sub_queue_size;
};

struct LidarOdometryParams {
  // lidar model, see above
  LidarID lidar_id;

  std::array<float, 6> covariance{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  // leaf sizes for voxel grid subsample filter
  // {x, y, z}
  std::array<float, 3> surf_leaf{-1.0f, -1.0f, -1.0f};
  std::array<float, 3> edge_leaf{-1.0f, -1.0f, -1.0f};

  unsigned int max_optimization_iter = 0;          // started at 12, annealed to 2
  unsigned int nominal_optimization_iter = 0;      // 2 in original code
  unsigned int max_nominal_optimization_iter = 0;  // 5 default
  double huber_loss_param = 0.0;                   // 0.1 in original code
  unsigned int solver_max_iter = 0;                // 4 in original code
  double gradient_check_precision = 0.0;           // 1e-4 in original code
  double max_final_cost = 0.0;                     // maximum fixed cost threshold from ceres

  unsigned int min_surf_cost_factor_count = 0;  // 20 in original code
  unsigned int min_edge_cost_factor_count = 0;  // 20 in original code

  unsigned int min_surf_map_size = 0;  // 50 from original implementation
  unsigned int min_edge_map_size = 0;  // 10 from original implementation

  // for outgoing ros message
  std::string map_frame_id = "";      // "/map"
  std::string odom_frame_id = "";     // "/odom"
  std::string vehicle_frame_id = "";  // "/baselink"
  std::string lidar_frame_id = "";

  bool enable_diagnostic_flag = false;
  bool show_odometry_delay_flag = false;
  uint32_t optimization_option_flag = 0;
  bool undistort_lidar = false;
  bool debug_flag = false;
  bool jump_detect = true;
  double pose_diff_x_thresh = 0.05;
  double pose_diff_y_thresh = 0.05;
  double pose_diff_yaw_thresh = 0.02;

  float k_feature_dist_thresh;       // Nearest neighbor search threshold, unit (meter)
  unsigned int k_nearest_neighbors;  // Number of nearest neighbor search return

  bool enable_state_logging = false;
  std::string state_logging_path;
};

struct FeatureMapManagerParams {
  std::string feature_map_path = "";
  std::string location_id = "";
  std::string surf_map_file_name = "";
  std::string edge_map_file_name = "";

  bool surf_down_sample = false;
  bool edge_down_sample = false;

  // leaf sizes for downsample filter
  float surf_leaf_x = -1.0f;
  float surf_leaf_y = -1.0f;
  float surf_leaf_z = -1.0f;

  float edge_leaf_x = -1.0f;
  float edge_leaf_y = -1.0f;
  float edge_leaf_z = -1.0f;
};

struct BootstrapParams {
  float max_time_threshold = 600;  // ms
  int min_surf_matches = 0;
  int min_edge_matches = 0;
  uint32_t min_consecutive_odom_success_frame_count = 0;
  float roll_max = 0;
  float pitch_max = 0;

  // initial location given bootstrapping
  // {x, y, z, roll, pitch, yaw}
  std::array<float, 6> initial_pose_guess{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

struct LidarOdometryDiagnosticConfig {
  float bootstrap_latency_warning_thresh;
  std::unordered_map<std::string, float> lidar_odom_success_percentage_thresh;
  float lidar_latency_warning_thresh;
  float optimization_latency_warning_thresh;
  float process_latency_warning_thresh;
};

struct CyngnLocalizerBridgeDiagnosticParams {
  std::unordered_map<std::string, float> input_odom_map_latency;
  std::unordered_map<std::string, float> input_odom_odom_latency;
  std::unordered_map<std::string, float> input_accel_latency;
  std::unordered_map<std::string, float> translation_covariance;
  std::unordered_map<std::string, float> yaw_covariance;
};

using CloudCurvature = std::vector<Double2d>;

}  // namespace cyngn::localization
