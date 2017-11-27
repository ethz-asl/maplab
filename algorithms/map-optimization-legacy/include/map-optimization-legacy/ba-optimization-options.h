#ifndef MAP_OPTIMIZATION_LEGACY_BA_OPTIMIZATION_OPTIONS_H_
#define MAP_OPTIMIZATION_LEGACY_BA_OPTIMIZATION_OPTIONS_H_

#include <cstddef>
#include <string>

#include <vi-map/unique-id.h>

namespace map_optimization_legacy {

struct BaOptimizationOptions {
  BaOptimizationOptions();
  virtual ~BaOptimizationOptions() = default;

  vi_map::MissionIdSet mission_ids_to_optimize;
  bool fix_not_selected_missions;  // If set to false, the
                                   // mission_ids_to_optimize will be selected
  // and the other mission data won't be changed in the
  // optimization.  If set to true, the missions not in
  // mission_ids_to_optimize will be fixed in the optimization.
  bool fix_ncamera_intrinsics;
  bool fix_ncamera_extrinsics_rotation;
  bool fix_ncamera_extrinsics_translation;
  bool fix_landmark_positions;
  bool fix_vertex_poses;
  bool fix_accel_bias;
  bool fix_gyro_bias;
  bool fix_velocity;
  bool add_pose_prior_for_fixed_vertices;
  bool remove_behind_camera_landmarks;
  bool fix_landmark_positions_of_fixed_vertices;
  bool include_wheel_odometry;
  bool include_gps;
  bool position_only_gps;
  bool include_visual;
  bool include_inertial;
  bool include_only_merged_landmarks;
  bool fix_wheel_odometry_extrinsics;
  bool fix_root_vertex;
  bool fix_baseframes;
  bool visual_outlier_rejection;
  bool run_custom_post_iteration_callback_on_vi_map;
  size_t min_number_of_visible_landmarks_at_vertex;
  size_t num_visual_outlier_rejection_loops;
  size_t min_number_of_landmark_observer_missions;
  size_t num_iterations;
  std::string fix_vertex_poses_except_of_mission;
  double prior_position_std_dev_meters;
  double prior_orientation_std_dev_radians;
  double prior_velocity_std_dev_meter_seconds;
  double gravity_magnitude;
  bool add_pose_prior_on_camera_extrinsics;
  double camera_extrinsics_position_prior_std_dev_meters;
  double camera_extrinsics_orientation_prior_std_dev_radians;
  bool add_pose_prior_on_wheel_odometry_extrinsics;
  double wheel_odometry_extrinsics_position_prior_std_dev_meters;
  double wheel_odometry_extrinsics_orientation_prior_std_dev_radians;
  bool fix_wheel_odometry_extrinsics_position;
  bool include_loop_closure_edges;
  bool use_switchable_constraints_for_loop_closure_edges;
  double loop_closure_error_term_cauchy_loss;
};

struct BaIterationOptions {
  BaIterationOptions();
  virtual ~BaIterationOptions() = default;
  size_t update_visualization_every_n_iterations;
  bool show_residual_statistics;
  bool apply_loss_function_for_residual_stats;
};

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_BA_OPTIMIZATION_OPTIONS_H_
