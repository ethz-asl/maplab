#include "map-optimization-legacy/ba-optimization-options.h"

#include <glog/logging.h>
#include <maplab-common/conversions.h>
#include <maplab-common/gravity-provider.h>

DEFINE_bool(
    lba_fix_ncamera_intrinsics, true,
    "Whether or not to fix the intrinsics of the ncamera(s).");
DEFINE_bool(
    lba_fix_ncamera_extrinsics_rotation, true,
    "Whether or not to fix the rotation extrinsics of the ncamera(s).");
DEFINE_bool(
    lba_fix_ncamera_extrinsics_translation, true,
    "Whether or not to fix the translation extrinsics of the ncamera(s).");
DEFINE_bool(
    lba_fix_landmark_positions, false,
    "Whether or not to fix the positions of the landmarks.");
DEFINE_bool(
    lba_fix_vertex_poses, false,
    "Whether or not to fix the positions of the vertices.");
DEFINE_bool(
    lba_fix_accel_bias, false,
    "Whether or not to fix the bias of the IMU accelerometer.");
DEFINE_bool(
    lba_fix_gyro_bias, false,
    "Whether or not to fix the bias of the IMU gyroscope.");
DEFINE_bool(
    lba_fix_velocity, false,
    "Whether or not to fix the velocity of the vertices.");
DEFINE_bool(
    lba_add_pose_prior_for_fixed_vertices, false,
    "Whether or not to add a pose prior error-term for all fixed vertices.");
DEFINE_bool(
    lba_remove_behind_camera_landmarks, true,
    "Whether or not to remove landmarks located behind the camera.");
DEFINE_bool(
    lba_fix_landmark_positions_of_fixed_vertices, false,
    "Whether or "
    "not to fix the positions of landmarks based at fixed vertices.");
DEFINE_bool(
    lba_include_wheel_odometry, false,
    "Whether or not to include wheel-odometry error-terms.");
DEFINE_bool(
    lba_include_gps, false, "Whether or not to include GPS error-terms.");
DEFINE_bool(
    lba_position_only_gps, false,
    "Whether or not to add GPS error-terms as constraints on the position only "
    "(3DoF)or as constrains on both position and orientation (6DoF)");
DEFINE_bool(
    lba_include_visual, true, "Whether or not to include visual error-terms.");
DEFINE_bool(
    lba_include_inertial, true, "Whether or not to include IMU error-terms.");
DEFINE_bool(
    lba_fix_wheel_odometry_extrinsics, true,
    "Whether or not to fix the extrinsics of the wheel-odometry.");
DEFINE_bool(
    lba_visual_outlier_rejection, false,
    "If visual outlier rejection "
    "should be done in the bundle adjustment.");
DEFINE_bool(
    lba_run_custom_post_iteration_callback_on_vi_map, true,
    "Whether or not to run a user-defined callback on the VIMap after "
    "every optimization iteration.");
DEFINE_bool(lba_show_residual_statistics, false, "Show residual statistics.");
DEFINE_bool(
    lba_apply_loss_function_for_residual_stats, false,
    "Apply loss function on residuals for the statistics.");
DEFINE_int32(
    lba_min_num_visible_landmarks_at_vertex, 5,
    "Minimum number of "
    "landmarks visible at a vertex for it to be added to the problem.");
DEFINE_int32(
    lba_update_visualization_every_n_iterations, 10,
    "Update the visualization every n optimization iterations.");
DEFINE_int32(
    lba_num_visual_outlier_rejection_loops, 2,
    "Number of bundle_adjustment visual outlier rejection loops.");
DEFINE_int32(
    lba_min_number_of_landmark_observer_missions, 0,
    "Minimum number of "
    "missions a landmark must be observed from to have its respective "
    "visual error-terms included.");
DEFINE_int32(
    lba_num_iterations, 50,
    "Maximum number of optimization iterations to perform.");
DEFINE_string(
    lba_fix_vertex_poses_except_of_mission, "",
    "Fixes all vertex "
    "poses except the vertex poses of the mission with the specified "
    "id.");
DEFINE_double(
    lba_prior_position_std_dev_meters, 0.0316227766,
    "Std. dev. in meters used for position priors. Default: sqrt(0.001)");
DEFINE_double(
    lba_prior_orientation_std_dev_radians, 0.00174533,
    "Std. dev. in radians (default"
    " corresponds to 0.1deg.) used for orientation priors.");
DEFINE_double(
    lba_prior_velocity_std_dev_meters_seconds, 0.1,
    "Std. dev. in meters per second used "
    "for priors on velocities.");
DEFINE_double(
    lba_gravity_magnitude_meters_seconds_square, -1.0,
    "Gravity magnitude in meters per "
    "square seconds. If < 0.0, the GravityProvider is used to set "
    "the gravity magnitude.");
DEFINE_bool(
    lba_add_pose_prior_on_camera_extrinsics, false,
    "Adding a prior on the pose of the camera extrinsics.");
DEFINE_double(
    lba_camera_extrinsics_position_prior_std_dev_meters, 1.0,
    "Std. dev. of the position prior on the camera extrinsics pose [meters].");
DEFINE_double(
    lba_camera_extrinsics_orientation_prior_std_dev_degrees, 1.0,
    "Std. dev. of the orientation prior on the camera extrinsics pose "
    "[meters].");
DEFINE_bool(
    lba_add_pose_prior_on_wheel_odometry_extrinsics, false,
    "Adding a prior on the pose of the wheel odometry extrinsics.");
DEFINE_double(
    lba_wheel_odometry_extrinsics_position_prior_std_dev_meters, 1.0,
    "Std. dev. of the position prior on the wheel odometry extrinsics pose "
    "[meters].");
DEFINE_double(
    lba_wheel_odometry_extrinsics_orientation_prior_std_dev_degrees, 1.0,
    "Std. dev. of the orientation prior on the wheel odometry extrinsics pose "
    "[meters].");
DEFINE_bool(
    lba_fix_wheel_odometry_extrinsics_position, false,
    "Fix the position of the wheel odometry extrinsics.");

DEFINE_bool(
    lba_include_only_merged_landmarks, false,
    "If true, only landmarks with at least two "
    "global landmark IDs are included in the optimization.");
DEFINE_bool(
    lba_fix_root_vertex, true,
    "Fix the root vertex of one of the "
    "missions for optimization.");
DEFINE_bool(lba_fix_baseframes, false, "Fix the baseframe transformations.");
DEFINE_bool(
    lba_include_loop_closure_edges, false,
    "Whether or not to include the loop-closure transformation error "
    "terms.");
DEFINE_bool(
    lba_use_switchable_constraints_for_loop_closure_edges, true,
    "If true, the optimizer may dynamically downweight loop-closure "
    "error-terms that are considered outliers by adjusting the switch "
    "variable of the error-term.");
DEFINE_double(
    lba_loop_closure_error_term_cauchy_loss, 10.0,
    "Cauchy loss parameter for the loop-closure error-terms. Note that the "
    "Cauchy loss is only used if the parameter is > 0 and the switch "
    "variable is not used.");

namespace map_optimization_legacy {

BaOptimizationOptions::BaOptimizationOptions()
    : fix_not_selected_missions(false),
      fix_ncamera_intrinsics(FLAGS_lba_fix_ncamera_intrinsics),
      fix_ncamera_extrinsics_rotation(
          FLAGS_lba_fix_ncamera_extrinsics_rotation),
      fix_ncamera_extrinsics_translation(
          FLAGS_lba_fix_ncamera_extrinsics_translation),
      fix_landmark_positions(FLAGS_lba_fix_landmark_positions),
      fix_vertex_poses(FLAGS_lba_fix_vertex_poses),
      fix_accel_bias(FLAGS_lba_fix_accel_bias),
      fix_gyro_bias(FLAGS_lba_fix_gyro_bias),
      fix_velocity(FLAGS_lba_fix_velocity),
      add_pose_prior_for_fixed_vertices(
          FLAGS_lba_add_pose_prior_for_fixed_vertices),
      remove_behind_camera_landmarks(FLAGS_lba_remove_behind_camera_landmarks),
      fix_landmark_positions_of_fixed_vertices(
          FLAGS_lba_fix_landmark_positions_of_fixed_vertices),
      include_wheel_odometry(FLAGS_lba_include_wheel_odometry),
      include_gps(FLAGS_lba_include_gps),
      position_only_gps(FLAGS_lba_position_only_gps),
      include_visual(FLAGS_lba_include_visual),
      include_inertial(FLAGS_lba_include_inertial),
      include_only_merged_landmarks(FLAGS_lba_include_only_merged_landmarks),
      fix_wheel_odometry_extrinsics(FLAGS_lba_fix_wheel_odometry_extrinsics),
      fix_root_vertex(FLAGS_lba_fix_root_vertex),
      fix_baseframes(FLAGS_lba_fix_baseframes),
      visual_outlier_rejection(FLAGS_lba_visual_outlier_rejection),
      run_custom_post_iteration_callback_on_vi_map(
          FLAGS_lba_run_custom_post_iteration_callback_on_vi_map),
      min_number_of_visible_landmarks_at_vertex(
          static_cast<size_t>(FLAGS_lba_min_num_visible_landmarks_at_vertex)),
      num_visual_outlier_rejection_loops(
          static_cast<size_t>(FLAGS_lba_num_visual_outlier_rejection_loops)),
      min_number_of_landmark_observer_missions(
          static_cast<size_t>(
              FLAGS_lba_min_number_of_landmark_observer_missions)),
      num_iterations(static_cast<size_t>(FLAGS_lba_num_iterations)),
      fix_vertex_poses_except_of_mission(
          FLAGS_lba_fix_vertex_poses_except_of_mission),
      prior_position_std_dev_meters(FLAGS_lba_prior_position_std_dev_meters),
      prior_orientation_std_dev_radians(
          FLAGS_lba_prior_orientation_std_dev_radians),
      prior_velocity_std_dev_meter_seconds(
          FLAGS_lba_prior_velocity_std_dev_meters_seconds),
      gravity_magnitude(FLAGS_lba_gravity_magnitude_meters_seconds_square),
      add_pose_prior_on_camera_extrinsics(
          FLAGS_lba_add_pose_prior_on_camera_extrinsics),
      camera_extrinsics_position_prior_std_dev_meters(
          FLAGS_lba_camera_extrinsics_position_prior_std_dev_meters),
      camera_extrinsics_orientation_prior_std_dev_radians(
          kDegToRad *
          FLAGS_lba_camera_extrinsics_orientation_prior_std_dev_degrees),
      add_pose_prior_on_wheel_odometry_extrinsics(
          FLAGS_lba_add_pose_prior_on_wheel_odometry_extrinsics),
      wheel_odometry_extrinsics_position_prior_std_dev_meters(
          FLAGS_lba_wheel_odometry_extrinsics_position_prior_std_dev_meters),
      wheel_odometry_extrinsics_orientation_prior_std_dev_radians(
          kDegToRad *
          FLAGS_lba_wheel_odometry_extrinsics_orientation_prior_std_dev_degrees),
      fix_wheel_odometry_extrinsics_position(
          FLAGS_lba_fix_wheel_odometry_extrinsics_position),
      include_loop_closure_edges(FLAGS_lba_include_loop_closure_edges),
      use_switchable_constraints_for_loop_closure_edges(
          FLAGS_lba_use_switchable_constraints_for_loop_closure_edges),
      loop_closure_error_term_cauchy_loss(
          FLAGS_lba_loop_closure_error_term_cauchy_loss) {
  CHECK_GE(FLAGS_lba_min_num_visible_landmarks_at_vertex, 0);
  CHECK_GE(FLAGS_lba_update_visualization_every_n_iterations, 0);
  CHECK_GE(FLAGS_lba_num_visual_outlier_rejection_loops, 0);
  CHECK_GE(FLAGS_lba_min_number_of_landmark_observer_missions, 0);
  CHECK_GE(FLAGS_lba_num_iterations, 0);
  if (gravity_magnitude < 0.0) {
    common::GravityProvider gravity_provider(
        common::locations::kAltitudeZurichMeters,
        common::locations::kLatitudeZurichDegrees);
    gravity_magnitude = gravity_provider.getGravityMagnitude();
  }
}

BaIterationOptions::BaIterationOptions()
    : update_visualization_every_n_iterations(
          static_cast<size_t>(
              FLAGS_lba_update_visualization_every_n_iterations)),
      show_residual_statistics(FLAGS_lba_show_residual_statistics),
      apply_loss_function_for_residual_stats(
          FLAGS_lba_apply_loss_function_for_residual_stats) {}

}  // namespace map_optimization_legacy
