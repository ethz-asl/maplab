#ifndef SIMULATION_GENERIC_PATH_GENERATOR_DEMO_H_
#define SIMULATION_GENERIC_PATH_GENERATOR_DEMO_H_

#include <gflags/gflags.h>

DECLARE_double(tg_circle_radius_meter);
DECLARE_double(tg_distance_to_keypoints_meter);
DECLARE_double(tg_kappa);
DECLARE_double(tg_lambda);
DECLARE_double(tg_landmark_offset_x_meter);
DECLARE_double(tg_landmark_offset_y_meter);
DECLARE_double(tg_landmark_offset_z_meter);
DECLARE_double(tg_landmark_variance_meter);
DECLARE_double(tg_landmark_volume_x_meter);
DECLARE_double(tg_landmark_volume_y_meter);
DECLARE_double(tg_landmark_volume_z_meter);
DECLARE_double(tg_number_of_rounds);
DECLARE_double(tg_sampling_time_second);
DECLARE_double(tg_start_offset_x_meter);
DECLARE_double(tg_start_offset_y_meter);
DECLARE_double(tg_start_offset_yaw_radians);
DECLARE_double(tg_start_offset_z_meter);
DECLARE_double(tg_vertical_to_radial_variance_factor);
DECLARE_int32(tg_imu_noise_bias_seed);
DECLARE_int32(tg_landmark_seed);
DECLARE_int32(tg_mode);
DECLARE_int32(tg_num_of_path_constraints);
DECLARE_int32(tg_num_of_landmarks);
DECLARE_string(tg_filename_waypoints);
DECLARE_string(tg_visualization_topic);

#endif  // SIMULATION_GENERIC_PATH_GENERATOR_DEMO_H_
