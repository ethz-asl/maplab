#include "simulation/generic-path-generator.h"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <maplab-common/gravity-provider.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <simulation/eigen-visualization.h>

// Basic usage examples:
// rosrun simulation simulation_generic // Generates circular trajectory with
// default parameters.
// rosrun simulation simulation_generic -mode 1 // Same as previous line.
// rosrun simulation simulation_generic -mode 2 // Generates elliptic trajectory
// with default parameters.
// rosrun simulation simulation_generic -mode 2 -kappa 0.1 -lambda 0.2
// rosrun simulation simulation_generic -mode 3 // Generate rotation-only path
// rosrun simulation simulation_generic -mode 4 // Generate translation-only
// path
// rosrun simulation simulation_generic -mode 5 -filename_waypoints
// "/tmp/waypoints_translation_only.txt"
// rosrun simulation simulation_generic -mode 5 -filename_waypoints
// "/tmp/waypoints_rotation_only.txt"

// gflags: DEFINE_type(name, default value, help)
// Prefix tg_* stands for "trajectory generator".
DEFINE_bool(tg_visualization, true, "Publish visualization topics for RVIZ.");
DEFINE_double(
    tg_acc_bias_sigma_meter_by_sqrt_second5, 0.1,
    "Accelerometer noise bias standard deviation");
DEFINE_double(
    tg_acc_sigma_meter_by_sqrt_second2, 0.1,
    "Accelerometer noise standard deviation");
DEFINE_double(tg_circle_radius_meter, 10.0, "Circle radius");
DEFINE_double(tg_distance_to_keypoints_meter, 5.0, "Distance to keypoints");
DEFINE_double(
    tg_gyro_bias_sigma_rad_by_sqrt_second3, 0.1,
    "Gyroscope bias noise standard deviation");
DEFINE_double(
    tg_gyro_sigma_rad_by_sqrt_second, 0.1,
    "Gyroscope noise standard deviation");
DEFINE_double(
    tg_kappa, 0.1,
    "Kappa defining the ellipse shape according to x = (1.0 / kappa) "
    "* cos(t).");
DEFINE_double(
    tg_lambda, 0.05,
    "Lambda defining the ellipse shape according to y = (1.0 / "
    "lambda) * sin(t).");
DEFINE_double(
    tg_landmark_offset_x_meter, 50.0, "Landmark volume offset in x direction");
DEFINE_double(
    tg_landmark_offset_y_meter, 0.0, "Landmark volume offset in y direction");
DEFINE_double(
    tg_landmark_offset_z_meter, 0.5, "Landmark volume offset in z direction");
DEFINE_double(tg_landmark_variance_meter, 3.0, "Landmark variance");
DEFINE_double(
    tg_landmark_volume_x_meter, 10.0,
    "Landmark volume in x direction, centered around offset");
DEFINE_double(
    tg_landmark_volume_y_meter, 10.0,
    "Landmark volume in y direction, centered around offset");
DEFINE_double(
    tg_landmark_volume_z_meter, 5.0,
    "Landmark volume in z direction, centered around offset");
DEFINE_double(tg_num_of_rounds, 1.0, "Number of rounds");
DEFINE_double(tg_sampling_time_second, 0.1, "Sampling time");
DEFINE_double(
    tg_start_offset_x_meter, 0.0, "Offset from the origin in x direction");
DEFINE_double(
    tg_start_offset_y_meter, 0.0, "Offset from the origin in y direction");
DEFINE_double(tg_start_offset_yaw_radians, 0.0, "Yaw start offset");
DEFINE_double(
    tg_start_offset_z_meter, 0.0,
    "Offset from the origin in z direction. This is the constant height.");
DEFINE_double(
    tg_vertical_to_radial_variance_factor, 2.0,
    "Ratio between radial and vertical standard deviation of the "
    "keypoint positions.");
DEFINE_int32(tg_imu_noise_bias_seed, 10, "IMU noise bias seed");
DEFINE_int32(tg_landmark_seed, 5, "Landmark seed");
DEFINE_int32(
    tg_mode, 1,
    "Trajectory mode: [1] circle, [2] ellipse, [3] rotation, [4] translation, "
    "[5] from file");
DEFINE_int32(tg_num_of_path_constraints, 8, "Number of path constraints");
DEFINE_int32(tg_sub_sampling_factor, 5, "Sub-sampling factor");
DEFINE_int32(tg_num_of_landmarks, 1000, "Number of landmarks");
DEFINE_string(
    tg_filename_waypoints, "waypoints_translation_only.txt",
    "List with waypoints: x y z yaw");
DEFINE_string(
    tg_visualization_frame_id, "/map",
    "Visualization frame id of the simulation");
DEFINE_string(
    tg_visualization_topic, "path", "Visualization topic of the simulation");

void insertMarkers(
    const visualization_msgs::MarkerArray& to_insert,
    visualization_msgs::MarkerArray* m) {
  CHECK_NOTNULL(m);
  m->markers.insert(
      m->markers.end(), to_insert.markers.begin(), to_insert.markers.end());
}

void setUp(test_trajectory_gen::PathAndLandmarkSettings* settings) {
  CHECK_NOTNULL(settings);
  // Retrieve Zurich's gravity.
  common::GravityProvider gravity_provider(
      common::locations::kAltitudeZurichMeters,
      common::locations::kLatitudeZurichDegrees);
  // Accelerometer noise bias standard deviation.
  settings->imu_sigmas.acc_bias_random_walk_noise_density =
      FLAGS_tg_acc_bias_sigma_meter_by_sqrt_second5;
  // Accelerometer noise standard deviation.
  settings->imu_sigmas.acc_noise_density =
      FLAGS_tg_acc_sigma_meter_by_sqrt_second2;
  // Circle radius.
  settings->circle_radius_meter = FLAGS_tg_circle_radius_meter;
  // Distance to keypoints.
  settings->distance_to_keypoints_meter = FLAGS_tg_distance_to_keypoints_meter;
  // Gravity magnitude.
  settings->gravity_meter_by_second2 = gravity_provider.getGravityMagnitude();
  // Gyroscope bias noise standard deviation.
  settings->imu_sigmas.gyro_bias_random_walk_noise_density =
      FLAGS_tg_gyro_bias_sigma_rad_by_sqrt_second3;
  // Gyroscope noise standard deviation.
  settings->imu_sigmas.gyro_noise_density =
      FLAGS_tg_gyro_sigma_rad_by_sqrt_second;
  // Kappa defining the ellipse shape according to
  // x = (1.0 / kappa) * cos(t).
  settings->kappa = FLAGS_tg_kappa;
  // Lambda defining the ellipse shape according to
  // y = (1.0 / lambda) * sin(t).
  settings->lambda = FLAGS_tg_lambda;
  // Landmark volume offset in x direction.
  settings->landmark_offset_x_meter = FLAGS_tg_landmark_offset_x_meter;
  // Landmark volume offset in y direction.
  settings->landmark_offset_y_meter = FLAGS_tg_landmark_offset_y_meter;
  // Landmark volume offset in z direction.
  settings->landmark_offset_z_meter = FLAGS_tg_landmark_offset_z_meter;
  // Landmark variance.
  settings->landmark_variance_meter = FLAGS_tg_landmark_variance_meter;
  // Landmark volume in x direction, centered around offset.
  settings->landmark_volume_x_meter = FLAGS_tg_landmark_volume_x_meter;
  // Landmark volume in y direction, centered around offset.
  settings->landmark_volume_y_meter = FLAGS_tg_landmark_volume_y_meter;
  // Landmark volume in z direction, centered around offset.
  settings->landmark_volume_z_meter = FLAGS_tg_landmark_volume_z_meter;
  // Number of rounds. Currently only applies for elliptical mode.
  settings->num_of_rounds = FLAGS_tg_num_of_rounds;
  // Sampling time in seconds.
  settings->sampling_time_second = FLAGS_tg_sampling_time_second;
  // Offset from the origin in x direction.
  settings->start_offset_x_meter = FLAGS_tg_start_offset_x_meter;
  // Offset from the origin in y direction.
  settings->start_offset_y_meter = FLAGS_tg_start_offset_y_meter;
  // Yaw start offset.
  settings->start_offset_yaw_radians = FLAGS_tg_start_offset_yaw_radians;
  // Offset from the origin in z direction. This is the constant height.
  settings->start_offset_z_meter = FLAGS_tg_start_offset_z_meter;
  // Ratio between radial and vertical standard deviation of the
  // keypoint positions.
  settings->vertical_to_radial_variance_factor =
      FLAGS_tg_vertical_to_radial_variance_factor;
  // IMU noise bias seed.
  settings->imu_noise_bias_seed =
      static_cast<size_t>(FLAGS_tg_imu_noise_bias_seed);
  // Landmark seed.
  settings->landmark_seed = static_cast<size_t>(FLAGS_tg_landmark_seed);
  // Trajectory mode.
  settings->mode = static_cast<test_trajectory_gen::Path>(FLAGS_tg_mode);
  // Number of path constraints.
  settings->num_of_path_constraints =
      static_cast<size_t>(FLAGS_tg_num_of_path_constraints);
  // Number of landmarks.
  settings->num_of_landmarks = static_cast<size_t>(FLAGS_tg_num_of_landmarks);
  // List with waypoints: x y z yaw.
  settings->filename_waypoints = FLAGS_tg_filename_waypoints;
}

int main(int argc, char** argv) {
  // Initialize google logging and flags.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Initialize ros node handle and visualization publisher.
  ros::init(argc, argv, "path_generator_demo");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(
      FLAGS_tg_visualization_topic, 1, true);

  // Define settings based on gflags.
  test_trajectory_gen::PathAndLandmarkSettings settings;
  setUp(&settings);

  // **************************************************************************
  //  Generating path and incorporating sensor biases.
  // **************************************************************************
  test_trajectory_gen::GenericPathGenerator path_generator(settings);
  path_generator.generatePath();
  path_generator.generateLandmarks();

  const mav_planning_utils::Motion4D<5, 2>::Vector& data =
      path_generator.getPathData();
  const Eigen::Matrix3Xd& landmarks = path_generator.getLandmarks();

  ROS_INFO("Generated %lu points", data.size());

  // **************************************************************************
  //  Visualization of the generated path and landmarks.
  // **************************************************************************
  if (FLAGS_tg_visualization) {
    visualization_msgs::MarkerArray markers, markers_tmp;
    std_msgs::Header header;
    header.frame_id = FLAGS_tg_visualization_frame_id;
    header.stamp = ros::Time::now();
    header.seq = 0;

    mav_planning_utils::MavState s;
    for (size_t i = 0; i < data.size(); i += FLAGS_tg_sub_sampling_factor) {
      path_generator.motionVectorToMavState(data[i], &s);
      mav_viz::drawAxesArrows(markers_tmp, s.p, s.q, 0.3);
      insertMarkers(markers_tmp, &markers);

      visualization_msgs::Marker marker;
      mav_viz::drawArrow(
          marker, s.p, s.p + s.a, mav_viz::createColorRGBA(1, 1, 0, 1));
      markers.markers.push_back(marker);

      mav_viz::drawArrow(
          marker, s.p, s.p + s.v, mav_viz::createColorRGBA(0, 1, 1, 1));
      markers.markers.push_back(marker);
    }

    for (int i = 0; i < landmarks.cols(); ++i) {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color = mav_viz::createColorRGBA(0.8, 0.5, 0.2, 1);

      geometry_msgs::Point point;
      point.x = landmarks(0, i);
      point.y = landmarks(1, i);
      point.z = landmarks(2, i);
      marker.points.push_back(point);

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      markers.markers.push_back(marker);
    }

    int cnt = 0;
    for (visualization_msgs::MarkerArray::_markers_type::iterator it =
             markers.markers.begin();
         it != markers.markers.end(); ++it) {
      it->action = visualization_msgs::Marker::ADD;
      it->header = header;
      it->id = cnt;
      it->ns = FLAGS_tg_visualization_topic;
      ++cnt;
    }
    pub.publish(markers);
  }

  while (ros::ok()) {
    usleep(1e5);
  }
}
