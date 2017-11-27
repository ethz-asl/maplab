#include <fstream>   // NOLINT
#include <iostream>  // NOLINT
#include <memory>
#include <sstream>

#include <glog/logging.h>
#include <mav_planning_utils/mav_state.h>
#include <mav_planning_utils/path_planning.h>
#include <mav_planning_utils/path_serialization.h>

#include "simulation/generic-path-generator.h"

namespace test_trajectory_gen {
void GenericPathGenerator::generatePath() {
  CHECK_GT(settings_.num_of_path_constraints, 0u)
      << "Negative or zero point count.";
  CHECK_GT(settings_.kappa, 0) << "Negative or zero kappa.";
  CHECK_GT(settings_.lambda, 0) << "Negative or zero lambda.";
  CHECK_GT(settings_.circle_radius_meter, 0)
      << "Negative or zero circle radius.";
  CHECK_GE(settings_.imu_sigmas.acc_noise_density, 0)
      << "Negative standard deviation (Accelerometer).";
  CHECK_GE(settings_.imu_sigmas.gyro_noise_density, 0)
      << "Negative standard deviation (Gyroscope).";
  CHECK_GE(settings_.num_of_path_constraints, 2u)
      << "Number of waypoints is smaller than two.";
  CHECK_GT(settings_.num_of_rounds, 0) << "Non-positive number of path rounds";
  CHECK_GT(settings_.imu_noise_bias_seed, 0u)
      << "Negative imu noise bias seed.";

  // Generate the trajectory path constraints.
  // TODO(hitimo): More elegant way (e.g. OOP) to select type of path wanted?
  // TODO(hitimo): Random path generator wanted?
  const Eigen::Vector4d start_offset(
      settings_.start_offset_x_meter, settings_.start_offset_y_meter,
      settings_.start_offset_z_meter, settings_.start_offset_yaw_radians);
  switch (settings_.mode) {
    case Path::kCircular:
      generatePathConstraintsCircle(
          settings_.circle_radius_meter, settings_.num_of_path_constraints,
          start_offset);
      LOG(INFO) << "Mode 1: Create circular path.";
      break;
    case Path::kElliptical:
      generatePathConstraintsEllipse(
          settings_.kappa, settings_.lambda, settings_.num_of_rounds,
          settings_.num_of_path_constraints, start_offset);
      LOG(INFO) << "Mode 2: Create elliptical path.";
      break;
    case Path::kRotationOnly:
      generatePathConstraintsRotationOnly();
      LOG(INFO) << "Mode 3: Create rotation-only path.";
      break;
    case Path::kTranslationOnly:
      generatePathConstraintsTranslationOnly();
      LOG(INFO) << "Mode 4: Create translation-only path.";
      break;
    case Path::kFromFile:
      generatePathConstraintsFromFile(settings_.filename_waypoints);
      LOG(INFO) << "Mode 5: Create path from waypoints (loaded from file).";
      break;
    case Path::kFromCtor:
      generatePathConstraintsFromCtor(settings_.pose_waypoints);
      LOG(INFO) << "Mode 6: Create path from waypoints (passed to ctor).";
      break;
    default:
      LOG(ERROR) << "Wrong path mode: mode " << settings_.mode
                 << " requested but needs to be " << Path::kCircular << ", "
                 << Path::kElliptical << ", " << Path::kRotationOnly << ", "
                 << Path::kTranslationOnly << ", " << Path::kFromFile << " or "
                 << Path::kFromCtor << ".";
      return;
  }

  // Insert the generated path constraints.
  mav_planning_utils::path_planning::Vertex1D max_position(
      kTimeToNext, kDerivativeToOptimizeVelocity);
  mav_planning_utils::path_planning::Vertex1D max_yaw(
      kTimeToNext, kDerivativeToOptimizeVelocity);
  insertPathConstraints(&max_position, &max_yaw);

  // Optimize the path based on all constraints.
  mav_planning_utils::path_planning::Path4D<12> path;

  // Read the actual Path4D object from a file. This fixes the problem of
  // inverting a large matrix which was done in the deprecated
  // mav_planning_utils package.
  readPath4dFromFile(settings_.mode, &path);

  // The code below illustrates how to write the Path4D object to a file.
  // path.optimizeWithTime(sv_, kContinuity, max_position, max_yaw);
  // writePath4dToFile(settings_.mode, path);

  // Downsampling of the generated trajectory by the sampling time.
  path.sample<5, 2>(
      path_data_, settings_.sampling_time_second, &timestamps_seconds_);
  // This is randomized.
  this->motionVectorToImuData(settings_.imu_noise_bias_seed);
  is_path_generated_ = true;
}

void GenericPathGenerator::generateLandmarks() {
  CHECK_GE(settings_.landmark_variance_meter, 0)
      << "Variance must not be negative";
  CHECK_GT(settings_.num_of_landmarks, 0u) << "Negative or zero point count.";
  CHECK_GE(settings_.distance_to_keypoints_meter, 0)
      << "Non-positive distance to keypoints.";
  CHECK_GT(settings_.vertical_to_radial_variance_factor, 0)
      << "Negative vertical to radial variance factor.";
  CHECK_GT(settings_.imu_noise_bias_seed, 0u) << "Negative landmark bias seed.";

  // Generate the landmarks based on the trajectory.
  // TODO(hitimo): More elegant way (e.g. OOP) to select type of path wanted?
  const Eigen::Vector3d start_offset_meter(
      settings_.start_offset_x_meter, settings_.start_offset_y_meter,
      settings_.start_offset_z_meter);
  const Eigen::Vector3d landmark_offset_meter(
      settings_.landmark_offset_x_meter, settings_.landmark_offset_y_meter,
      settings_.landmark_offset_z_meter);
  const Eigen::Vector3d landmark_volume_meter(
      settings_.landmark_volume_x_meter, settings_.landmark_volume_y_meter,
      settings_.landmark_volume_z_meter);
  switch (settings_.mode) {
    case Path::kCircular:      // Fallthrough intended.
    case Path::kRotationOnly:  // Fallthrough intended.
      // kCircular and kRotationOnly.
      generateLandmarksCircle(
          settings_.circle_radius_meter, settings_.distance_to_keypoints_meter,
          settings_.landmark_variance_meter,
          settings_.vertical_to_radial_variance_factor,
          settings_.num_of_landmarks, settings_.landmark_seed,
          start_offset_meter);
      break;
    case Path::kElliptical:
      // Path::kElliptical.
      generateLandmarksEllipse(
          settings_.distance_to_keypoints_meter, settings_.kappa,
          settings_.lambda, settings_.landmark_variance_meter,
          settings_.num_of_rounds, settings_.vertical_to_radial_variance_factor,
          settings_.num_of_landmarks, settings_.landmark_seed,
          start_offset_meter);
      break;
    case Path::kTranslationOnly:  // Fallthrough intended.
    case Path::kFromFile:         // Fallthrough intended.
    case Path::kFromCtor:         // Fallthrough intended.
      // Path::kTranslationOnly, Path::kFromFile and Path::kFromCtor.
      generateLandmarksVolume(
          landmark_offset_meter, landmark_volume_meter,
          settings_.landmark_variance_meter, settings_.num_of_landmarks,
          settings_.landmark_seed);
      break;
    default:
      LOG(ERROR) << "Wrong path mode: mode " << settings_.mode
                 << " requested but needs to be " << Path::kCircular << ", "
                 << Path::kElliptical << ", " << Path::kRotationOnly << ", "
                 << Path::kTranslationOnly << ", " << Path::kFromFile << " or "
                 << Path::kFromCtor << ".";
      return;
  }
  are_landmarks_generated_ = true;
}

// Generate position constraints based on the circle equation:
// x = r * cos(alpha)
// y = r * sin(alpha)
void GenericPathGenerator::generatePathConstraintsCircle(
    const double circle_radius_meter, const size_t num_of_path_constraints,
    const Eigen::Vector4d start_offset) {
  // Insert start point at (radius,0,0,0) + offset.
  double x_coord = start_offset(0) + circle_radius_meter;
  double y_coord = start_offset(1);
  double z_coord = start_offset(2);
  double yaw = start_offset(3);
  // The "pose" consists of x, y, z (in meter) and yaw (in radian).
  pose_waypoints_.emplace_back(x_coord, y_coord, z_coord, yaw);

  // Insert inner points.
  const double angle_step =
      2.0 * M_PI / (static_cast<double>(num_of_path_constraints) - 1.0);
  // Two path constraints for start and end point, the remaining for in between.
  for (size_t i = 0; i < num_of_path_constraints - 2; ++i) {
    yaw += angle_step;
    x_coord = start_offset(0) + circle_radius_meter * cos(yaw);
    y_coord = start_offset(1) + circle_radius_meter * sin(yaw);
    z_coord = start_offset(2);
    pose_waypoints_.emplace_back(x_coord, y_coord, z_coord, yaw);
  }

  // Insert end point at (radius,0,0,0) + offset.
  x_coord = start_offset(0) + circle_radius_meter;
  y_coord = start_offset(1);
  z_coord = start_offset(2), yaw = start_offset(3) + 2.0 * M_PI;
  pose_waypoints_.emplace_back(x_coord, y_coord, z_coord, yaw);
}

// Generate position constraints based on the ellipse equation:
// x = (1.0 / kappa ) * cos (t)
// y = (1.0 / lambda ) * sin (t)
void GenericPathGenerator::generatePathConstraintsEllipse(
    const double kappa, const double lambda, const double num_of_rounds,
    const size_t num_of_path_constraints, const Eigen::Vector4d start_offset) {
  // Insert start point.
  double x_coord = start_offset(0) + (1.0 / kappa) * cos(0.0);
  double y_coord = start_offset(1) + (1.0 / lambda) * sin(0.0);
  double z_coord = start_offset(2);
  double yaw = start_offset(3);
  // The "pose" consists of x, y, z (in meter) and yaw (in radian).
  pose_waypoints_.emplace_back(x_coord, y_coord, z_coord, yaw);

  // Insert inner points.
  const double angle_step =
      2.0 * M_PI / (static_cast<double>(num_of_path_constraints) - 1.0);
  // Two path constraints for start and end point, the remaining for in between.
  for (size_t i = 0; i < num_of_path_constraints - 2; ++i) {
    double t = num_of_rounds * 2.0 * M_PI * (i + 1.0) / num_of_path_constraints;
    yaw += angle_step;
    x_coord = start_offset(0) + (1.0 / kappa) * cos(t);
    y_coord = start_offset(1) + (1.0 / lambda) * sin(t);
    z_coord = start_offset(2);
    pose_waypoints_.emplace_back(x_coord, y_coord, z_coord, yaw);
  }

  // Insert end point.
  x_coord = start_offset(0) + (1.0 / kappa) * cos(num_of_rounds * 2.0 * M_PI);
  y_coord = start_offset(1) + (1.0 / lambda) * sin(num_of_rounds * 2.0 * M_PI);
  z_coord = start_offset(2);
  yaw = start_offset(3) + 2.0 * M_PI;
  pose_waypoints_.emplace_back(x_coord, y_coord, z_coord, yaw);
}

// Load position constraints from file.
void GenericPathGenerator::generatePathConstraintsRotationOnly() {
  CHECK(pose_waypoints_.empty());
  // The "pose" consists of x, y, z (in meter) and yaw (in radian).
  for (size_t i = 0; i < 10; ++i) {
    pose_waypoints_.emplace_back(0.0, 0.0, 0.0, 2.0 * M_PI / 10.0 * i);
  }
}

void GenericPathGenerator::generatePathConstraintsTranslationOnly() {
  CHECK(pose_waypoints_.empty());
  // The "pose" consists of x, y, z (in meter) and yaw (in radian).
  for (size_t i = 0; i < 10; ++i) {
    pose_waypoints_.emplace_back(i, 0.0, 0.0, 0.0);
  }
}

void GenericPathGenerator::generatePathConstraintsFromCtor(
    const Aligned<std::vector, Eigen::Vector4d>& pose_waypoints) {
  CHECK(pose_waypoints_.empty());
  CHECK(!pose_waypoints.empty());
  CHECK_GE(pose_waypoints.size(), 2u)
      << "Number of waypoints is smaller than two.";
  pose_waypoints_ = pose_waypoints;
}

// Load position constraints from file.
void GenericPathGenerator::generatePathConstraintsFromFile(
    const std::string& filename) {
  std::ifstream infile(filename);
  // Make sure the unit test does not fail if no file is found...
  if (!infile.is_open()) {
    LOG(ERROR) << "Tried to read waypoints from: " << filename
               << " but the file does not exist!";
    LOG(WARNING) << "Creating a simple translation-only example instead.";
    generatePathConstraintsTranslationOnly();
  }
  std::string line;
  LOG(INFO) << "Reading waypoints from: " << filename << ".";
  while (std::getline(infile, line)) {
    std::vector<double> p;
    double val;
    std::istringstream lineStream(line);
    while (lineStream >> val) {
      p.push_back(val);
    }
    CHECK_EQ(p.size(), 4u) << "Wrong file format: 'X Y Z YAW' (double, 4 "
                              "columns, space-separated) expected";
    pose_waypoints_.emplace_back(p[0], p[1], p[2], p[3]);
  }
  LOG(INFO) << "Loaded the following waypoints from file ("
            << pose_waypoints_.size() << " waypoints):";
  for (size_t i = 0; i < pose_waypoints_.size(); ++i) {
    LOG(INFO) << "i=" << i << ": " << pose_waypoints_[i].transpose();
  }
  CHECK_GE(pose_waypoints_.size(), 2u)
      << "Number of waypoints is smaller than two.";
}

// Insert position constraints.
void GenericPathGenerator::insertPathConstraints(
    mav_planning_utils::path_planning::Vertex1D* max_position,
    mav_planning_utils::path_planning::Vertex1D* max_yaw) {
  CHECK_NOTNULL(max_position);
  CHECK_NOTNULL(max_yaw);
  // Insert first waypoint as a position constraint.
  sv_.push_back(
      mav_planning_utils::path_planning::Vertex4D(
          kTimeToNext, kDerivativeToOptimizeAcceleration, kDerivatives));
  sv_.back().addConstraint(kConstraintPosition, pose_waypoints_[0]);

  // Insert all waypoints in between first and last waypoint as position
  // constraints.
  if (pose_waypoints_.size() > 2) {
    for (size_t i = 1; i < pose_waypoints_.size() - 1; ++i) {
      sv_.push_back(
          mav_planning_utils::path_planning::Vertex4D(
              kTimeToNext, kDerivativeToOptimizeAcceleration));
      sv_.back().addConstraint(kConstraintPosition, pose_waypoints_[i]);
    }
  }

  // Insert last waypoint as a position constraint.
  sv_.push_back(
      mav_planning_utils::path_planning::Vertex4D(
          kTimeToNext, kDerivativeToOptimizeAcceleration, kDerivatives));
  sv_.back().addConstraint(kConstraintPosition, pose_waypoints_.back());

  // Insert velocity constraints.
  max_position->addConstraint(kConstraintVelocity, kConstraintValuePosition);

  // Insert yaw rate constraints.
  max_yaw->addConstraint(kConstraintVelocity, kConstraintValueYaw);
}

void GenericPathGenerator::generateLandmarksCircle(
    const double circle_radius_meter, const double distance_to_keypoints_meter,
    const double landmark_variance_meter,
    const double vertical_to_radial_variance_factor,
    const size_t num_of_landmarks, const size_t landmark_seed,
    const Eigen::Vector3d start_offset_meter) {
  std::mt19937 gen(landmark_seed);
  std::uniform_real_distribution<> dis(0, 1);
  G_landmarks_.resize(3, num_of_landmarks);
  for (size_t i = 0; i < num_of_landmarks; ++i) {
    double random_angle = dis(gen) * 2 * M_PI;
    double radius_noise = landmark_variance_meter * (dis(gen) - 0.5);
    double vertical_noise = vertical_to_radial_variance_factor *
                            landmark_variance_meter * (dis(gen) - 0.5);

    double x_coord =
        start_offset_meter(0) +
        (circle_radius_meter + distance_to_keypoints_meter + radius_noise) *
            cos(random_angle);
    double y_coord =
        start_offset_meter(1) +
        (circle_radius_meter + distance_to_keypoints_meter + radius_noise) *
            sin(random_angle);
    double z_coord = start_offset_meter(2) + vertical_noise;

    G_landmarks_.col(i) = Eigen::Vector3d(x_coord, y_coord, z_coord);
  }
}

void GenericPathGenerator::generateLandmarksEllipse(
    const double distance_to_keypoints_meter, const double kappa,
    const double lambda, const double landmark_variance_meter,
    const double num_of_rounds, const double vertical_to_radial_variance_factor,
    const size_t num_of_landmarks, const size_t landmark_seed,
    const Eigen::Vector3d start_offset_meter) {
  std::mt19937 gen(landmark_seed);
  std::uniform_real_distribution<> dis(0, 1);
  G_landmarks_.resize(3, num_of_landmarks);
  for (size_t i = 0; i < num_of_landmarks; ++i) {
    double random_angle = num_of_rounds * dis(gen) * 2.0 * M_PI;
    double radius_noise = landmark_variance_meter * (dis(gen) - 0.5);
    double vertical_noise = vertical_to_radial_variance_factor *
                            landmark_variance_meter * (dis(gen) - 0.5);

    double x_coord =
        start_offset_meter(0) +
        (1.0 / kappa + distance_to_keypoints_meter + radius_noise) *
            cos(random_angle);
    double y_coord =
        start_offset_meter(1) +
        (1.0 / lambda + distance_to_keypoints_meter + radius_noise) *
            sin(random_angle);
    double z_coord = start_offset_meter(2) + vertical_noise;

    G_landmarks_.col(i) = Eigen::Vector3d(x_coord, y_coord, z_coord);
  }
}

void GenericPathGenerator::generateLandmarksVolume(
    Eigen::Vector3d landmark_offset_meter,
    Eigen::Vector3d landmark_volume_meter, const double landmark_variance_meter,
    const size_t num_of_landmarks, const size_t landmark_seed) {
  std::mt19937 gen(landmark_seed);
  std::uniform_real_distribution<> dis(0, 1);
  G_landmarks_.resize(3, num_of_landmarks);
  // Volume contains all the landmarks for a variance of 1.0m (or below).
  for (size_t i = 0; i < num_of_landmarks; ++i) {
    double x_coord =
        landmark_variance_meter * (landmark_volume_meter(0) * (dis(gen) - 0.5) +
                                   landmark_offset_meter(0));
    double y_coord =
        landmark_variance_meter * (landmark_volume_meter(1) * (dis(gen) - 0.5) +
                                   landmark_offset_meter(1));
    double z_coord =
        landmark_variance_meter * (landmark_volume_meter(2) * (dis(gen) - 0.5) +
                                   landmark_offset_meter(2));
    G_landmarks_.col(i) = Eigen::Vector3d(x_coord, y_coord, z_coord);
  }
}
}  // namespace test_trajectory_gen
