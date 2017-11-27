#include <memory>
#include <random>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <ceres-error-terms/common.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "map-optimization-legacy/test/6dof-test-trajectory-gen.h"

using imu_integrator::ImuIntegratorRK4;
using imu_integrator::kErrorStateSize;
using imu_integrator::kStateSize;
using imu_integrator::kStateOrientationBlockSize;
using imu_integrator::kPositionBlockSize;
using imu_integrator::kImuReadingSize;
using imu_integrator::kGyroReadingOffset;
using imu_integrator::kStatePositionOffset;
using imu_integrator::kStateOrientationOffset;

namespace map_optimization_legacy {

class ViwlsGraph : public ::testing::Test {
 protected:
  virtual void SetUp() {
    common::GravityProvider gravity_provider(
        common::locations::kAltitudeZurichMeters,
        common::locations::kLatitudeZurichDegrees);

    // No IMU noise considered.
    settings_.imu_sigmas.acc_bias_random_walk_noise_density = 0.0;
    settings_.imu_sigmas.acc_noise_density = 0.0;
    settings_.gravity_meter_by_second2 = gravity_provider.getGravityMagnitude();
    settings_.imu_sigmas.gyro_bias_random_walk_noise_density = 0.0;
    settings_.imu_sigmas.gyro_noise_density = 0.0;
    // Seed does not matter here, we don't want noise anyways.
    settings_.imu_noise_bias_seed = 1;
    settings_.landmark_seed = 10;
    settings_.landmark_variance_meter = 3.0;
    settings_.mode = test_trajectory_gen::Path::kCircular;
    settings_.num_of_landmarks = 500;
    // 8 points for path constraints.
    settings_.num_of_path_constraints = 8;
    settings_.sampling_time_second = 0.01;
  }

  void generatePath();

  void constructIntegrator() {
    integrator_ = std::shared_ptr<ImuIntegratorRK4>(
        new ImuIntegratorRK4(
            settings_.imu_sigmas.gyro_noise_density,
            settings_.imu_sigmas.gyro_bias_random_walk_noise_density,
            settings_.imu_sigmas.acc_noise_density,
            settings_.imu_sigmas.acc_bias_random_walk_noise_density,
            settings_.gravity_meter_by_second2));
  }

  void integrate() {
    integrator_->integrate(
        current_state_, debiased_imu_readings_, settings_.sampling_time_second,
        &next_state_, &next_phi_, &next_cov_);
  }

  test_trajectory_gen::PathAndLandmarkSettings settings_;

  std::shared_ptr<ImuIntegratorRK4> integrator_;
  Eigen::Matrix<double, kStateSize, 1> current_state_;
  Eigen::Matrix<double, 2 * kImuReadingSize, 1> debiased_imu_readings_;
  Eigen::Matrix<double, kStateSize, 1> next_state_;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize> next_phi_;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize> next_cov_;

  Eigen::Matrix3Xd true_positions_;
  Eigen::Matrix4Xd true_rotations_;
  Eigen::Matrix<double, imu_integrator::kImuReadingSize, Eigen::Dynamic>
      imu_data_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void ViwlsGraph::generatePath() {
  SixDofTestTrajectoryGenerator generator(settings_);

  LOG(INFO) << "Generating trajectory, may take a while...";
  generator.generate6DofPath();
  true_positions_ = generator.getTruePositions();
  true_rotations_ = generator.getTrueRotations();
  imu_data_ = generator.getImuData();
  generator.saveTrajectory("6dof_traj.csv");
  LOG(INFO) << "Number of path points: " << true_positions_.cols();
}

// This test verifies if the data generated from 6DoF trajectory generator
// (adds additional excitation to 4DoF path from mav_planning_utils)
// is properly integrated, i.e. it is matching the provided groundtruth poses.
TEST_F(ViwlsGraph, GeneratedTrajectoryVisualBundleAdj) {
  generatePath();
  constructIntegrator();

  current_state_.setZero();
  // Note that state contains B_q_G quaternion (it simplifies Jacobians).
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = true_rotations_.col(0);
  current_state_.block<kPositionBlockSize, 1>(kStatePositionOffset, 0) =
      true_positions_.col(0);

  for (int i = 0; i < (imu_data_.cols() - 1); ++i) {
    debiased_imu_readings_.block<kImuReadingSize, 1>(0, 0) = imu_data_.col(i);
    debiased_imu_readings_.block<kImuReadingSize, 1>(kImuReadingSize, 0) =
        imu_data_.col(i + 1);

    integrate();
    current_state_ = next_state_;

    Eigen::Vector3d position = current_state_.block<3, 1>(13, 0);
    Eigen::Quaterniond B_q_G_curr(current_state_.block<4, 1>(0, 0).data());
    Eigen::Quaterniond B_q_G_gt(true_rotations_.col(i + 1).data());
    EXPECT_NEAR_EIGEN(position, true_positions_.col(i + 1), 1e-5);
    EXPECT_NEAR_KINDR_QUATERNION(
        pose::Quaternion(B_q_G_curr), pose::Quaternion(B_q_G_gt), 1e-5);
  }
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
