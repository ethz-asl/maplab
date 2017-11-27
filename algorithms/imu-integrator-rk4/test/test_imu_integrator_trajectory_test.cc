#include <fstream>
#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <simulation/generic-path-generator.h>

#include "imu-integrator/imu-integrator.h"

using namespace imu_integrator;  // NOLINT

double getGravityMagnitude() {
  return 9.81;
}

class PosegraphErrorTerms : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    settings_.imu_sigmas.acc_bias_random_walk_noise_density = 0;
    settings_.imu_sigmas.acc_noise_density = 0;
    settings_.gravity_meter_by_second2 = getGravityMagnitude();
    settings_.imu_sigmas.gyro_bias_random_walk_noise_density = 0;
    settings_.imu_sigmas.gyro_noise_density = 0;
    settings_.imu_noise_bias_seed = 10;
    settings_.mode = test_trajectory_gen::Path::kCircular;
    settings_.num_of_path_constraints = 8;
    settings_.sampling_time_second = 0.001;
    settings_.gravity_meter_by_second2 = getGravityMagnitude();
    delta_time_seconds_ = 0.0;
  }

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
        current_state_, debiased_imu_readings_, delta_time_seconds_,
        &next_state_, &next_phi_, &next_cov_);
  }

  std::shared_ptr<ImuIntegratorRK4> integrator_;

  // Struct contains settings for imu integrator and path generator.
  test_trajectory_gen::PathAndLandmarkSettings settings_;

  // Imu integrator integrate() arguments.
  Eigen::Matrix<double, kStateSize, 1> current_state_;
  Eigen::Matrix<double, 2 * kImuReadingSize, 1>
      debiased_imu_readings_;
  Eigen::Matrix<double, kStateSize, 1> next_state_;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize>
      next_phi_;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize>
      next_cov_;
  double delta_time_seconds_;
};

TEST_F(PosegraphErrorTerms, ImuIntegratorTrajectory4d) {
  test_trajectory_gen::GenericPathGenerator path_generator(settings_);
  path_generator.generatePath();

  const Eigen::Matrix4Xd& true_G_q_B = path_generator.getTrueRotations();
  const Eigen::Matrix3Xd& true_positions = path_generator.getTruePositions();
  const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data =
      path_generator.getImuData();

  constructIntegrator();

  current_state_.setZero();
  // Please note that state contains B_q_G quaternion (it simplifies Jacobians).
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0)
      << 0,
      0, 0, 1;
  current_state_.block<kPositionBlockSize, 1>(
      kStatePositionOffset, 0)
      << 10,
      0, 0;

  delta_time_seconds_ = settings_.sampling_time_second;

  std::ofstream traj_file;
  traj_file.open("traj.csv");

  for (int i = 0; i < (imu_data.cols() - 1); ++i) {
    debiased_imu_readings_.block<kImuReadingSize, 1>(0, 0) =
        imu_data.col(i);
    debiased_imu_readings_.block<kImuReadingSize, 1>(
        kImuReadingSize, 0) = imu_data.col(i + 1);

    integrate();

    current_state_ = next_state_;

    Eigen::Vector3d position = current_state_.block<3, 1>(13, 0);
    Eigen::Quaterniond B_q_G(current_state_.block<4, 1>(0, 0).data());
    Eigen::Quaterniond G_q_B(true_G_q_B.col(i + 1).data());
    EXPECT_NEAR_EIGEN(position, true_positions.col(i + 1), 1e-5);
    EXPECT_NEAR_KINDR_QUATERNION(
        pose::Quaternion(B_q_G), pose::Quaternion(G_q_B.inverse()), 1e-6);

    traj_file << current_state_(13, 0) << ", " << current_state_(14, 0) << ", "
              << current_state_(15, 0) << ", " << true_positions(0, i + 1)
              << ", " << true_positions(1, i + 1) << ", "
              << true_positions(2, i + 1) << std::endl;
  }
  traj_file.close();
}

MAPLAB_UNITTEST_ENTRYPOINT
