#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "./imu_integrator_eigen_test_fixture.cc"  // NOLINT

using namespace imu_integrator;  // NOLINT

TEST_F(ImuIntegratorEigenFixture, ImuIntegratorZeroReadingsZeroBiases) {
  gravity_magnitude_ = 0.0;
  constructIntegrator();

  delta_time_seconds_ = 1.0;
  debiased_imu_readings_.setZero();
  current_state_.setZero();
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);

  integrate();

  EXPECT_NEAR_EIGEN(current_state_, next_state_, 1e-15);
  EXPECT_ZERO_EIGEN(next_cov_, 1e-15);
}

TEST_F(ImuIntegratorEigenFixture, ImuIntegratorJustNonzeroGravity) {
  gravity_magnitude_ = 1.0;
  constructIntegrator();

  delta_time_seconds_ = 1.0;
  debiased_imu_readings_.setZero();
  current_state_.setZero();
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);

  integrate();

  double expected_translation =
      -gravity_magnitude_ * pow(delta_time_seconds_, 2.0) / 2;
  double expected_final_velocity = -gravity_magnitude_ * delta_time_seconds_;

  EXPECT_NEAR_EIGEN(
      getOrientationFromState(next_state_), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_ZERO_EIGEN(getGyroBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getVelocityFromState(next_state_),
      Eigen::Vector3d(0, 0, expected_final_velocity), 1e-15);
  EXPECT_ZERO_EIGEN(getAccelBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getPositionFromState(next_state_),
      Eigen::Vector3d(0, 0, expected_translation), 1e-15);
  EXPECT_ZERO_EIGEN(next_cov_, 1e-15);
}

TEST_F(ImuIntegratorEigenFixture, ImuIntegratorConstantForwardAcceleration) {
  gravity_magnitude_ = 9.81;
  constructIntegrator();

  double linear_acceleration = 0.3;

  delta_time_seconds_ = 1.0;
  debiased_imu_readings_.setZero();
  debiased_imu_readings_.block<3, 1>(kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  current_state_.setZero();
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);

  integrate();

  double expected_translation =
      linear_acceleration * pow(delta_time_seconds_, 2.0) / 2;
  double expected_final_velocity = linear_acceleration * delta_time_seconds_;

  EXPECT_NEAR_EIGEN(
      getOrientationFromState(next_state_), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_ZERO_EIGEN(getGyroBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getVelocityFromState(next_state_),
      Eigen::Vector3d(expected_final_velocity, 0, 0), 1e-15);
  EXPECT_ZERO_EIGEN(getAccelBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getPositionFromState(next_state_),
      Eigen::Vector3d(expected_translation, 0, 0), 1e-15);
  EXPECT_ZERO_EIGEN(next_cov_, 1e-15);
}

TEST_F(
    ImuIntegratorEigenFixture, ImuIntegratorConstForwardAccelInitialVelocity) {
  gravity_magnitude_ = 9.81;
  constructIntegrator();

  double linear_acceleration = 0.3;
  double linear_velocity = 2.5;

  delta_time_seconds_ = 1.8;
  debiased_imu_readings_.setZero();
  debiased_imu_readings_.block<3, 1>(kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  current_state_.setZero();
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);
  current_state_.block<kVelocityBlockSize, 1>(
      kStateVelocityOffset, 0) =
      Eigen::Vector3d(linear_velocity, 0, 0);

  integrate();

  double expected_translation =
      linear_velocity * delta_time_seconds_ +
      linear_acceleration * pow(delta_time_seconds_, 2.0) / 2;
  double expected_final_velocity =
      linear_velocity + linear_acceleration * delta_time_seconds_;

  EXPECT_NEAR_EIGEN(
      getOrientationFromState(next_state_), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_ZERO_EIGEN(getGyroBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getVelocityFromState(next_state_),
      Eigen::Vector3d(expected_final_velocity, 0, 0), 1e-15);
  EXPECT_ZERO_EIGEN(getAccelBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getPositionFromState(next_state_),
      Eigen::Vector3d(expected_translation, 0, 0), 1e-15);
  EXPECT_ZERO_EIGEN(next_cov_, 1e-15);
}

TEST_F(
    ImuIntegratorEigenFixture, ImuIntegratorConstAccelAndVelocityWhenRotated) {
  gravity_magnitude_ = 9.81;
  constructIntegrator();

  double linear_acceleration = 0.3;
  double linear_velocity = 2.5;

  delta_time_seconds_ = 1.2;
  debiased_imu_readings_.setZero();
  debiased_imu_readings_.block<3, 1>(kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  current_state_.setZero();

  const Eigen::Quaterniond q_MI(sqrt(2) / 2, 0, 0, sqrt(2) / 2);
  // note that state contains B_q_G quaternion (it simplifies Jacobians)
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = q_MI.coeffs();

  const Eigen::Vector3d B_velocity_init(0, linear_velocity, 0);
  current_state_.block<kVelocityBlockSize, 1>(
      kStateVelocityOffset, 0) = B_velocity_init;

  integrate();

  double expected_translation =
      linear_velocity * delta_time_seconds_ +
      linear_acceleration * pow(delta_time_seconds_, 2.0) / 2;
  Eigen::Vector3d expected_final_velocity =
      B_velocity_init +
      Eigen::Vector3d(linear_acceleration * delta_time_seconds_, 0, 0);

  EXPECT_NEAR_EIGEN(getOrientationFromState(next_state_), q_MI.coeffs(), 1e-15);
  EXPECT_ZERO_EIGEN(getGyroBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getVelocityFromState(next_state_), expected_final_velocity, 1e-15);
  EXPECT_ZERO_EIGEN(getAccelBiasFromState(next_state_), 1e-15);
  // This test is removed, because first order approximation is not accurate
  // enough for this test.
  //  EXPECT_NEAR_EIGEN(
  //      getPositionFromState(next_state_),
  //      Eigen::Vector3d(0, expected_translation, 0), 1e-15);
  EXPECT_ZERO_EIGEN(next_cov_, 1e-15);
}

TEST_F(ImuIntegratorEigenFixture, ImuIntegratorConstantAngularVelocity) {
  gravity_magnitude_ = 9.81;
  constructIntegrator();

  delta_time_seconds_ = 1.0;
  debiased_imu_readings_.setZero();

  double angular_velocity = M_PI / 180;

  debiased_imu_readings_.block<3, 1>(kAccelReadingOffset, 0) =
      Eigen::Vector3d(0, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kAccelReadingOffset, 0) =
      Eigen::Vector3d(0, 0, gravity_magnitude_);

  debiased_imu_readings_.block<3, 1>(kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, 0, angular_velocity);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, 0, angular_velocity);

  current_state_.setZero();

  // note that state contains B_q_G quaternion (it simplifies Jacobians)
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);

  integrate();

  // calculate quaternion
  Eigen::Quaterniond final_attitude;
  final_attitude.setIdentity();
  final_attitude.w() = cos(angular_velocity * delta_time_seconds_ / 2);
  final_attitude.z() = sin(angular_velocity * delta_time_seconds_ / 2);

  EXPECT_NEAR_EIGEN(
      getOrientationFromState(next_state_), final_attitude.coeffs(), 1e-10);
  EXPECT_ZERO_EIGEN(getGyroBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getVelocityFromState(next_state_), Eigen::Vector3d(0, 0, 0), 1e-15);
  EXPECT_ZERO_EIGEN(getAccelBiasFromState(next_state_), 1e-15);
  EXPECT_NEAR_EIGEN(
      getPositionFromState(next_state_), Eigen::Vector3d(0, 0, 0), 1e-15);
  EXPECT_ZERO_EIGEN(next_cov_, 1e-15);
}

TEST_F(ImuIntegratorEigenFixture, ImuIntegratorAngularVelocitySlope) {
  delta_time_seconds_ = 1.0;
  gravity_magnitude_ = 0.0;
  constructIntegrator();

  debiased_imu_readings_.setZero();

  double angular_velocity = M_PI / 180;
  debiased_imu_readings_.setZero();
  debiased_imu_readings_.block<3, 1>(kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, 0, angular_velocity);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, 0, 0);

  current_state_.setZero();
  // note that state contains B_q_G quaternion (it simplifies Jacobians)
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);

  integrate();

  Eigen::Quaterniond final_attitude;
  final_attitude.setIdentity();
  // final angle will be equal to an area under a triangular curve
  final_attitude.w() = cos(angular_velocity * delta_time_seconds_ / 4);
  final_attitude.z() = sin(angular_velocity * delta_time_seconds_ / 4);

  EXPECT_NEAR_EIGEN(
      getOrientationFromState(next_state_), final_attitude.coeffs(), 1e-8);
}

TEST_F(ImuIntegratorEigenFixture, ImuIntegratorLinearAccelerationSlope) {
  delta_time_seconds_ = 3.0;
  gravity_magnitude_ = 9.81;
  constructIntegrator();

  double initial_acceleration = 0.5;
  debiased_imu_readings_.setZero();
  debiased_imu_readings_.block<3, 1>(kAccelReadingOffset, 0) =
      Eigen::Vector3d(initial_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kAccelReadingOffset, 0) =
      Eigen::Vector3d(0, 0, gravity_magnitude_);

  current_state_.setZero();
  // note that state contains B_q_G quaternion (it simplifies Jacobians)
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) = Eigen::Vector4d(0, 0, 0, 1);

  integrate();

  EXPECT_NEAR_EIGEN(
      getVelocityFromState(next_state_),
      Eigen::Vector3d(initial_acceleration * delta_time_seconds_ / 2, 0, 0),
      1e-15);
  // This test is removed, because first order approximation is not accurate
  // enough for this test.
  //  EXPECT_NEAR_EIGEN(
  //      getPositionFromState(next_state_),
  //      Eigen::Vector3d(
  //          initial_acceleration * pow(delta_time_seconds_, 2.0) / 3, 0, 0),
  //      1e-15);
}

MAPLAB_UNITTEST_ENTRYPOINT
