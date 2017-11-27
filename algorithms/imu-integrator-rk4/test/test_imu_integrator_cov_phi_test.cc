#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "./imu_integrator_test_fixture.cc"  // NOLINT

using namespace imu_integrator;  // NOLINT

TEST_F(PosegraphErrorTerms, ImuIntegratorPhiTest) {
  gravity_magnitude_ = 9.81;
  constructIntegrator();

  double linear_acceleration = 0.3;
  double linear_velocity = 2.5;
  double angular_velocity = 0.2;

  delta_time_seconds_ = 1.2;
  debiased_imu_readings_.setZero();
  debiased_imu_readings_.block<3, 1>(kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, angular_velocity, 0);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, angular_velocity, 0);

  current_state_.setZero();

  // note that state contains B_q_G quaternion (it simplifies Jacobians)
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) =
      Eigen::Vector4d(0, 0, -sqrt(2) / 2, sqrt(2) / 2);
  current_state_.block<kVelocityBlockSize, 1>(
      kStateVelocityOffset, 0) =
      Eigen::Vector3d(0, linear_velocity, 0);

  integrate();

  Eigen::Matrix<double, kStateSize, 1> expected_state;
  expected_state << 0.0846492581372881, 0.0846492581372881, -0.702021716305672,
      0.702021716305672, 0, 0, 0, 8.65973959207622e-17, 0.737576694265446,
      -0.155699400194541, 0, 0, 0, 4.79616346638068e-17, 2.22150140337671,
      -0.0510759941610040;

  EXPECT_NEAR_EIGEN(expected_state, next_state_, 1e-12);

  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize>
      expected_phi;
  expected_phi << 0.971338240000000, 0, -0.237696000000000, -1.18848000000000,
      0, 0.143308800000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      -1.20000000000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.237696000000000, 0,
      0.971338240000000, -0.143308800000000, 0, -1.18848000000000, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      -11.6159961600000, 2.83173484660892e-15, 1.76240332800000,
      7.01201664000000, -1.82973636242423e-15, -0.780019200000000, 1, 0, 0,
      -2.88657986402541e-16, -1.20000000000000, 0, 0, 0, 0,
      -2.79551933601851e-15, -11.6163005998055, 4.49518864797938e-16,
      1.81460734438588e-15, 6.92743671392755, -1.81077552952047e-16, 0, 1, 0,
      1.18851106684286, -2.88657986402541e-16, 0.143309886409959, 0, 0, 0, 0,
      -1.76242330573455, 0, 0, 1.33640937025818, 0, 0, 0, 1, 0.143309886409959,
      0, -1.18851106684286, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, -7.01201664000000, 1.56834545350648e-15,
      0.780019200000000, 2.82009600000000, -6.27338181402592e-16,
      -0.255916800000000, 1.20000000000000, 0, 0, -1.59872115546023e-16,
      -0.720000000000000, 0, 1, 0, 0, -1.56074449364496e-15, -7.01212400583899,
      1.73314163021132e-16, 6.26762641786627e-16, 2.79463440350340,
      -3.80047993076005e-17, 0, 1.20000000000000, 0, 0.716550198505571,
      -1.59872115546023e-16, 0.0574448050022037, 0, 1, 0, 0, -0.778498596623289,
      0, 0, 0.423899157973974, 0, 0, 0, 1.20000000000000, 0.0574448050022037, 0,
      -0.716550198505571, 0, 0, 1;
  EXPECT_NEAR_EIGEN(expected_phi, next_phi_, 1e-12);
}

TEST_F(PosegraphErrorTerms, ImuIntegratorCovarianceTest) {
  gravity_magnitude_ = 9.81;

  gyro_noise_sigma_ = 0.1;
  gyro_bias_sigma_ = 0.2;
  acc_noise_sigma_ = 0.3;
  acc_bias_sigma_ = 0.4;

  constructIntegrator();

  double linear_acceleration = 0.3;
  double linear_velocity = 2.5;
  double angular_velocity = 0.2;

  delta_time_seconds_ = 1.2;
  debiased_imu_readings_.setZero();
  debiased_imu_readings_.block<3, 1>(kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kAccelReadingOffset, 0) =
      Eigen::Vector3d(linear_acceleration, 0, gravity_magnitude_);
  debiased_imu_readings_.block<3, 1>(kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, angular_velocity, 0);
  debiased_imu_readings_.block<3, 1>(
      kImuReadingSize + kGyroReadingOffset, 0) =
      Eigen::Vector3d(0, angular_velocity, 0);

  current_state_.setZero();

  // note that state contains B_q_G quaternion (it simplifies Jacobians)
  current_state_.block<kStateOrientationBlockSize, 1>(
      kStateOrientationOffset, 0) =
      Eigen::Vector4d(0, 0, -sqrt(2) / 2, sqrt(2) / 2);
  current_state_.block<kVelocityBlockSize, 1>(
      kStateVelocityOffset, 0) =
      Eigen::Vector3d(0, linear_velocity, 0);

  integrate();

  Eigen::Matrix<double, kStateSize, 1> expected_state;
  expected_state << 0.0846492581372881, 0.0846492581372881, -0.702021716305672,
      0.702021716305672, 0, 0, 0, 8.65973959207622e-17, 0.737576694265446,
      -0.155699400194541, 0, 0, 0, 4.79616346638068e-17, 2.22150140337671,
      -0.0510759941610040;
  EXPECT_NEAR_EIGEN(expected_state, next_state_, 1e-12);

  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize>
      expected_cov;
  expected_cov << 0.0350400000000000, 0, 0, -0.0286617600000000, 0,
      0.00230400000000000, -0.172175846400000, -4.45710156782297e-17, 0, 0, 0,
      0, -0.0283564800000000, -6.30791419098387e-18, 0, 0, 0.0350400000000000,
      0, 0, -0.0288000000000000, 0, 4.46455672431512e-17, -0.168177157094506,
      -0.0365959324713338, 0, 0, 0, 6.27338181402592e-18, -0.0279463440350340,
      -0.00423899157973974, 0, 0, 0.0350400000000000, -0.00230400000000000, 0,
      -0.0286617600000000, -0.000390528000000002, 1.07178266262053e-19, 0, 0, 0,
      0, -0.00252633600000000, -9.37362187869439e-19, 0, -0.0286617600000000, 0,
      -0.00230400000000000, 0.0480000000000000, 0, 0, 0.112803840000000,
      2.50244625021878e-17, 0, 0, 0, 0, 0.0339033600000000,
      3.76402908841556e-18, 0, 0, -0.0288000000000000, 0, 0, 0.0480000000000000,
      0, -2.50935272561037e-17, 0.110364878264592, 0.0236018727140161, 0, 0, 0,
      -3.76402908841556e-18, 0.0335350639622800, 0.00509040848806728,
      0.00230400000000000, 0, -0.0286617600000000, 0, 0, 0.0480000000000000,
      -0.0102366720000000, -3.02580360767024e-18, 0, 0, 0, 0,
      -0.00103680000000000, -1.15107923193136e-19, 0, -0.172175846400000,
      4.46455672431512e-17, -0.000390528000000002, 0.112803840000000,
      -2.50935272561037e-17, -0.0102366720000000, 0.754998336000000,
      2.04021926416624e-18, -2.30360143102141e-17, -2.98427949019242e-17,
      -0.115200000000000, 0, 0.355949251200000, 8.38393207317152e-19,
      -1.09798817476696e-17, -4.45710156782297e-17, -0.168177157094506,
      1.07178266262053e-19, 2.50244625021878e-17, 0.110364878264592,
      -3.02580360767024e-18, 2.04021926416624e-18, 0.735296192760905,
      0.0978658447999381, 0.113546093072937, -2.98427949019242e-17,
      0.0183243293903595, 1.63368530092706e-18, 0.348169068658769,
      0.0350392741650491, 0, -0.0365959324713338, 0, 0, 0.0236018727140161, 0,
      -2.30360143102141e-17, 0.0978658447999381, 0.217201164592677,
      0.0183243293903595, 0, -0.113546093072937, -1.47816605123911e-17,
      0.0483439085961552, 0.113251017709422, 0, 0, 0, 0, 0, 0,
      -2.98427949019242e-17, 0.113546093072937, 0.0183243293903595,
      0.192000000000000, 0, 0, -1.02318153949454e-17, 0.0457488190565349,
      0.00551470128021155, 0, 0, 0, 0, 0, 0, -0.115200000000000,
      -2.98427949019242e-17, 0, 0, 0.192000000000000, 0, -0.0460800000000000,
      -1.02318153949454e-17, 0, 0, 0, 0, 0, 0, 0, 0, 0.0183243293903595,
      -0.113546093072937, 0, 0, 0.192000000000000, 0, 0.00551470128021155,
      -0.0457488190565349, -0.0283564800000000, 6.27338181402592e-18,
      -0.00252633600000000, 0.0339033600000000, -3.76402908841556e-18,
      -0.00103680000000000, 0.355949251200000, 1.63368530092706e-18,
      -1.47816605123911e-17, -1.02318153949454e-17, -0.0460800000000000, 0,
      0.0518400000000000, 0, 0, -6.30791419098387e-18, -0.0279463440350340,
      -9.37362187869439e-19, 3.76402908841556e-18, 0.0335350639622800,
      -1.15107923193136e-19, 8.38393207317152e-19, 0.348169068658769,
      0.0483439085961552, 0.0457488190565349, -1.02318153949454e-17,
      0.00551470128021155, 0, 0.0518400000000000, 0, 0, -0.00423899157973974, 0,
      0, 0.00509040848806728, 0, -1.09798817476696e-17, 0.0350392741650491,
      0.113251017709422, 0.00551470128021155, 0, -0.0457488190565349, 0, 0,
      0.0518400000000000;
  EXPECT_NEAR_EIGEN(expected_cov, next_cov_, 1e-12);
}

MAPLAB_UNITTEST_ENTRYPOINT
