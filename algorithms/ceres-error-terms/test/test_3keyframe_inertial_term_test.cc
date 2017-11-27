#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ceres-error-terms/common.h>
#include <ceres-error-terms/inertial-error-term.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <imu-integrator/imu-integrator.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

using namespace ceres_error_terms;  // NOLINT

class PosegraphErrorTerms : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    rot0_.coeffs() << 0, 0, sqrt(2) / 2, sqrt(2) / 2;
    pos0_ << 0, 0, 0;
    velocity0_ << 0, 2, 0;

    common::GravityProvider gravity_provider(
        common::locations::kAltitudeZurichMeters,
        common::locations::kLatitudeZurichDegrees);
    gravity_magnitude_ = gravity_provider.getGravityMagnitude();

    // Use integrator to obtain ground truth states for vertices.
    imu_integrator::ImuIntegratorRK4 integrator(0, 0, 0, 0, gravity_magnitude_);

    Eigen::Matrix<double, imu_integrator::kStateSize, 1> current_state;
    Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
        debiased_imu_readings;
    Eigen::Matrix<double, imu_integrator::kStateSize, 1> next_state;
    Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                  imu_integrator::kErrorStateSize>
        next_phi;
    Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                  imu_integrator::kErrorStateSize>
        next_cov;

    current_state.setZero();
    current_state.block<4, 1>(imu_integrator::kStateOrientationOffset, 0) =
        rot0_.coeffs();
    current_state.block<3, 1>(imu_integrator::kStateVelocityOffset, 0) =
        velocity0_;
    current_state.block<3, 1>(imu_integrator::kStatePositionOffset, 0) = pos0_;

    debiased_imu_readings << 3, 0, gravity_magnitude_, 0, 0.2, 0, 3, 0,
        gravity_magnitude_, 0, 0.2, 0;

    const double delta_time_seconds = 2.0;
    integrator.integrate(
        current_state, debiased_imu_readings, delta_time_seconds, &next_state,
        &next_phi, &next_cov);

    rot1_.coeffs() =
        next_state.block<4, 1>(imu_integrator::kStateOrientationOffset, 0);
    velocity1_ =
        next_state.block<3, 1>(imu_integrator::kStateVelocityOffset, 0);
    pos1_ = next_state.block<3, 1>(imu_integrator::kStatePositionOffset, 0);

    current_state.swap(next_state);
    integrator.integrate(
        current_state, debiased_imu_readings, delta_time_seconds, &next_state,
        &next_phi, &next_cov);

    rot2_.coeffs() =
        next_state.block<4, 1>(imu_integrator::kStateOrientationOffset, 0);
    velocity2_ =
        next_state.block<3, 1>(imu_integrator::kStateVelocityOffset, 0);
    pos2_ = next_state.block<3, 1>(imu_integrator::kStatePositionOffset, 0);

    accel_bias0_.setZero();
    accel_bias1_.setZero();
    accel_bias2_.setZero();
    gyro_bias0_.setZero();
    gyro_bias1_.setZero();
    gyro_bias2_.setZero();

    imu_timestamps_01_ << 0, delta_time_seconds * 1e9;
    imu_data_01_ << 3, 3, 0, 0, gravity_magnitude_, gravity_magnitude_, 0, 0,
        0.2, 0.2, 0, 0;

    imu_timestamps_12_ << delta_time_seconds * 1e9,
        2 * delta_time_seconds * 1e9;
    imu_data_12_ << 3, 3, 0, 0, gravity_magnitude_, gravity_magnitude_, 0, 0,
        0.2, 0.2, 0, 0;
  }

  void addResidual();
  void solve();

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;

  Eigen::Matrix<int64_t, 1, 2> imu_timestamps_01_;
  Eigen::Matrix<double, 6, 2> imu_data_01_;

  Eigen::Matrix<int64_t, 1, 2> imu_timestamps_12_;
  Eigen::Matrix<double, 6, 2> imu_data_12_;

  Eigen::Quaterniond rot0_;
  Eigen::Quaterniond rot1_;
  Eigen::Quaterniond rot2_;
  Eigen::Vector3d pos0_;
  Eigen::Vector3d pos1_;
  Eigen::Vector3d pos2_;
  Eigen::Matrix<double, 7, 1> pose0_;
  Eigen::Matrix<double, 7, 1> pose1_;
  Eigen::Matrix<double, 7, 1> pose2_;

  Eigen::Vector3d accel_bias0_;
  Eigen::Vector3d accel_bias1_;
  Eigen::Vector3d accel_bias2_;
  Eigen::Vector3d gyro_bias0_;
  Eigen::Vector3d gyro_bias1_;
  Eigen::Vector3d gyro_bias2_;
  Eigen::Vector3d velocity0_;
  Eigen::Vector3d velocity1_;
  Eigen::Vector3d velocity2_;

  double gravity_magnitude_;
};

void PosegraphErrorTerms::addResidual() {
  rot0_.normalize();
  rot1_.normalize();
  rot2_.normalize();
  pose0_ << rot0_.coeffs(), pos0_;
  pose1_ << rot1_.coeffs(), pos1_;
  pose2_ << rot2_.coeffs(), pos2_;

  ceres::CostFunction* inertial_term_cost01 =
      new ceres_error_terms::InertialErrorTerm(
          imu_data_01_, imu_timestamps_01_, 1, 1, 1, 1, gravity_magnitude_);

  problem_.AddResidualBlock(
      inertial_term_cost01, NULL, pose0_.data(), gyro_bias0_.data(),
      velocity0_.data(), accel_bias0_.data(), pose1_.data(), gyro_bias1_.data(),
      velocity1_.data(), accel_bias1_.data());

  ceres::CostFunction* inertial_term_cost12 =
      new ceres_error_terms::InertialErrorTerm(
          imu_data_12_, imu_timestamps_12_, 1, 1, 1, 1, gravity_magnitude_);

  problem_.AddResidualBlock(
      inertial_term_cost12, NULL, pose1_.data(), gyro_bias1_.data(),
      velocity1_.data(), accel_bias1_.data(), pose2_.data(), gyro_bias2_.data(),
      velocity2_.data(), accel_bias2_.data());

  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;
  problem_.SetParameterization(pose0_.data(), pose_parameterization);
  problem_.SetParameterization(pose1_.data(), pose_parameterization);
  problem_.SetParameterization(pose2_.data(), pose_parameterization);

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(gyro_bias2_.data());
  problem_.SetParameterBlockConstant(accel_bias2_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity2_.data());
  problem_.SetParameterBlockConstant(pose0_.data());
  problem_.SetParameterBlockConstant(pose2_.data());
}

void PosegraphErrorTerms::solve() {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 500;
  options.gradient_tolerance = 1e-50;
  options.function_tolerance = 1e-50;
  options.parameter_tolerance = 1e-50;
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;

  ceres::Solve(options, &problem_, &summary_);

  LOG(INFO) << summary_.message;
  LOG(INFO) << summary_.BriefReport();
}

TEST_F(PosegraphErrorTerms, InertialTerm3KeyframesZeroCost) {
  addResidual();
  solve();

  EXPECT_LT(summary_.final_cost, 1e-12);
}

TEST_F(PosegraphErrorTerms, InertialTerm3KeyframesPositionNoise) {
  Eigen::Vector3d true_pos1 = pos1_;
  pos1_ += Eigen::Vector3d(0.1, -0.2, 0.13);

  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(pose1_.tail(3), true_pos1, 1e-6);
  EXPECT_LT(summary_.final_cost, 1e-12);
}

TEST_F(PosegraphErrorTerms, InertialTerm3KeyframesRotationNoise) {
  Eigen::Vector4d true_rot1 = rot1_.coeffs();
  rot1_.coeffs() += Eigen::Vector4d(0.1, -0.2, 0.13, 0.2);
  rot1_.normalize();

  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(pose1_.head(4), true_rot1, 1e-6);
  EXPECT_LT(summary_.final_cost, 1e-12);
}

TEST_F(PosegraphErrorTerms, InertialTerm3KeyframesVelocityNoise) {
  Eigen::Vector3d true_velocity1 = velocity1_;
  velocity1_ += Eigen::Vector3d(0.1, -0.2, 0.13);

  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(velocity1_, true_velocity1, 1e-6);
  EXPECT_LT(summary_.final_cost, 1e-12);
}

TEST_F(PosegraphErrorTerms, InertialTerm3KeyframesRotVelPosNoise) {
  Eigen::Vector3d true_velocity1 = velocity1_;
  velocity1_ += Eigen::Vector3d(0.1, -0.2, 0.13);

  Eigen::Vector4d true_rot1 = rot1_.coeffs();
  rot1_.coeffs() += Eigen::Vector4d(0.1, -0.2, 0.13, 0.2);
  rot1_.normalize();

  Eigen::Vector3d true_pos1 = pos1_;
  pos1_ += Eigen::Vector3d(0.1, -0.2, 0.13);

  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(velocity1_, true_velocity1, 1e-6);
  EXPECT_NEAR_EIGEN(pose1_.head(4), true_rot1, 1e-6);
  EXPECT_NEAR_EIGEN(pose1_.tail(3), true_pos1, 1e-6);
  EXPECT_LT(summary_.final_cost, 1e-12);
}

MAPLAB_UNITTEST_ENTRYPOINT
