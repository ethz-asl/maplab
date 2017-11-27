#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "ceres-error-terms/inertial-error-term-eigen.h"
#include "ceres-error-terms/parameterization/quaternion-param-eigen.h"

using ceres_error_terms::InertialErrorTermEigen;

class PosegraphErrorTermsEigen : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    rot0_.coeffs() << 0, 0, 0, 1;
    rot1_.coeffs() << 0, 0, 0, 1;
    pos0_ << 0, 0, 0;
    pos1_ << 1.5, 0, 0;

    accel_bias0_ << 0, 0, 0;
    accel_bias1_ << 0, 0, 0;
    gyro_bias0_ << 0, 0, 0;
    gyro_bias1_ << 0, 0, 0;

    velocity0_ << 1, 0, 0;
    velocity1_ << 2, 0, 0;

    common::GravityProvider gravity_provider(
        common::locations::kAltitudeZurichMeters,
        common::locations::kLatitudeZurichDegrees);
    gravity_magnitude_ = gravity_provider.getGravityMagnitude();

    imu_timestamps_ns_ << 0, 0.5 * 1e9, 1.0 * 1e9;
    imu_data_ << 1, 1, 1, 0, 0, 0, gravity_magnitude_, gravity_magnitude_,
        gravity_magnitude_, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  }

  void addResidual();
  void solve();
  void checkGradient();

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;

  Eigen::Matrix<int64_t, 1, 3> imu_timestamps_ns_;
  Eigen::Matrix<double, 6, 3> imu_data_;

  Eigen::Quaterniond rot0_;
  Eigen::Quaterniond rot1_;
  Eigen::Vector3d pos0_;
  Eigen::Vector3d pos1_;

  Eigen::Vector3d accel_bias0_;
  Eigen::Vector3d accel_bias1_;
  Eigen::Vector3d gyro_bias0_;
  Eigen::Vector3d gyro_bias1_;

  Eigen::Vector3d velocity0_;
  Eigen::Vector3d velocity1_;

  Eigen::Matrix<double, 6, 1> imu_bias0_;
  Eigen::Matrix<double, 6, 1> imu_bias1_;

  double gravity_magnitude_;
};

void PosegraphErrorTermsEigen::addResidual() {
  rot0_.normalize();
  rot1_.normalize();
  imu_bias0_ << gyro_bias0_, accel_bias0_;
  imu_bias1_ << gyro_bias1_, accel_bias1_;

  ceres::CostFunction* inertial_term_cost =
      new ceres_error_terms::InertialErrorTermEigen(
          imu_data_, imu_timestamps_ns_, 1, 1, 1, 1, gravity_magnitude_);

  problem_.AddResidualBlock(
      inertial_term_cost, NULL, rot0_.coeffs().data(), pos0_.data(),
      velocity0_.data(), imu_bias0_.data(), rot1_.coeffs().data(), pos1_.data(),
      velocity1_.data(), imu_bias1_.data());

  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::EigenQuaternionParameterization;
  problem_.SetParameterization(
      rot0_.coeffs().data(), quaternion_parameterization);
  problem_.SetParameterization(
      rot1_.coeffs().data(), quaternion_parameterization);
}

void PosegraphErrorTermsEigen::checkGradient() {
  rot0_.normalize();
  rot1_.normalize();
  imu_bias0_ << gyro_bias0_, accel_bias0_;
  imu_bias1_ << gyro_bias1_, accel_bias1_;

  ceres::CostFunction* inertial_term_cost =
      new ceres_error_terms::InertialErrorTermEigen(
          imu_data_, imu_timestamps_ns_, 1, 1, 1, 1, gravity_magnitude_);

  std::vector<double*> parameter_blocks;
  parameter_blocks.push_back(rot0_.coeffs().data());
  parameter_blocks.push_back(pos0_.data());
  parameter_blocks.push_back(velocity0_.data());
  parameter_blocks.push_back(imu_bias0_.data());
  parameter_blocks.push_back(rot1_.coeffs().data());
  parameter_blocks.push_back(pos1_.data());
  parameter_blocks.push_back(velocity1_.data());
  parameter_blocks.push_back(imu_bias1_.data());

  ceres::LocalParameterization* orientation_parameterization =
      new ceres_error_terms::EigenQuaternionParameterization;

  ceres::NumericDiffOptions numeric_diff_options;
  numeric_diff_options.ridders_relative_initial_step_size = 1e-3;

  std::vector<const ceres::LocalParameterization*> local_parameterizations;
  local_parameterizations.push_back(orientation_parameterization);
  local_parameterizations.push_back(NULL);
  local_parameterizations.push_back(NULL);
  local_parameterizations.push_back(NULL);
  local_parameterizations.push_back(orientation_parameterization);
  local_parameterizations.push_back(NULL);
  local_parameterizations.push_back(NULL);
  local_parameterizations.push_back(NULL);

  ceres::GradientChecker gradient_checker(
      inertial_term_cost, &local_parameterizations, numeric_diff_options);
  ceres::GradientChecker::ProbeResults results;

  if (!gradient_checker.Probe(parameter_blocks.data(), 1e-9, &results)) {
    std::cout << "An error has occurred:\n" << results.error_log;
  }
}

void PosegraphErrorTermsEigen::solve() {
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

TEST_F(PosegraphErrorTermsEigen, InertialTermZeroCost) {
  addResidual();
  solve();

  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermFinalPositionOptimization) {
  pos1_ << 1.43, -0.2, 0.175;
  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pos0_.data());
  problem_.SetParameterBlockConstant(rot0_.coeffs().data());

  solve();

  EXPECT_NEAR_EIGEN(pos1_, Eigen::Vector3d(1.5, 0, 0), 1e-15);
  EXPECT_NEAR_EIGEN(rot1_.coeffs(), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(
    PosegraphErrorTermsEigen, InertialTermFinalPositionVelocityOptimization) {
  pos1_ << 1.43, -0.2, 0.175;
  velocity1_ << 2.1, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pos0_.data());
  problem_.SetParameterBlockConstant(rot0_.coeffs().data());
  solve();
  EXPECT_NEAR_EIGEN(pos1_, Eigen::Vector3d(1.5, 0, 0), 1e-15);
  EXPECT_NEAR_EIGEN(rot1_.coeffs(), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermFinalRotationOptimization) {
  rot1_.coeffs() << 0.024225143749034013, 0.04470367401201076,
      0.04242220263937102, 0.9978051316080664;
  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pos0_.data());
  problem_.SetParameterBlockConstant(rot0_.coeffs().data());

  solve();
  //  EXPECT_NEAR_EIGEN(pos1_, pos1_, 1e-15);
  EXPECT_NEAR_EIGEN(rot1_.coeffs(), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermFinalVelocityOptimization) {
  velocity1_ << 2.1, 0, 0;
  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pos0_.data());
  problem_.SetParameterBlockConstant(rot0_.coeffs().data());
  problem_.SetParameterBlockConstant(pos1_.data());
  problem_.SetParameterBlockConstant(rot1_.coeffs().data());

  solve();
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermStartPositionOptimization) {
  pos0_ << 0.13, -0.2, 0.175;
  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pos1_.data());
  problem_.SetParameterBlockConstant(rot1_.coeffs().data());
  solve();

  //  checkGradient();

  EXPECT_ZERO_EIGEN(pos0_, 1e-15);
  EXPECT_NEAR_EIGEN(rot0_.coeffs(), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermStartRotationOptimization) {
  rot0_.coeffs() << 0.024225143749034013, 0.04470367401201076,
      0.04242220263937102, 0.9978051316080664;
  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pos1_.data());
  problem_.SetParameterBlockConstant(rot1_.coeffs().data());
  solve();
  EXPECT_ZERO_EIGEN(pos0_, 1e-15);
  EXPECT_NEAR_EIGEN(rot0_.coeffs(), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermStartVelocityOptimization) {
  velocity0_ << 2.1, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pos0_.data());
  problem_.SetParameterBlockConstant(rot0_.coeffs().data());
  problem_.SetParameterBlockConstant(pos1_.data());
  problem_.SetParameterBlockConstant(rot1_.coeffs().data());

  solve();
  EXPECT_NEAR_EIGEN(velocity0_, Eigen::Vector3d(1, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermStartImuBiasOptimization) {
  gyro_bias0_ << 0.3, -0.2, 0.1;
  accel_bias0_ << 0.1, -0.2, 0.1;

  addResidual();

  problem_.SetParameterBlockConstant(imu_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pos0_.data());
  problem_.SetParameterBlockConstant(rot0_.coeffs().data());
  problem_.SetParameterBlockConstant(pos1_.data());
  problem_.SetParameterBlockConstant(rot1_.coeffs().data());

  solve();
  EXPECT_ZERO_EIGEN(imu_bias0_, 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTermsEigen, InertialTermFinalImuBiasOptimization) {
  gyro_bias1_ << 0.3, -0.2, 0.1;
  accel_bias1_ << -0.3, -0.2, 0.1;

  addResidual();

  problem_.SetParameterBlockConstant(imu_bias0_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pos0_.data());
  problem_.SetParameterBlockConstant(rot0_.coeffs().data());
  problem_.SetParameterBlockConstant(pos1_.data());
  problem_.SetParameterBlockConstant(rot1_.coeffs().data());

  solve();
  EXPECT_ZERO_EIGEN(imu_bias1_, 1e-15);
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

MAPLAB_UNITTEST_ENTRYPOINT
