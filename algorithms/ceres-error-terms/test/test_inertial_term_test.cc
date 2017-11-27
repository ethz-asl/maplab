#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ceres-error-terms/inertial-error-term.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

using ceres_error_terms::InertialErrorTerm;

class PosegraphErrorTerms : public ::testing::Test {
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

    imu_timestamps_ << 0, 0.5 * 1e9, 1.0 * 1e9;
    imu_data_ << 1, 1, 1, 0, 0, 0, gravity_magnitude_, gravity_magnitude_,
        gravity_magnitude_, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  }

  void addResidual();
  void solve();

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;

  Eigen::Matrix<int64_t, 1, 3> imu_timestamps_;
  Eigen::Matrix<double, 6, 3> imu_data_;

  Eigen::Quaterniond rot0_;
  Eigen::Quaterniond rot1_;
  Eigen::Vector3d pos0_;
  Eigen::Vector3d pos1_;
  Eigen::Matrix<double, 7, 1> pose0_;
  Eigen::Matrix<double, 7, 1> pose1_;

  Eigen::Vector3d accel_bias0_;
  Eigen::Vector3d accel_bias1_;
  Eigen::Vector3d gyro_bias0_;
  Eigen::Vector3d gyro_bias1_;
  Eigen::Vector3d velocity0_;
  Eigen::Vector3d velocity1_;

  double gravity_magnitude_;
};

void PosegraphErrorTerms::addResidual() {
  rot0_.normalize();
  rot1_.normalize();
  pose0_ << rot0_.coeffs(), pos0_;
  pose1_ << rot1_.coeffs(), pos1_;

  ceres::CostFunction* inertial_term_cost =
      new ceres_error_terms::InertialErrorTerm(
          imu_data_, imu_timestamps_, 1, 1, 1, 1, gravity_magnitude_);

  problem_.AddResidualBlock(
      inertial_term_cost, NULL, pose0_.data(), gyro_bias0_.data(),
      velocity0_.data(), accel_bias0_.data(), pose1_.data(), gyro_bias1_.data(),
      velocity1_.data(), accel_bias1_.data());

  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;
  problem_.SetParameterization(pose0_.data(), pose_parameterization);
  problem_.SetParameterization(pose1_.data(), pose_parameterization);
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

TEST_F(PosegraphErrorTerms, InertialTermZeroCost) {
  addResidual();
  solve();

  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermFinalPositionOptimization) {
  pos1_ << 1.43, -0.2, 0.175;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pose0_.data());

  solve();
  EXPECT_NEAR_EIGEN(pose1_.tail(3), Eigen::Vector3d(1.5, 0, 0), 1e-15);
  EXPECT_NEAR_EIGEN(pose1_.head(4), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermFinalPositionVelocityOptimization) {
  pos1_ << 1.43, -0.2, 0.175;
  velocity1_ << 2.1, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pose0_.data());

  solve();
  EXPECT_NEAR_EIGEN(pose1_.tail(3), Eigen::Vector3d(1.5, 0, 0), 1e-15);
  EXPECT_NEAR_EIGEN(pose1_.head(4), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermFinalRotationOptimization) {
  rot1_.coeffs() << 0.024225143749034013, 0.04470367401201076,
      0.04242220263937102, 0.9978051316080664;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pose0_.data());

  solve();
  EXPECT_NEAR_EIGEN(pose1_.tail(3), pos1_, 1e-15);
  EXPECT_NEAR_EIGEN(pose1_.head(4), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermFinalVelocityOptimization) {
  velocity1_ << 2.1, 0, 0;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pose0_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermStartPositionOptimization) {
  pos0_ << 0.13, -0.2, 0.175;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_ZERO_EIGEN(pose0_.tail(3), 1e-15);
  EXPECT_NEAR_EIGEN(pose0_.head(4), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermStartRotationOptimization) {
  rot0_.coeffs() << 0.024225143749034013, 0.04470367401201076,
      0.04242220263937102, 0.9978051316080664;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_ZERO_EIGEN(pose0_.tail(3), 1e-15);
  EXPECT_NEAR_EIGEN(pose0_.head(4), Eigen::Vector4d(0, 0, 0, 1), 1e-15);
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermStartVelocityOptimization) {
  velocity0_ << 2.1, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pose0_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_NEAR_EIGEN(velocity0_, Eigen::Vector3d(1, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermStartGyroBiasOptimization) {
  gyro_bias0_ << 0.3, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pose0_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_ZERO_EIGEN(gyro_bias0_, 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermStartAccelBiasOptimization) {
  accel_bias0_ << 0.1, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pose0_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_ZERO_EIGEN(accel_bias0_, 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermFinalGyroBiasOptimization) {
  gyro_bias1_ << 0.3, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(pose0_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_ZERO_EIGEN(gyro_bias1_, 1e-15);
  EXPECT_NEAR_EIGEN(velocity1_, Eigen::Vector3d(2, 0, 0), 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

TEST_F(PosegraphErrorTerms, InertialTermFinalAccelBiasOptimization) {
  accel_bias1_ << -0.3, -0.2, 0.1;
  addResidual();

  problem_.SetParameterBlockConstant(gyro_bias0_.data());
  problem_.SetParameterBlockConstant(accel_bias0_.data());
  problem_.SetParameterBlockConstant(gyro_bias1_.data());
  problem_.SetParameterBlockConstant(velocity0_.data());
  problem_.SetParameterBlockConstant(velocity1_.data());
  problem_.SetParameterBlockConstant(pose0_.data());
  problem_.SetParameterBlockConstant(pose1_.data());

  solve();
  EXPECT_ZERO_EIGEN(accel_bias1_, 1e-15);
  EXPECT_LT(summary_.final_cost, 1e-15);
}

MAPLAB_UNITTEST_ENTRYPOINT
