#include <Eigen/Core>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/pose-prior-error-term.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

class PosegraphErrorTerms : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    prior_position_ << 1, 2, 3;
    prior_orientation_.coeffs() << sqrt(2) / 2, 0, 0, sqrt(2) / 2;

    current_pose_ << prior_orientation_.coeffs(), prior_position_;

    covariance_matrix_.setIdentity();
  }

  void addResidual();
  void solve();

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;

  Eigen::Vector3d prior_position_;
  Eigen::Quaterniond prior_orientation_;

  Eigen::Matrix<double, 7, 1> current_pose_;
  Eigen::Matrix<double, 6, 6> covariance_matrix_;
};

void PosegraphErrorTerms::addResidual() {
  ceres::CostFunction* cost_function =
      new ceres_error_terms::PosePriorErrorTerm(
          prior_orientation_.coeffs(), prior_position_, covariance_matrix_);

  problem_.AddResidualBlock(
      cost_function, NULL, current_pose_.data(), &current_pose_(4, 0));
  ceres::LocalParameterization* quaternion_parametrization =
      new ceres_error_terms::JplQuaternionParameterization;
  problem_.SetParameterization(
      current_pose_.data(), quaternion_parametrization);
}

void PosegraphErrorTerms::solve() {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.parameter_tolerance = 1e-20;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;
  options.max_num_iterations = 1e3;
  ceres::Solve(options, &problem_, &summary_);

  VLOG(1) << summary_.BriefReport() << std::endl;
  VLOG(1) << summary_.message << std::endl;
}

TEST_F(PosegraphErrorTerms, TestPosePriorErrorTermZeroCost) {
  addResidual();
  solve();

  const double kPrecision = 1.e-15;
  EXPECT_NEAR(summary_.initial_cost, 0.0, kPrecision);
  EXPECT_NEAR(summary_.final_cost, 0.0, kPrecision);
  EXPECT_EQ(summary_.iterations.size(), 1u);
}

TEST_F(PosegraphErrorTerms, TestPosePriorErrorTermPositionOptimization) {
  current_pose_.tail(3) << 4, 7, 4;
  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_pose_.tail(3), prior_position_, 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

TEST_F(PosegraphErrorTerms, TestPosePriorErrorTermOrientationOptimization) {
  current_pose_.head(4) << 0, 0, 0, 1;
  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_pose_.head(4), prior_orientation_.coeffs(), 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

TEST_F(PosegraphErrorTerms, TestPoseriorErrorTermOrientationOptimization2) {
  current_pose_.head(4) << 0, -sqrt(2) / 2, sqrt(2) / 2, 0;
  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_pose_.head(4), prior_orientation_.coeffs(), 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

TEST_F(PosegraphErrorTerms, TestPosePriorErrorTermOrientationOptimization3) {
  current_pose_.head(4) << sqrt(2) / 2, -sqrt(2) / 2, 0, 0;
  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_pose_.head(4), prior_orientation_.coeffs(), 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

TEST_F(PosegraphErrorTerms, TestPosePriorErrorTermPoseOptimization) {
  current_pose_.head(4) << 0, -sqrt(2) / 2, sqrt(2) / 2, 0;
  current_pose_.tail(3) << -1, 7, 53;
  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_pose_.tail(3), prior_position_, 1e-10);
  EXPECT_NEAR_EIGEN(current_pose_.head(4), prior_orientation_.coeffs(), 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

MAPLAB_UNITTEST_ENTRYPOINT
