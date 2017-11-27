#include <Eigen/Core>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/position-error-term.h>
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
  Eigen::Matrix<double, 3, 3> covariance_matrix_;
};

void PosegraphErrorTerms::addResidual() {
  ceres::CostFunction* cost_function = new ceres_error_terms::PositionErrorTerm(
      prior_position_, covariance_matrix_);

  problem_.AddResidualBlock(cost_function, NULL, current_pose_.data());
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;
  problem_.SetParameterization(current_pose_.data(), pose_parameterization);
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

  LOG(INFO) << summary_.BriefReport() << std::endl;
  LOG(INFO) << summary_.message << std::endl;
}

TEST_F(PosegraphErrorTerms, TestPositionPriorErrorTermZeroCost) {
  addResidual();
  solve();

  EXPECT_EQ(summary_.initial_cost, 0.0);
  EXPECT_EQ(summary_.final_cost, 0.0);
  EXPECT_EQ(summary_.iterations.size(), 1u);
}

TEST_F(PosegraphErrorTerms, TestPositionPriorErrorTermPositionOptimization) {
  current_pose_.tail(3) << 4, 7, 4;
  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_pose_.tail(3), prior_position_, 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

TEST_F(PosegraphErrorTerms, TestPositionPriorErrorTermPositionOptimization2) {
  current_pose_.tail(3) << 2.484e8, -1.264e9, -8.4567e24;
  addResidual();
  solve();

  EXPECT_NEAR_EIGEN(current_pose_.tail(3), prior_position_, 1e-10);
  EXPECT_LT(summary_.final_cost, 1e-10);
}

MAPLAB_UNITTEST_ENTRYPOINT
