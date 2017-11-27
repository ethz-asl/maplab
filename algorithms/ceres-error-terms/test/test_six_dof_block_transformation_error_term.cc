#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres/ceres.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <ceres-error-terms/common.h>
#include <ceres-error-terms/six-dof-block-pose-error-term-autodiff.h>

class SixDofBlockPoseErrorTerms : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    prior_position_A_ << 1, 2, 3;
    prior_position_B_ << 4, 5, 6;
    prior_orientation_A_.coeffs() << sqrt(2) / 2, 0, 0, sqrt(2) / 2;
    prior_orientation_B_.coeffs() << 0.15496688, -0.60676538, 0.30722298,
        0.71654385;
    prior_orientation_B_.coeffs().normalize();

    pose_A_ << prior_orientation_A_.coeffs(), prior_position_A_;
    pose_B_ << prior_orientation_B_.coeffs(), prior_position_B_;

    covariance_matrix_.setIdentity();
  }

  void addResidual(const aslam::Transformation& T_A_B);
  void solve();

  void fixA();
  void fixB();

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;

  Eigen::Vector3d prior_position_A_;
  Eigen::Vector3d prior_position_B_;
  Eigen::Quaterniond prior_orientation_A_;
  Eigen::Quaterniond prior_orientation_B_;

  Eigen::Matrix<double, 7, 1> pose_A_;
  Eigen::Matrix<double, 7, 1> pose_B_;
  Eigen::Matrix<double, 6, 6> covariance_matrix_;
};

void SixDofBlockPoseErrorTerms::addResidual(
    const aslam::Transformation& T_A_B) {
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;

  ceres::CostFunction* relative_pose_cost = new ceres::AutoDiffCostFunction<
      ceres_error_terms::SixDoFBlockPoseErrorTerm,
      ceres_error_terms::SixDoFBlockPoseErrorTerm::residualBlockSize,
      ceres_error_terms::poseblocks::kPoseSize,
      ceres_error_terms::poseblocks::kPoseSize>(
      new ceres_error_terms::SixDoFBlockPoseErrorTerm(
          T_A_B, covariance_matrix_));

  problem_.AddResidualBlock(
      relative_pose_cost, NULL, pose_A_.data(), pose_B_.data());

  problem_.SetParameterization(pose_A_.data(), pose_parameterization);
  problem_.SetParameterization(pose_B_.data(), pose_parameterization);
}

void SixDofBlockPoseErrorTerms::fixA() {
  problem_.SetParameterBlockConstant(pose_A_.data());
}

void SixDofBlockPoseErrorTerms::fixB() {
  problem_.SetParameterBlockConstant(pose_B_.data());
}

void SixDofBlockPoseErrorTerms::solve() {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.parameter_tolerance = 1e-14;
  options.gradient_tolerance = 1e-14;
  options.function_tolerance = 1e-14;
  options.max_num_iterations = 30u;
  LOG(INFO) << "Solving...";
  ceres::Solve(options, &problem_, &summary_);

  LOG(INFO) << summary_.BriefReport() << std::endl;
  LOG(INFO) << summary_.message << std::endl;
}

TEST_F(SixDofBlockPoseErrorTerms, TestPosePriorErrorTermZeroCost) {
  aslam::Transformation T_G_A(prior_orientation_A_, prior_position_A_);
  aslam::Transformation T_G_B(prior_orientation_B_, prior_position_B_);

  addResidual(T_G_A.inverse() * T_G_B);
  fixA();
  solve();

  EXPECT_NEAR(summary_.initial_cost, 0.0, 1e-12);
  EXPECT_NEAR(summary_.final_cost, 0.0, 1e-12);
  EXPECT_EQ(summary_.iterations.size(), 1u);
}

TEST_F(SixDofBlockPoseErrorTerms, TestPosePriorErrorTermIdentityPoseA) {
  aslam::Transformation T_G_A(prior_orientation_A_, prior_position_A_);

  addResidual(aslam::Transformation());
  fixA();
  solve();

  aslam::Transformation T_G_B_est(
      Eigen::Quaterniond(pose_B_.head(4).data()), pose_B_.tail(3));

  EXPECT_NEAR(summary_.final_cost, 0.0, 1e-12);
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(
          T_G_B_est.getTransformationMatrix(), T_G_A.getTransformationMatrix(),
          1e-8));
}

TEST_F(SixDofBlockPoseErrorTerms, TestPosePriorErrorTermIdentityPoseB) {
  aslam::Transformation T_G_B(prior_orientation_B_, prior_position_B_);

  addResidual(aslam::Transformation());
  fixB();
  solve();

  aslam::Transformation T_G_A_est(
      Eigen::Quaterniond(pose_A_.head(4).data()), pose_A_.tail(3));

  EXPECT_NEAR(summary_.final_cost, 0.0, 1e-12);
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(
          T_G_A_est.getTransformationMatrix(), T_G_B.getTransformationMatrix(),
          1e-8));
}

TEST_F(SixDofBlockPoseErrorTerms, TestPosePriorErrorTermNonIdentity) {
  aslam::Transformation T_G_A(prior_orientation_A_, prior_position_A_);

  Eigen::Vector3d q_A_B_axis_angle_vec;
  q_A_B_axis_angle_vec << 0.2, -1.2, 0.4;
  Eigen::Vector3d q_A_B_axis = q_A_B_axis_angle_vec;
  q_A_B_axis.normalize();
  aslam::AngleAxis q_A_B_angle_axis(q_A_B_axis_angle_vec.norm(), q_A_B_axis);
  aslam::Quaternion q_A_B(q_A_B_angle_axis);
  Eigen::Vector3d p_A_B;
  p_A_B << 12.4, 423.3, -341.423;
  aslam::Transformation T_A_B(q_A_B, p_A_B);

  addResidual(T_A_B);
  fixA();
  solve();

  aslam::Transformation T_G_B_est(
      Eigen::Quaterniond(pose_B_.head(4).data()), pose_B_.tail(3));

  EXPECT_NEAR(summary_.final_cost, 0.0, 1e-12);
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(
          T_G_B_est.getTransformationMatrix(),
          (T_G_A * T_A_B).getTransformationMatrix(), 1e-8));
}

MAPLAB_UNITTEST_ENTRYPOINT
