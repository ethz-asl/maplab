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

#include "ceres-error-terms/block-pose-prior-error-term.h"
#include "ceres-error-terms/common.h"
#include "ceres-error-terms/six-dof-block-pose-error-term-with-extrinsics-autodiff.h"

class SixDofBlockPoseErrorTermsWithExtrinsics : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    // Some arbitrary numbers for position and orientation.
    prior_position_G_Ik_ << 1.0, 2.0, 3.0;
    prior_position_G_Ikp1_ << 4.0, 5.0, 6.0;
    prior_position_B_I_ << 7.0, 8.0, 9.0;
    prior_orientation_G_Ik_.coeffs() << sqrt(2) / 2, 0, 0, sqrt(2) / 2;
    prior_orientation_G_Ikp1_.coeffs() << 0.15496688, -0.60676538, 0.30722298,
        0.71654385;
    prior_orientation_G_Ikp1_.coeffs().normalize();
    prior_orientation_B_I_.coeffs() << 0.47112391, 0.45194212, 0.07820137,
        0.75344219;
    prior_orientation_B_I_.coeffs().normalize();

    pose_G_Ik_ << prior_orientation_G_Ik_.coeffs(), prior_position_G_Ik_;
    pose_G_Ikp1_ << prior_orientation_G_Ikp1_.coeffs(), prior_position_G_Ikp1_;

    position_B_I_ << prior_position_B_I_;
    orientation_B_I_ << prior_orientation_B_I_.inverse().coeffs();
    pose_B_I_ << orientation_B_I_, position_B_I_;

    T_Bk_Bkp1_measurement_covariance_matrix_.setIdentity();
    T_B_I_prior_covariance_matrix_.setIdentity();
  }

  void addPriorOn_T_B_I(const aslam::Transformation& T_B_I);
  void add_T_Bk_Bkp1_Measurement(
      const aslam::Transformation& T_Ik_Ikp1_measurement);
  void solve();

  void fix_T_G_Ik();
  void fix_T_G_Ikp1();
  void fix_T_B_I();

  aslam::Transformation get_T_G_Ik() const;
  aslam::Transformation get_T_G_Ikp1() const;
  aslam::Transformation get_T_B_I() const;

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;

  Eigen::Vector3d prior_position_G_Ik_;
  Eigen::Vector3d prior_position_G_Ikp1_;
  Eigen::Vector3d prior_position_B_I_;
  Eigen::Quaterniond prior_orientation_G_Ik_;
  Eigen::Quaterniond prior_orientation_G_Ikp1_;
  Eigen::Quaterniond prior_orientation_B_I_;

  Eigen::Matrix<double, 7, 1> pose_G_Ik_;
  Eigen::Matrix<double, 7, 1> pose_G_Ikp1_;
  Eigen::Matrix<double, 7, 1> pose_B_I_;
  Eigen::Vector3d position_B_I_;
  Eigen::Vector4d orientation_B_I_;

  Eigen::Matrix<double, 6, 6> T_Bk_Bkp1_measurement_covariance_matrix_;
  Eigen::Matrix<double, 6, 6> T_B_I_prior_covariance_matrix_;
};

void SixDofBlockPoseErrorTermsWithExtrinsics::add_T_Bk_Bkp1_Measurement(
    const aslam::Transformation& T_Bk_Bkp1) {
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;
  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::JplQuaternionParameterization;

  ceres::CostFunction* relative_pose_cost = new ceres::AutoDiffCostFunction<
      ceres_error_terms::SixDoFBlockPoseErrorTermWithExtrinsics,
      ceres_error_terms::SixDoFBlockPoseErrorTermWithExtrinsics::
          kResidualBlockSize,
      ceres_error_terms::poseblocks::kPoseSize,
      ceres_error_terms::poseblocks::kPoseSize,
      ceres_error_terms::poseblocks::kOrientationBlockSize,
      ceres_error_terms::poseblocks::kPositionBlockSize>(
      new ceres_error_terms::SixDoFBlockPoseErrorTermWithExtrinsics(
          T_Bk_Bkp1, T_Bk_Bkp1_measurement_covariance_matrix_));
  problem_.AddResidualBlock(
      relative_pose_cost, nullptr, pose_G_Ik_.data(), pose_G_Ikp1_.data(),
      orientation_B_I_.data(), position_B_I_.data());

  problem_.SetParameterization(pose_G_Ik_.data(), pose_parameterization);
  problem_.SetParameterization(pose_G_Ikp1_.data(), pose_parameterization);
  problem_.SetParameterization(
      orientation_B_I_.data(), quaternion_parameterization);
}

void SixDofBlockPoseErrorTermsWithExtrinsics::addPriorOn_T_B_I(
    const aslam::Transformation& T_B_I) {
  ceres::CostFunction* prior_cost_function(
      new ceres_error_terms::BlockPosePriorErrorTerm(
          T_B_I.getEigenQuaternion().coeffs(), T_B_I.getPosition(),
          T_B_I_prior_covariance_matrix_));

  problem_.AddResidualBlock(
      prior_cost_function, nullptr, orientation_B_I_.data(),
      position_B_I_.data());
}

void SixDofBlockPoseErrorTermsWithExtrinsics::fix_T_G_Ik() {
  problem_.SetParameterBlockConstant(pose_G_Ik_.data());
}

void SixDofBlockPoseErrorTermsWithExtrinsics::fix_T_G_Ikp1() {
  problem_.SetParameterBlockConstant(pose_G_Ikp1_.data());
}

void SixDofBlockPoseErrorTermsWithExtrinsics::fix_T_B_I() {
  problem_.SetParameterBlockConstant(orientation_B_I_.data());
  problem_.SetParameterBlockConstant(position_B_I_.data());
}

aslam::Transformation SixDofBlockPoseErrorTermsWithExtrinsics::get_T_G_Ik()
    const {
  const Eigen::Map<const Eigen::Quaterniond> q_G_Ik(pose_G_Ik_.data());
  return aslam::Transformation(aslam::Quaternion(q_G_Ik), pose_G_Ik_.tail(3));
}

aslam::Transformation SixDofBlockPoseErrorTermsWithExtrinsics::get_T_G_Ikp1()
    const {
  const Eigen::Map<const Eigen::Quaterniond> q_G_Ikp1(pose_G_Ikp1_.data());
  return aslam::Transformation(
      aslam::Quaternion(q_G_Ikp1), pose_G_Ikp1_.tail(3));
}

aslam::Transformation SixDofBlockPoseErrorTermsWithExtrinsics::get_T_B_I()
    const {
  const Eigen::Map<const Eigen::Quaterniond> q_I_B_JPL(pose_B_I_.data());
  return aslam::Transformation(
      aslam::Quaternion(q_I_B_JPL).inverse(), pose_B_I_.tail(3));
}

void SixDofBlockPoseErrorTermsWithExtrinsics::solve() {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.parameter_tolerance = 1e-14;
  options.gradient_tolerance = 1e-14;
  options.function_tolerance = 1e-14;
  options.max_num_iterations = 1e5;
  ceres::Solve(options, &problem_, &summary_);
}

TEST_F(
    SixDofBlockPoseErrorTermsWithExtrinsics, TestPosePriorErrorTermZeroCost) {
  const aslam::Transformation T_Bk_Bkp1 = get_T_B_I() * get_T_G_Ik().inverse() *
                                          get_T_G_Ikp1() *
                                          get_T_B_I().inverse();

  add_T_Bk_Bkp1_Measurement(T_Bk_Bkp1);
  fix_T_B_I();
  fix_T_G_Ik();
  solve();

  EXPECT_NEAR(summary_.initial_cost, 0.0, 1e-12);
  EXPECT_NEAR(summary_.final_cost, 0.0, 1e-12);
}

TEST_F(
    SixDofBlockPoseErrorTermsWithExtrinsics,
    TestPosePriorErrorTermIdentityFixBIFixGIk) {
  add_T_Bk_Bkp1_Measurement(aslam::Transformation());
  fix_T_B_I();
  fix_T_G_Ik();
  solve();

  EXPECT_NEAR(summary_.final_cost, 0.0, 1e-12);
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(
          get_T_G_Ikp1().getTransformationMatrix(),
          get_T_G_Ik().getTransformationMatrix(), 1e-8));
}

// TODO(andrepfr): Test disabled since quaternion convention of
// six-dof-block-transformation is different from the one expected in this test.
// Re-enable once bundle adjustment and error terms are changed to have a
// consistent convention.
TEST_F(
    SixDofBlockPoseErrorTermsWithExtrinsics,
    DISABLED_TestPosePriorErrorTermArbitraryMeasurementWithPriorOn_T_B_I) {
  Eigen::Vector3d angle_axis_Bk_Bkp1_vec;
  angle_axis_Bk_Bkp1_vec << 0.8, -1.2, 0.456;  // Some fixed arbitrary numbers.
  Eigen::Vector3d angle_axis_Bk_Bkp1_axis = angle_axis_Bk_Bkp1_vec;
  angle_axis_Bk_Bkp1_axis.normalize();
  const aslam::AngleAxis q_Bk_Bkp1_angle_axis(
      angle_axis_Bk_Bkp1_vec.norm(), angle_axis_Bk_Bkp1_axis);
  Eigen::Vector3d p_Bk_Bkp1(
      125.4, 43423.3, -33441.423);  // Some fixed arbitrary numbers.
  const aslam::Transformation T_Bk_BKp1_measurement(
      aslam::Quaternion(q_Bk_Bkp1_angle_axis), p_Bk_Bkp1);
  add_T_Bk_Bkp1_Measurement(T_Bk_BKp1_measurement);

  Eigen::Vector3d angle_axis_B_I_vec;
  angle_axis_B_I_vec << -0.4, 1.2, -0.98;  // Some fixed arbitrary numbers.
  Eigen::Vector3d angle_axis_B_I_axis = angle_axis_B_I_vec;
  angle_axis_B_I_axis.normalize();
  const aslam::AngleAxis q_B_I_angle_axis(
      angle_axis_B_I_vec.norm(), angle_axis_B_I_axis);
  Eigen::Vector3d p_B_I(
      -4575.045, 8751.15, 456.5);  // Some fixed arbitrary numbers.
  const aslam::Transformation T_B_I_measurement(
      aslam::Quaternion(q_B_I_angle_axis), p_B_I);

  addPriorOn_T_B_I(T_B_I_measurement);

  fix_T_G_Ik();
  solve();

  EXPECT_NEAR(summary_.final_cost, 0.0, 1e-6);
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(
          get_T_B_I().getTransformationMatrix(),
          T_B_I_measurement.getTransformationMatrix(), 1e-8));

  const aslam::Transformation T_Ik_Ikp1_ground_truth =
      T_B_I_measurement.inverse() * T_Bk_BKp1_measurement * T_B_I_measurement;
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(
          (get_T_G_Ik().inverse() * get_T_G_Ikp1()).getTransformationMatrix(),
          T_Ik_Ikp1_ground_truth.getTransformationMatrix(), 1e-5));
}
MAPLAB_UNITTEST_ENTRYPOINT
