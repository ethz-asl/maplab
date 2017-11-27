#include <ceres/ceres.h>
#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "ceres-error-terms/parameterization/quaternion-param-jpl.h"
#include "ceres-error-terms/test/parameterization-numerical-diff.h"

TEST(JplQuaternionParameterization, JacobianCorrect) {
  typedef Eigen::Matrix<double, 4, 3, Eigen::RowMajor> JacobianType;
  Eigen::Quaterniond q_AB(1.0, 1.0, 0.0, 1.0);
  q_AB.normalize();

  ceres_error_terms::JplQuaternionParameterization parameterization;
  JacobianType dq_dtheta;
  ASSERT_TRUE(
      parameterization.ComputeJacobian(q_AB.coeffs().data(), dq_dtheta.data()));

  JacobianType dq_dtheta_numeric;
  const bool success =
      ceres_error_terms::EvaluateNumericalJacobianOfParameterization<
          ceres_error_terms::JplQuaternionParameterization, 4, 3>(
          parameterization, q_AB.coeffs(), &dq_dtheta_numeric);
  ASSERT_TRUE(success);

  EXPECT_NEAR_EIGEN(dq_dtheta, dq_dtheta_numeric, 1e-10);
}

TEST(JplYawQuaternionParameterization, Plus) {
  Eigen::Quaterniond q_AB(1.0, 1.0, 0.0, 1.0);
  q_AB.normalize();
  const Eigen::Vector3d rpy_init =
      common::getRollPitchYawFromQuaternionJpl(q_AB);

  double delta_total_rad = 0.0;
  ceres_error_terms::JplYawQuaternionParameterization yaw_param;
  for (double delta : std::vector<double>{-0.5, 0.0, 0.5, 1.0}) {
    Eigen::Quaterniond q_AB_plus_delta;
    ASSERT_TRUE(yaw_param.Plus(q_AB.coeffs().data(), &delta,
                               q_AB_plus_delta.coeffs().data()));
    delta_total_rad += delta;
    q_AB = q_AB_plus_delta;

    const Eigen::Vector3d rpy =
        common::getRollPitchYawFromQuaternionJpl(q_AB_plus_delta);
    EXPECT_NEAR(rpy_init[0], rpy[0], 1e-6);
    EXPECT_NEAR(rpy_init[1], rpy[1], 1e-6);

    if (delta_total_rad == 0.0) {
      // We are at the same yaw as initially.
      EXPECT_NEAR(rpy_init[2], rpy[2], 1e-6);
    } else {
      EXPECT_NE(rpy_init[2], rpy[2]);
    }
  }
}

TEST(JplYawQuaternionParameterization, JacobianCorrect) {
  typedef Eigen::Matrix<double, 4, 1> JacobianType;
  Eigen::Quaterniond q_AB(1.0, 1.0, 0.0, 1.0);
  q_AB.normalize();

  ceres_error_terms::JplYawQuaternionParameterization parameterization;
  JacobianType dq_dtheta;
  ASSERT_TRUE(
      parameterization.ComputeJacobian(q_AB.coeffs().data(), dq_dtheta.data()));

  JacobianType dq_dtheta_numeric;
  const bool success =
      ceres_error_terms::EvaluateNumericalJacobianOfParameterization<
          ceres_error_terms::JplYawQuaternionParameterization, 4, 1>(
          parameterization, q_AB.coeffs(), &dq_dtheta_numeric);
  ASSERT_TRUE(success);

  EXPECT_NEAR_EIGEN(dq_dtheta, dq_dtheta_numeric, 1e-10);
}

TEST(JplRollPitchQuaternionParameterization, JacobianCorrect) {
  typedef Eigen::Matrix<double, 4, 2, Eigen::RowMajor> JacobianType;
  Eigen::Quaterniond q_IM(1.0, 1.0, 5.0, 0.0);
  q_IM.normalize();

  Eigen::Quaterniond q_GM(1.0, 5.0, 0.0, 2.0);
  q_GM.normalize();

  ceres_error_terms::JplRollPitchQuaternionParameterization parameterization(
      q_GM.coeffs());
  JacobianType dq_dtheta;
  ASSERT_TRUE(
      parameterization.ComputeJacobian(q_IM.coeffs().data(), dq_dtheta.data()));

  JacobianType dq_dtheta_numeric;
  const bool success =
      ceres_error_terms::EvaluateNumericalJacobianOfParameterization<
          ceres_error_terms::JplRollPitchQuaternionParameterization, 4, 2>(
          parameterization, q_IM.coeffs(), &dq_dtheta_numeric);
  ASSERT_TRUE(success);

  EXPECT_NEAR_EIGEN(dq_dtheta, dq_dtheta_numeric, 1e-10);
}

MAPLAB_UNITTEST_ENTRYPOINT
