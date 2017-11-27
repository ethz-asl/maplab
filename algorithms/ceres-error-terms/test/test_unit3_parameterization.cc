#include <Eigen/Core>
#include <Eigen/Dense>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "ceres-error-terms/parameterization/unit3-param.h"

TEST(Unit3, TestUnit3CeresParametrization) {
  // Initial state values.
  Eigen::Quaterniond q(Eigen::Quaterniond::Identity().slerp(
      0.10, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Quaterniond p(Eigen::Quaterniond::Identity().slerp(
      0.20, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Vector2d u(0.2, 0.3);

  // Check: (q boxplus u)
  Eigen::Quaterniond q_rot;
  Eigen::Quaterniond q_rot_ceres;
  Eigen::Vector2d u_new;
  ceres_error_terms::Unit3::Plus(q, u, &q_rot);

  ceres_error_terms::Unit3Parameterization ceres_unit3_parametrization;

  ceres_unit3_parametrization.Plus(
      q.coeffs().data(), u.data(), q_rot_ceres.coeffs().data());

  pose::Quaternion q_rot_(q_rot);
  pose::Quaternion q_rot_ceres_(q_rot_ceres);
  EXPECT_NEAR_KINDR_QUATERNION(q_rot_, q_rot_ceres_, 1e-5);

  // Check: p boxminus q
  Eigen::Vector2d theta;
  Eigen::Vector2d theta_ceres;
  Eigen::Quaterniond p_orig;
  ceres_error_terms::Unit3::Minus(p, q, &theta);
  ceres_unit3_parametrization.Minus(
      p.coeffs().data(), q.coeffs().data(), theta_ceres.data());

  EXPECT_NEAR_EIGEN(theta, theta_ceres, 1e-5);
}

TEST(Unit3, TestUnit3) {
  // Initial state values.
  Eigen::Quaterniond q(Eigen::Quaterniond::Identity().slerp(
      0.10, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Quaterniond p(Eigen::Quaterniond::Identity().slerp(
      0.20, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Vector2d u(0.2, 0.3);

  // Check: (q boxplus u) boxminus q = u
  Eigen::Quaterniond q_rot;
  Eigen::Vector2d u_new;
  ceres_error_terms::Unit3::Plus(q, u, &q_rot);
  ceres_error_terms::Unit3::Minus(q_rot, q, &u_new);

  EXPECT_NEAR_EIGEN(u, u_new, 1e-5);

  // Check: (q boxplus u) boxplus -u = q
  Eigen::Quaterniond q_orig;
  Eigen::Vector2d u_neg = -1.0 * u;
  ceres_error_terms::Unit3::Plus(q_rot, u_neg, &q_orig);

  pose::Quaternion q_(q);
  pose::Quaternion q_orig_(q_orig);
  EXPECT_NEAR_KINDR_QUATERNION(q_, q_orig_, 1e-5);

  // Check: q boxplus (p boxminus q) = p
  Eigen::Vector2d theta;
  Eigen::Quaterniond p_orig;
  ceres_error_terms::Unit3::Minus(p, q, &theta);
  ceres_error_terms::Unit3::Plus(q, theta, &p_orig);

  pose::Quaternion p_(p);
  pose::Quaternion p_orig_(p_orig);
  EXPECT_NEAR_KINDR_QUATERNION(p_, p_orig_, 1e-5);
}

MAPLAB_UNITTEST_ENTRYPOINT
