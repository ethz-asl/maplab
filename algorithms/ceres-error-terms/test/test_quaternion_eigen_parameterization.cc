#include <Eigen/Core>
#include <Eigen/Dense>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "ceres-error-terms/parameterization/quaternion-param-eigen.h"

TEST(QuaternionEigenParametrization, TestQuaternionEigenCeresParametrization) {
  // Initial state values.
  Eigen::Quaterniond q(Eigen::Quaterniond::Identity().slerp(
      0.10, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Quaterniond p(Eigen::Quaterniond::Identity().slerp(
      0.20, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Vector3d theta(0.2, 0.3, -0.1);

  // Check: (q boxplus u)
  Eigen::Quaterniond q_rot;
  Eigen::Quaterniond q_rot_ceres;
  Eigen::Vector3d u_new;
  common::eigen_quaternion_helpers::Plus(q.coeffs(), theta, &q_rot);

  ceres_error_terms::EigenQuaternionParameterization
      ceres_quaternion_eigen_param;

  ceres_quaternion_eigen_param.Plus(
      q.coeffs().data(), theta.data(), q_rot_ceres.coeffs().data());

  pose::Quaternion q_rot_(q_rot);
  pose::Quaternion q_rot_ceres_(q_rot_ceres);
  EXPECT_NEAR_KINDR_QUATERNION(q_rot_, q_rot_ceres_, 1e-5);

  // Check: p boxminus q
  Eigen::Vector3d theta_ceres_interface;
  common::eigen_quaternion_helpers::Minus(p, q, &theta);
  ceres_quaternion_eigen_param.Minus(
      p.coeffs().data(), q.coeffs().data(), theta_ceres_interface.data());

  EXPECT_NEAR_EIGEN(theta, theta_ceres_interface, 1e-5);
}

TEST(QuaternionEigenParametrization, TestQuaternionEigen) {
  // Initial state values.
  Eigen::Quaterniond q(Eigen::Quaterniond::Identity().slerp(
      0.10, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Quaterniond p(Eigen::Quaterniond::Identity().slerp(
      0.20, Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1))));

  Eigen::Vector3d theata(0.2, 0.3, -0.1);

  // Check: (q boxplus theata) boxminus q = theata
  Eigen::Quaterniond q_rot;
  Eigen::Vector3d theta_new;
  common::eigen_quaternion_helpers::Plus(q.coeffs(), theata, &q_rot);
  common::eigen_quaternion_helpers::Minus(q_rot, q, &theta_new);

  EXPECT_NEAR_EIGEN(theata, theta_new, 1e-5);

  // Check: (q boxplus theata) boxplus -theata = q
  Eigen::Quaterniond q_orig;
  Eigen::Vector3d theta_neg = -theata;
  common::eigen_quaternion_helpers::Plus(q_rot.coeffs(), theta_neg, &q_orig);

  pose::Quaternion q_(q);
  pose::Quaternion q_orig_(q_orig);
  EXPECT_NEAR_KINDR_QUATERNION(q_, q_orig_, 1e-5);

  // Check: q boxplus (p boxminus q) = p
  Eigen::Vector3d theta_temp;
  Eigen::Quaterniond p_orig;
  common::eigen_quaternion_helpers::Minus(p, q, &theta_temp);
  common::eigen_quaternion_helpers::Plus(q.coeffs(), theta_temp, &p_orig);

  pose::Quaternion p_(p);
  pose::Quaternion p_orig_(p_orig);
  EXPECT_NEAR_KINDR_QUATERNION(p_, p_orig_, 1e-5);
}

MAPLAB_UNITTEST_ENTRYPOINT
