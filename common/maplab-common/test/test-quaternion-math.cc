#include <aslam/common/pose-types.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "maplab-common/quaternion-math.h"

namespace common {
TEST(QuaternionMath, toRotationMatrixJPL) {
  Eigen::Vector4d quat_coeff_xyzw;
  quat_coeff_xyzw << 1.0, 0.0, 0.0, 0.0;

  Eigen::Matrix3d expected_rotation_matrix;
  expected_rotation_matrix << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;

  Eigen::Matrix3d rotation_matrix;
  common::toRotationMatrixJPL(quat_coeff_xyzw, &rotation_matrix);
  EXPECT_NEAR_EIGEN(expected_rotation_matrix, rotation_matrix, 1e-8);
}

TEST(QuaternionMath, fromRotationMatixJPL) {
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;

  Eigen::Vector4d expected_quat_coeff_xyzw;
  expected_quat_coeff_xyzw << 1.0, 0.0, 0.0, 0.0;

  Eigen::Vector4d quat_coeff_xyzw;
  common::fromRotationMatrixJPL(rotation_matrix, &quat_coeff_xyzw);
  EXPECT_NEAR_EIGEN(expected_quat_coeff_xyzw, quat_coeff_xyzw, 1e-8);
}

TEST(QuaternionMath, toFromChainJPL) {
  // Generate a random rotation matrix.
  aslam::Quaternion random_rotation;
  random_rotation.setRandom();
  Eigen::Matrix3d rotation_matrix_expected =
      random_rotation.getRotationMatrix();

  // Transform this rotation matrix to a JPL quaternion and back to the rotation
  // matrix.
  Eigen::Vector4d quat_coeffs;
  Eigen::Matrix3d rotation_matrix;
  common::fromRotationMatrixJPL(rotation_matrix_expected, &quat_coeffs);
  common::toRotationMatrixJPL(quat_coeffs, &rotation_matrix);

  EXPECT_NEAR_EIGEN(rotation_matrix_expected, rotation_matrix, 1e-8);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
