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

TEST(QuaternionMath, AxisAngle) {
  // Top-down view:
  //      B1
  //
  //  r   x
  //   \  ^
  //     \|
  // y<--(.)z
  //
  //
  //      B2
  //
  //          z(+)
  // y(+)x(-) /
  //      \  /
  //       \/

  // This test creates two coordinate frames B1 and B2, which are related
  // by a 90-degree rotation around a rotation axis [1.0, 1.0, 0.0]
  // expressed in frame B1.
  // It makes sure and illustrates, that the respective minkindr Axis-Angle
  // representation conforms with this (i.e., the rotation axis is expressed
  // in frame B1).

  const aslam::Transformation T_A_B1;

  Eigen::Matrix3d C_B1_B2;
  C_B1_B2 << 0.5, 0.5, sin(M_PI_4), 0.5, 0.5, -sin(M_PI_4), -sin(M_PI_4),
      sin(M_PI_4), 0.0;
  const aslam::Transformation T_B1_B2(
      aslam::Quaternion(C_B1_B2), aslam::Position3D::Zero());

  aslam::AngleAxis angle_axis_B1_B2(T_B1_B2.getRotationMatrix());
  Eigen::Vector3d B1_axis = angle_axis_B1_B2.axis();

  Eigen::Vector3d B1_axis_ground_truth(1.0, 1.0, 0.0);
  B1_axis_ground_truth.normalize();

  EXPECT_NEAR_EIGEN(B1_axis, B1_axis_ground_truth, 1e-8);
  constexpr double kAngleGroundRruthRad = M_PI_2;  // == 90 degrees.
  EXPECT_NEAR(angle_axis_B1_B2.angle(), kAngleGroundRruthRad, 1e-8);
}

class ZComponentRotationAngleTest : public ::testing::Test {
 protected:
  void testRotationsAround_B1_axis(
      const Eigen::Vector3d& B1_axis,
      const aslam::Transformation& T_A_B1) const;
  double queryAbsoluteRotationAngleAround_A_z_Axis_rad(
      const Eigen::Vector3d& B1_axis, const double rotation_angle_rad,
      const aslam::Transformation& T_A_B1) const;
  static constexpr double kTranslationNormMeters = 1e5;
};

constexpr double ZComponentRotationAngleTest::kTranslationNormMeters;

void ZComponentRotationAngleTest::testRotationsAround_B1_axis(
    const Eigen::Vector3d& B1_axis, const aslam::Transformation& T_A_B1) const {
  ASSERT_NEAR(B1_axis.squaredNorm(), 1.0, 1e-12);
  const Eigen::Vector3d A_axis = T_A_B1.getRotation().rotate(B1_axis);
  // The range boundaries are chosen to lie beyond a full 360 degree rotation
  // in both directions in order to cover all possible corner cases.
  constexpr double kMinRotationAngleDegrees = -1000.0;
  constexpr double kMaxRotationAngleDegrees = 1000.0;

  for (double rotation_angle_degrees = kMinRotationAngleDegrees;
       rotation_angle_degrees < kMaxRotationAngleDegrees;
       rotation_angle_degrees += 1.0) {
    constexpr double kDeg2Rad = M_PI / 180.0;
    const double rotation_angle_rad = rotation_angle_degrees * kDeg2Rad;

    const double rot_angle_around_A_z_axis_rad =
        queryAbsoluteRotationAngleAround_A_z_Axis_rad(
            B1_axis, rotation_angle_rad, T_A_B1);

    // Move the angle into a range of [0, 180] degrees.
    double rotation_angle_range_rad = std::fmod(rotation_angle_rad, 2.0 * M_PI);
    if (rotation_angle_range_rad < 0.0) {
      rotation_angle_range_rad += 2.0 * M_PI;
    }
    CHECK_GE(rotation_angle_range_rad, 0.0);
    CHECK_LT(rotation_angle_range_rad, 2.0 * M_PI);
    if (rotation_angle_range_rad > M_PI) {
      rotation_angle_range_rad = 2.0 * M_PI - rotation_angle_range_rad;
      // In this case, the rotation axis needs to be flipped too, but since
      // we retrieve the absolute z-component value below, this is not
      // necessary here.
    }
    CHECK_GE(rotation_angle_range_rad, 0.0);
    CHECK_LE(rotation_angle_range_rad, M_PI);

    const Eigen::Vector3d A_axis_scaled_rad = A_axis * rotation_angle_range_rad;

    // Get the absolute rotation angle within the range of [0, 180].
    const double absolute_rotation_angle_around_A_z_axis_rad =
        std::fabs(A_axis_scaled_rad[2]);
    CHECK_GE(absolute_rotation_angle_around_A_z_axis_rad, 0.0);
    CHECK_LE(absolute_rotation_angle_around_A_z_axis_rad, M_PI);
    EXPECT_NEAR(
        rot_angle_around_A_z_axis_rad,
        absolute_rotation_angle_around_A_z_axis_rad, 1e-8);
  }
}

double ZComponentRotationAngleTest::queryAbsoluteRotationAngleAround_A_z_Axis_rad(  // NOLINT
    const Eigen::Vector3d& B1_axis, const double rotation_angle_rad,
    const aslam::Transformation& T_A_B1) const {
  const aslam::AngleAxis axis_angle_B1_B2(rotation_angle_rad, B1_axis);
  const aslam::Position3D p_B1_B2 =
      aslam::Position3D::Random() * kTranslationNormMeters;
  const aslam::Transformation T_B1_B2(
      aslam::Quaternion(axis_angle_B1_B2.getRotationMatrix()), p_B1_B2);
  const aslam::Transformation T_A_B2 = T_A_B1 * T_B1_B2;

  return getAbsoluteRotationAngleAround_A_z_Axis_rad(T_A_B1, T_A_B2);
}

TEST_F(ZComponentRotationAngleTest, YawAngleSimple) {
  Eigen::Matrix3d C_A_B1;
  C_A_B1 << 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0;
  constexpr double kRotationNormRad = 2.0;

  const aslam::Position3D p_A_B1 =
      aslam::Position3D::Random() * kTranslationNormMeters;
  const aslam::Transformation T_A_B1(aslam::Quaternion(C_A_B1), p_A_B1);

  constexpr double kDeg2Rad = M_PI / 180.0;
  constexpr double kRad2Deg = 1.0 / kDeg2Rad;

  // The x-axis in frame B1 maps onto the -z axis of frame A. So any rotation
  // around the x-axis of frame B1 should translate into z-component rotation
  // in frame A.
  const Eigen::Vector3d B1_rotation_axis(1.0, 0.0, 0.0);

  // Simple case: No rotation.
  double rotation_angle_deg = 0.0;
  double absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(absolute_rotation_A_z_component_rad, 0.0, 1e-8);

  // Positive rotation in range [0, 90].
  rotation_angle_deg = 70.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg, rotation_angle_deg, 1e-8);

  // Positive rotation in range [90, 180].
  rotation_angle_deg = 124.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg, rotation_angle_deg, 1e-8);

  // Positive rotation in range [180, 360]
  rotation_angle_deg = 340.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg,
      360.0 - rotation_angle_deg, 1e-8);

  // Positive rotation in range [360, ...]
  rotation_angle_deg = 390.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg,
      rotation_angle_deg - 360.0, 1e-8);

  // Negative rotation in range [-90, 0]
  rotation_angle_deg = -55.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg, -rotation_angle_deg,
      1e-8);

  // Negative rotation in range [-180, -90]
  rotation_angle_deg = -177.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg, -rotation_angle_deg,
      1e-8);

  // Negative rotation in range [-270, -180]
  rotation_angle_deg = -185.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg,
      rotation_angle_deg + 360.0, 1e-8);

  // Negative rotation in range [-360, -270]
  rotation_angle_deg = -280.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg,
      rotation_angle_deg + 360.0, 1e-8);

  // Negative rotation in range [-..., -360]
  rotation_angle_deg = -400.0;
  absolute_rotation_A_z_component_rad =
      queryAbsoluteRotationAngleAround_A_z_Axis_rad(
          B1_rotation_axis, rotation_angle_deg * kDeg2Rad, T_A_B1);
  EXPECT_NEAR(
      absolute_rotation_A_z_component_rad * kRad2Deg,
      -rotation_angle_deg - 360.0, 1e-8);
}

TEST_F(ZComponentRotationAngleTest, YawAngle) {
  constexpr double kRotationNormRad = 2.0;

  aslam::Transformation T_A_B1;
  T_A_B1.setRandom(kTranslationNormMeters, kRotationNormRad);

  // Test all possible directions of axes.
  for (double x = -1.0; x <= 1.0; x += 0.2) {
    for (double y = -1.0; y <= 1.0; y += 0.2) {
      for (double z = -1.0; z <= 1.0; z += 0.2) {
        Eigen::Vector3d axis(x, y, z);
        axis.normalize();
        testRotationsAround_B1_axis(axis, T_A_B1);
      }
    }
  }
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
