
#include <Eigen/Core>
#include <aslam/common/pose-types.h>

#include "maplab-common/interpolation-helpers.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {

TEST(MaplabCommon, TestEigenInterpolation) {
  Eigen::Vector3d vector_A;
  vector_A << 1, 2, 3;
  constexpr double kTimeA = 100.0;

  Eigen::Vector3d vector_B;
  vector_B << 4, 5, 6;
  constexpr double kTimeB = 200.0;

  const double interpolation_time = 150.0;

  Eigen::Vector3d interpolated_vector;
  linerarInterpolation(
      kTimeA, vector_A, kTimeB, vector_B, interpolation_time,
      &interpolated_vector);

  Eigen::Vector3d interpolated_vector_ground_truth;
  interpolated_vector_ground_truth << 2.5, 3.5, 4.5;

  EXPECT_EQ(interpolated_vector_ground_truth, interpolated_vector);

  // Test boundaries.
  linerarInterpolation(
      kTimeA, vector_A, kTimeB, vector_B, kTimeA, &interpolated_vector);
  EXPECT_EQ(vector_A, interpolated_vector);
  linerarInterpolation(
      kTimeA, vector_A, kTimeB, vector_B, kTimeB, &interpolated_vector);
  EXPECT_EQ(vector_B, interpolated_vector);
}

TEST(MaplabCommon, TestGenericInterpolation) {
  constexpr char kCharA = ' ';
  constexpr int kTimeA = 32;

  constexpr char kCharB = '~';
  constexpr int kTimeB = 126;

  constexpr int kInterpolationTime = 58;

  char interpolated_char;
  linerarInterpolation(
      kTimeA, kCharA, kTimeB, kCharB, kInterpolationTime, &interpolated_char);

  constexpr char kInterpolatedCharGroundTruth = ':';
  EXPECT_EQ(kInterpolatedCharGroundTruth, interpolated_char);

  // Test boundaries.
  linerarInterpolation(
      kTimeA, kCharA, kTimeB, kCharB, kTimeA, &interpolated_char);
  EXPECT_EQ(kCharA, interpolated_char);
  linerarInterpolation(
      kTimeA, kCharA, kTimeB, kCharB, kTimeB, &interpolated_char);
  EXPECT_EQ(kCharB, interpolated_char);
}

TEST(MaplabCommon, TestRotationInterpolation) {
  const double kSqrt2DividedBy2 = std::sqrt(2) / 2.0;
  aslam::Quaternion quat_A(kSqrt2DividedBy2, 0, kSqrt2DividedBy2, 0);
  quat_A.normalize();

  constexpr double kTimeA = 100.0;

  aslam::Quaternion quat_B(-kSqrt2DividedBy2, 0, kSqrt2DividedBy2, 0);
  quat_B.normalize();

  constexpr double kTimeB = 200.0;

  constexpr double kInterpolationTime = 150.0;

  aslam::Quaternion interpolated_quat;
  interpolateRotation(
      kTimeA, quat_A, kTimeB, quat_B, kInterpolationTime, &interpolated_quat);

  aslam::Quaternion interpolated_quat_ground_truth(0.0, 0.0, 1.0, 0.0);
  interpolated_quat_ground_truth.normalize();

  constexpr double kTolerance = 1e-8;
  EXPECT_NEAR(
      interpolated_quat_ground_truth.x(), interpolated_quat.x(), kTolerance);
  EXPECT_NEAR(
      interpolated_quat_ground_truth.y(), interpolated_quat.y(), kTolerance);
  EXPECT_NEAR(
      interpolated_quat_ground_truth.z(), interpolated_quat.z(), kTolerance);
  EXPECT_NEAR(
      interpolated_quat_ground_truth.w(), interpolated_quat.w(), kTolerance);

  // Test boundaries.
  interpolateRotation(
      kTimeA, quat_A, kTimeB, quat_B, kTimeA, &interpolated_quat);
  EXPECT_NEAR(quat_A.x(), interpolated_quat.x(), kTolerance);
  EXPECT_NEAR(quat_A.y(), interpolated_quat.y(), kTolerance);
  EXPECT_NEAR(quat_A.z(), interpolated_quat.z(), kTolerance);
  EXPECT_NEAR(quat_A.w(), interpolated_quat.w(), kTolerance);
  interpolateRotation(
      kTimeA, quat_A, kTimeB, quat_B, kTimeB, &interpolated_quat);
  EXPECT_NEAR(quat_B.x(), interpolated_quat.x(), kTolerance);
  EXPECT_NEAR(quat_B.y(), interpolated_quat.y(), kTolerance);
  EXPECT_NEAR(quat_B.z(), interpolated_quat.z(), kTolerance);
  EXPECT_NEAR(quat_B.w(), interpolated_quat.w(), kTolerance);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
