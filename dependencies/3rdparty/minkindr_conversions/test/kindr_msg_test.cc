#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/transform-2d.h>

#include "minkindr_conversions/kindr_msg.h"
#include "testing_predicates.h"

namespace tf {

const double kTestTolerance = std::numeric_limits<double>::epsilon() * 3;

TEST(KindrMsgTest, poseKindrToMsgToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();
  Eigen::Vector3d position(Eigen::Vector3d::Random());
  kindr::minimal::QuatTransformation kindr_transform(rotation, position);

  geometry_msgs::Pose msg;
  poseKindrToMsg(kindr_transform, &msg);
  kindr::minimal::QuatTransformation output_transform;
  poseMsgToKindr(msg, &output_transform);

  EXPECT_NEAR_EIGEN(
      output_transform.getRotation().toImplementation().coeffs(),
      rotation.coeffs(), kTestTolerance);
  EXPECT_NEAR_EIGEN(output_transform.getPosition(), position, kTestTolerance);
}

TEST(KindrMsgTest, poseKindr2DToMsgToKindr2D) {
  static constexpr double kTestRotationAngleRad = 0.5;
  Eigen::Rotation2D<double> rotation(kTestRotationAngleRad);
  const Eigen::Vector2d position = Eigen::Vector2d::Random();
  kindr::minimal::Transformation2D kindr_transform(rotation, position);

  geometry_msgs::Pose msg;
  poseKindr2DToMsg(kindr_transform, &msg);
  kindr::minimal::Transformation2D output_transform;
  poseMsgToKindr2D(msg, &output_transform);

  EXPECT_NEAR(
      output_transform.getRotation().angle(), rotation.angle(), kTestTolerance);
  EXPECT_NEAR_EIGEN(output_transform.getPosition(), position, kTestTolerance);
}

TEST(KindrMsgTest, poseMsgToKindr2DFailsForInvalidInputPose) {
  geometry_msgs::Pose invalid_position_msg;
  invalid_position_msg.position.z = 1.0;
  kindr::minimal::Transformation2D invalid_position_output_transform;
  EXPECT_DEATH(
      poseMsgToKindr2D(
          invalid_position_msg, &invalid_position_output_transform),
      "No proper 2D position.");

  geometry_msgs::Pose invalid_rotation_msg;
  kindr::minimal::Transformation2D invalid_rotation_output_transform;
  const kindr::minimal::RotationQuaternion invalid_2d_rotation(
      kindr::minimal::AngleAxis(0.5, 0.0, 1.0, 0.0));
  quaternionKindrToMsg(invalid_2d_rotation, &invalid_rotation_msg.orientation);
  EXPECT_DEATH(
      poseMsgToKindr2D(invalid_rotation_msg, &invalid_rotation_output_transform),
      "No proper 2D rotation.");
}

TEST(KindrMsgTest, transformKindrToMsgToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();
  Eigen::Vector3d position(Eigen::Vector3d::Random());
  kindr::minimal::QuatTransformation kindr_transform(rotation, position);

  geometry_msgs::Transform msg;
  transformKindrToMsg(kindr_transform, &msg);
  kindr::minimal::QuatTransformation output_transform;
  transformMsgToKindr(msg, &output_transform);

  EXPECT_NEAR_EIGEN(
      output_transform.getRotation().toImplementation().coeffs(),
      rotation.coeffs(), kTestTolerance);
  EXPECT_NEAR_EIGEN(output_transform.getPosition(), position, kTestTolerance);
}

TEST(KindrMsgTest, quaternionKindrToMsgToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();

  geometry_msgs::Quaternion msg;
  quaternionKindrToMsg(rotation, &msg);
  Eigen::Quaterniond output_rotation;
  quaternionMsgToKindr(msg, &output_rotation);

  EXPECT_NEAR_EIGEN(
      output_rotation.coeffs(), rotation.coeffs(), kTestTolerance);
}

TEST(KindrMsgTest, vectorKindrToMsgToKindr) {
  Eigen::Vector3d position(Eigen::Vector3d::Random());

  geometry_msgs::Vector3 msg;
  vectorKindrToMsg(position, &msg);
  Eigen::Vector3d output_position;
  vectorMsgToKindr(msg, &output_position);

  EXPECT_NEAR_EIGEN(output_position, position, kTestTolerance);
}

}  // namespace tf

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
