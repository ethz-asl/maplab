#include "minkindr_conversions/kindr_tf.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "testing_predicates.h"

namespace tf {

const double kTestTolerance = std::numeric_limits<double>::epsilon() * 3;

TEST(KindrTfTest, transformPointKindrToTf) {
  // General test idea: evaluate how a point is transformed by each form of the
  // transformation.
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();
  Eigen::Vector3d position(Eigen::Vector3d::Random());
  kindr::minimal::QuatTransformation kindr_transform(rotation, position);

  tf::Transform tf_transform;
  transformKindrToTF(kindr_transform, &tf_transform);

  Eigen::Vector3d kindr_point(Eigen::Vector3d::Random());
  tf::Vector3 tf_point;

  vectorKindrToTF(kindr_point, &tf_point);

  tf::Vector3 tf_output = tf_transform * tf_point;
  Eigen::Vector3d kindr_output = kindr_transform * kindr_point;

  EXPECT_NEAR(kindr_output.x(), tf_output.x(), kTestTolerance);
  EXPECT_NEAR(kindr_output.y(), tf_output.y(), kTestTolerance);
  EXPECT_NEAR(kindr_output.z(), tf_output.z(), kTestTolerance);
}

TEST(KindrTfTest, transformKindrToTFToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();
  Eigen::Vector3d position(Eigen::Vector3d::Random());
  kindr::minimal::QuatTransformation kindr_transform(rotation, position);

  tf::Transform tf_transform;
  transformKindrToTF(kindr_transform, &tf_transform);
  kindr::minimal::QuatTransformation output_transform;
  transformTFToKindr(tf_transform, &output_transform);

  EXPECT_NEAR_EIGEN(output_transform.getRotation().toImplementation().coeffs(),
                    rotation.coeffs(), kTestTolerance);
  EXPECT_NEAR_EIGEN(output_transform.getPosition(), position, kTestTolerance);
}

TEST(KindrTfTest, quaternionKindrToTFToKindr) {
  Eigen::Quaterniond rotation(Eigen::Vector4d::Random());
  rotation.normalize();

  tf::Quaternion tf_quaternion;
  quaternionKindrToTF(rotation, &tf_quaternion);
  Eigen::Quaterniond output_rotation;
  quaternionTFToKindr(tf_quaternion, &output_rotation);

  EXPECT_NEAR_EIGEN(output_rotation.coeffs(), rotation.coeffs(), kTestTolerance);
}

TEST(KindrTfTest, vectorKindrToTFToKindr) {
  Eigen::Vector3d position(Eigen::Vector3d::Random());

  tf::Vector3 tf_vector;
  vectorKindrToTF(position, &tf_vector);
  Eigen::Vector3d output_position;
  vectorTFToKindr(tf_vector, &output_position);

  EXPECT_NEAR_EIGEN(output_position, position, kTestTolerance);
}

}  // namespace tf

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
