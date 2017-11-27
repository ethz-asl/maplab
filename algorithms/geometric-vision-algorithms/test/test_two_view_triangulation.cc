#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <geometric-vision/linear-triangulation.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

using namespace geometric_vision;  // NOLINT

class GeometricVisionTwoViewTriangulationTest
    : public ::testing::TestWithParam<std::pair<double, double> > {};  // NOLINT

TEST_P(GeometricVisionTwoViewTriangulationTest, ParallelOpticalAxes) {
  LinearTriangulation triangulator;

  pose::Transformation camera0(
      pose::Position3D(0, 0, 0), pose::Quaternion(1, 0, 0, 0));
  pose::Transformation camera1(
      pose::Position3D(1, 0, 0), pose::Quaternion(1, 0, 0, 0));

  std::pair<double, double> measurements = GetParam();
  Eigen::Vector2d measurement0(measurements.first, 0);
  Eigen::Vector2d measurement1(measurements.second, 0);

  Eigen::Vector3d triangulated_point;
  EXPECT_TRUE(
      triangulator.triangulateFromNormalizedTwoViews(
          measurement0, camera0, measurement1, camera1, &triangulated_point));

  // Calculation of expected result.
  const double b = camera0.getPosition()(0) - camera1.getPosition()(0);
  const double f = 1;
  const double Z = (b * f) / (measurement0(0) - measurement1(0));
  const double X = measurement0(0) * Z / f;
  const double Y = measurement0(1) * Z / f;
  Eigen::Vector3d expected_result(X, Y, Z);

  EXPECT_NEAR_EIGEN(expected_result, triangulated_point, 1e-15);
}

INSTANTIATE_TEST_CASE_P(
    GeometricVision, GeometricVisionTwoViewTriangulationTest,
    ::testing::Values(
        std::pair<double, double>(0.1, -0.1),
        std::pair<double, double>(0.2, -0.2),
        std::pair<double, double>(0.1, -0.2),
        std::pair<double, double>(0.05, -0.4),
        std::pair<double, double>(0.3, 0.0)));

struct TriangulationParams {
  TriangulationParams(
      double camera1_rotation_y, double camera1_x, double camera1_y,
      double camera1_z, double G_p_fi_x, double G_p_fi_y, double G_p_fi_z)
      : camera1_rotation_Y_(camera1_rotation_y),
        camera1_position_(camera1_x, camera1_y, camera1_z),
        G_p_fi_(G_p_fi_x, G_p_fi_y, G_p_fi_z) {}

  double camera1_rotation_Y_;
  Eigen::Vector3d camera1_position_;
  Eigen::Vector3d G_p_fi_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GeometricVisionRotatedTwoViewParamTest
    : public ::testing::TestWithParam<TriangulationParams> {
 protected:
  Eigen::Vector2d reprojectPoint(
      const pose::Transformation& transform, const Eigen::Vector3d& G_p_fi) {
    const Eigen::Quaterniond& G_q_C =
        transform.getRotation().toImplementation();
    Eigen::Matrix3d G_R_C = G_q_C.toRotationMatrix();

    // TODO(dymczykm) Investigate why the transformation need to be
    // inverted here
    //    const Eigen::Vector3d C_p_fi = C_R_G
    //        * (G_p_fi - transform.getPosition().toImplementation());
    const Eigen::Vector3d C_p_fi = G_R_C * G_p_fi + transform.getPosition();

    return C_p_fi.hnormalized().head<2>();
  }

  pose::Quaternion getQuaternionFromYRotation(double rotation) {
    // Invert quaternion to get passive rotation.
    return pose::Quaternion(cos(rotation / 2), 0.0, -sin(rotation / 2), 0.0);
  }
};

INSTANTIATE_TEST_CASE_P(
    GeometricVision, GeometricVisionRotatedTwoViewParamTest,
    ::testing::Values(
        TriangulationParams(0, 3, 0, 0, 1.5, 0, 5),
        TriangulationParams(0, 1, 0.05, 0, 0.5, 0.1, 5),
        TriangulationParams(0, 1.2, 0, 0.1, 0.64, 0, 3.2),
        TriangulationParams(-0.4, 1, 0.1, -0.3, 0.64, 0.05, 5.1),
        TriangulationParams(-M_PI / 2, 1, 0.0, 1.0, 0, 0, 1)));

TEST_P(
    GeometricVisionRotatedTwoViewParamTest, TwoViewTriangulationOneCamRotated) {
  LinearTriangulation triangulator;
  TriangulationParams params = GetParam();

  pose::Transformation camera0(
      pose::Position3D(0, 0, 0), pose::Quaternion(1, 0, 0, 0));
  pose::Transformation camera1(
      pose::Position3D(params.camera1_position_),
      getQuaternionFromYRotation(params.camera1_rotation_Y_));

  Eigen::Vector2d measurement0 = this->reprojectPoint(camera0, params.G_p_fi_);
  Eigen::Vector2d measurement1 = this->reprojectPoint(camera1, params.G_p_fi_);

  Eigen::Vector3d triangulated_point, triangulated_point_homog;
  EXPECT_TRUE(
      triangulator.triangulateFromNormalizedTwoViews(
          measurement0, camera0, measurement1, camera1, &triangulated_point));
  EXPECT_TRUE(
      triangulator.triangulateFromNormalizedTwoViewsHomogeneous(
          measurement0, camera0, measurement1, camera1,
          &triangulated_point_homog));
  EXPECT_NEAR_EIGEN(params.G_p_fi_, triangulated_point, 1e-12);
  EXPECT_NEAR_EIGEN(triangulated_point, triangulated_point_homog, 1e-12);
}

TEST(GeometricVision, TwoViewTriangulationParallelRays) {
  LinearTriangulation triangulator;
  pose::Transformation camera0(
      pose::Position3D(0, 0, 0), pose::Quaternion(1, 0, 0, 0));
  pose::Transformation camera1(
      pose::Position3D(1, 0, 0), pose::Quaternion(1, 0, 0, 0));

  Eigen::Vector2d measurement0(0, 0);
  Eigen::Vector2d measurement1(0, 0);

  Eigen::Vector3d triangulated_point;
  EXPECT_FALSE(
      triangulator.triangulateFromNormalizedTwoViews(
          measurement0, camera0, measurement1, camera1, &triangulated_point));
}

MAPLAB_UNITTEST_ENTRYPOINT
