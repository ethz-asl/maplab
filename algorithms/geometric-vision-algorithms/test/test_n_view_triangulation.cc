#include <cmath>
#include <cstdlib>
#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <geometric-vision/linear-triangulation.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

using namespace geometric_vision;  // NOLINT

class GeometricVisionNViewParamTest
    : public ::testing::TestWithParam<Eigen::Vector3d> {
 protected:
  Eigen::Vector2d reprojectPoint(
      const pose::Transformation& G_T_C, const Eigen::Vector3d& G_p_fi) {
    const Eigen::Vector3d C_p_fi = G_T_C.inverse() * G_p_fi;
    return C_p_fi.hnormalized().head<2>();
  }
};

INSTANTIATE_TEST_CASE_P(
    GeometricVision, GeometricVisionNViewParamTest,
    ::testing::Values(
        Eigen::Vector3d(1.5, 0.0, 4.0), Eigen::Vector3d(1.1, 0.1, 3.0),
        Eigen::Vector3d(0.9, -0.05, 1.43), Eigen::Vector3d(1.2, 0.07, 2.73),
        Eigen::Vector3d(2.1, -0.05, 5.2), Eigen::Vector3d(1.8, -0.05, 2.8),
        Eigen::Vector3d(-0.2, -0.25, 1.2), Eigen::Vector3d(3.1, -1.05, 6.1)));

TEST_P(GeometricVisionNViewParamTest, NViewTriangulationTest) {
  const unsigned int number_of_views = 5;

  pose::Quaternion rotations[] = {
      pose::Quaternion(1, 0, 0, 0),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.1)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.05, Eigen::Vector3d(0.3, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.2, 0.3, 0.1)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(-0.1, Eigen::Vector3d(0.1, 1.0, 0.0)))
              .normalized())};

  pose::Position3D positions[] = {
      pose::Position3D(0, 0, 0), pose::Position3D(-3, 0, 0),
      pose::Position3D(0.85, 0.1, -0.3), pose::Position3D(-0.1, -0.05, 0.4),
      pose::Position3D(0.7, 0.3, 0.21)};

  Eigen::Vector3d G_p_fi = GetParam();

  Aligned<std::vector, Eigen::Vector2d> measurements;
  Aligned<std::vector, pose::Transformation> camera_poses;
  camera_poses.resize(number_of_views);
  measurements.resize(number_of_views);
  for (unsigned int i = 0; i < number_of_views; ++i) {
    pose::Transformation G_T_Ci(positions[i], rotations[i]);
    measurements[i] = reprojectPoint(G_T_Ci, G_p_fi);
    camera_poses[i] = G_T_Ci;
  }

  Eigen::Vector3d triangulated_point;
  LinearTriangulation triangulator;
  EXPECT_TRUE(
      triangulator.triangulateFromNormalizedNViews(
          measurements, camera_poses, &triangulated_point));
  EXPECT_NEAR_EIGEN(G_p_fi, triangulated_point, 1e-12);
}

TEST_P(GeometricVisionNViewParamTest, NoisyNViewTriangulationTest) {
  srand(1);
  const unsigned int number_of_views = 20;

  pose::Quaternion rotations[] = {
      pose::Quaternion(1, 0, 0, 0),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.1)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.05, Eigen::Vector3d(0.3, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.02, Eigen::Vector3d(-0.2, 0.3, 0.1)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(-0.1, Eigen::Vector3d(0.12, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(-0.1, Eigen::Vector3d(-0.1, 1.0, 0.2)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.01, Eigen::Vector3d(0.1, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(-0.3, Eigen::Vector3d(0.1, 1.0, 0.1)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(-1.5, Eigen::Vector3d(-0.1, 1.0, 0.3)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(-0.3, Eigen::Vector3d(0.1, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.1, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.0, Eigen::Vector3d(0.0, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.02, Eigen::Vector3d(0.1, 1.0, 0.1)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.01, Eigen::Vector3d(0.1, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.0, Eigen::Vector3d(0.1, 1.0, 0.1)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.8, Eigen::Vector3d(0.0, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.0, Eigen::Vector3d(0.1, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.5, 0.5, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(0.0, Eigen::Vector3d(0.0, 1.0, 0.0)))
              .normalized()),
      pose::Quaternion(
          Eigen::Quaterniond(
              Eigen::AngleAxisd(-0.02, Eigen::Vector3d(0.1, 1.0, 0.1)))
              .normalized())};

  pose::Position3D positions[] = {
      pose::Position3D(0, 0, 0),           pose::Position3D(-3, 0, 0),
      pose::Position3D(-1.85, 0.6, -0.3),  pose::Position3D(-0.1, -0.05, 0.4),
      pose::Position3D(0.7, 0.1, -0.05),   pose::Position3D(-3.7, -1.1, 0.05),
      pose::Position3D(-0.53, 1.3, 0.13),  pose::Position3D(-2.5, -0.3, 0.21),
      pose::Position3D(-3.8, 0.1, 1),      pose::Position3D(0.7, -0.6, 1),
      pose::Position3D(-5, 0.43, 1),       pose::Position3D(-1.7, -0.6, 1),
      pose::Position3D(-4, -0.13, 1),      pose::Position3D(-3.5, 0, 0.1),
      pose::Position3D(-3, 0.1, 0),        pose::Position3D(-1.85, 0.12, -0.32),
      pose::Position3D(-0.1, -0.25, 0.41), pose::Position3D(-0.89, 0.12, -0.05),
      pose::Position3D(-3.7, -0.3, 0.02),  pose::Position3D(2, 0.2, 0.19)};

  Eigen::Vector3d G_p_fi = GetParam();

  Aligned<std::vector, Eigen::Vector2d> measurements;
  Aligned<std::vector, pose::Transformation> camera_poses;
  camera_poses.resize(number_of_views);
  measurements.resize(number_of_views);
  for (unsigned int i = 0; i < number_of_views; ++i) {
    pose::Transformation G_T_Ci(positions[i], rotations[i]);
    measurements[i] = reprojectPoint(G_T_Ci, G_p_fi);
    measurements[i] += Eigen::Vector2d::Random() / 1e4;
    camera_poses[i] = G_T_Ci;
  }

  Eigen::Vector3d triangulated_point;
  LinearTriangulation triangulator;
  EXPECT_TRUE(
      triangulator.triangulateFromNormalizedNViews(
          measurements, camera_poses, &triangulated_point));

  // Check consistency of reprojection errors.
  for (unsigned int i = 0; i < number_of_views; ++i) {
    Eigen::Vector2d reprojected_point =
        reprojectPoint(camera_poses[i], triangulated_point);
    EXPECT_NEAR_EIGEN(
        reprojected_point, reprojectPoint(camera_poses[i], G_p_fi), 1e-3);
  }
  // Check retriangulated point coordinates.
  EXPECT_NEAR_EIGEN(G_p_fi, triangulated_point, 1e-3);
}

MAPLAB_UNITTEST_ENTRYPOINT
