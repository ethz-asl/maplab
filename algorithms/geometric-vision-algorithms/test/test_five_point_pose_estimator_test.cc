#include <memory>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-fisheye.h>

#include <geometric-vision/five-point-pose-estimator.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

class VariableCameraAngle : public ::testing::TestWithParam<double> {};

INSTANTIATE_TEST_CASE_P(
    OpengvPoseEstimationFivePoint, VariableCameraAngle,
    ::testing::Values(0, M_PI / 24.0, M_PI / 8.0, -M_PI / 8.0, -M_PI / 24.0));

TEST_P(VariableCameraAngle, PinholeCameraFivePointPoseInterface) {
  opengv_pose_estimation::FivePointPoseEstimator pose_estimator;

  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;
  const double distortion_param = 0.90;
  const double fu = 200;
  const double fv = 200;
  const unsigned int ru = 640;
  const unsigned int rv = 480;
  const double cu = ru / 2;
  const double cv = rv / 2;
  Eigen::VectorXd distortion_params(1);
  distortion_params << distortion_param;
  std::shared_ptr<CameraType> camera;
  aslam::Distortion::UniquePtr distortion(
      new DistortionType(distortion_params));

  Eigen::VectorXd intrinsics(4);
  intrinsics << fu, fv, cu, cv;

  camera = std::shared_ptr<CameraType>(
      new CameraType(intrinsics, ru, rv, distortion));

  constexpr double kVariationAngle = M_PI / 36;  // 5 deg.
  Eigen::Quaterniond G_q_C_a(
      Eigen::AngleAxisd(
          GetParam() - kVariationAngle, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond G_q_C_b(
      Eigen::AngleAxisd(
          GetParam() + kVariationAngle, Eigen::Vector3d::UnitY()));
  Eigen::Matrix3d G_R_C_a = G_q_C_a.toRotationMatrix();
  Eigen::Matrix3d G_R_C_b = G_q_C_b.toRotationMatrix();
  Eigen::Vector3d G_p_C_a(1, 2.5, 3);
  Eigen::Vector3d G_p_C_b(1, 2, 3);

  const int num_of_points = 100;
  const int num_of_outliers = 10;
  Eigen::Matrix2Xd measurements_a;
  Eigen::Matrix2Xd measurements_b;
  Eigen::Matrix3Xd landmark_positions;
  measurements_a.resize(Eigen::NoChange, num_of_points + num_of_outliers);
  measurements_b.resize(Eigen::NoChange, num_of_points + num_of_outliers);
  landmark_positions.resize(Eigen::NoChange, num_of_points + num_of_outliers);
  for (int i = 0; i < num_of_points + num_of_outliers; ++i) {
    // We need to vary the depth -- plane may not be enough to recover
    // the camera pose.
    Eigen::Vector3d C_p_fa = camera->createRandomVisiblePoint(i % 5 + 1);
    Eigen::Vector2d keypoint_measurement_a;
    camera->project3(C_p_fa, &keypoint_measurement_a);
    measurements_a.col(i) = keypoint_measurement_a;

    // Add outliers.
    if (i >= num_of_points) {
      measurements_a.col(i) =
          keypoint_measurement_a +
          (Eigen::Vector2d() << (i + 10) % 8, -10).finished();
    }

    Eigen::Matrix<double, 3, 1> G_landmark_position =
        G_R_C_a * C_p_fa + G_p_C_a;

    Eigen::Vector3d C_p_fb =
        G_R_C_b.transpose() * (G_landmark_position - G_p_C_b);
    Eigen::Vector2d keypoint_measurement_b;
    camera->project3(C_p_fb, &keypoint_measurement_b);
    measurements_b.col(i) = keypoint_measurement_b;

    // Add outliers.
    if (i >= num_of_points) {
      measurements_b.col(i) = keypoint_measurement_b +
                              (Eigen::Vector2d() << (i + 5) % 7, 10).finished();
    }
  }

  Eigen::Quaterniond expected_rotation = G_q_C_a.inverse() * G_q_C_b;

  pose::Transformation estimated_transform;
  std::vector<int> inlier_matches;
  constexpr double kPixelSigma = 0.8;
  constexpr double kFocalLength = 100;
  const double kRansacThreshold = 1.0 - cos(atan(kPixelSigma / kFocalLength));
  pose_estimator.Compute(
      measurements_a, measurements_b, kRansacThreshold, 500, camera,
      &estimated_transform, &inlier_matches);

  EXPECT_GT(inlier_matches.size(), num_of_points * 0.90);

  EXPECT_NEAR_EIGEN(
      estimated_transform.getRotation().toImplementation().coeffs(),
      expected_rotation.coeffs(), 1e-2);
}

MAPLAB_UNITTEST_ENTRYPOINT
