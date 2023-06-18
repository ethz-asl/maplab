#include <cstdlib>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/entrypoint.h>
#include <aslam/common/macros.h>
#include <aslam/common/pose-types.h>

#include "aslam/geometric-vision/pnp-pose-estimator.h"

class VariableCameraAngle : public ::testing::TestWithParam<double> {
 public:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

  std::shared_ptr<CameraType> createCamera();
};

std::shared_ptr<VariableCameraAngle::CameraType>
VariableCameraAngle::createCamera() {
  const double distortion_param = 0.95;
  const double fu = 200;
  const double fv = 200;
  const unsigned int ru = 640;
  const unsigned int rv = 480;
  const double cu = ru / 2;
  const double cv = rv / 2;
  std::shared_ptr<CameraType> camera;
  Eigen::VectorXd distortion_params(1);
  distortion_params << distortion_param;
  aslam::Distortion::UniquePtr distortion(
      new DistortionType(distortion_params));

  Eigen::VectorXd intrinsics(4);
  intrinsics << fu, fv, cu, cv;

  camera.reset(new CameraType(intrinsics, ru, rv, distortion));
  CHECK(camera);

  aslam::CameraId cam_id;
  generateId(&cam_id);
  camera->setId(cam_id);

  return camera;
}

INSTANTIATE_TEST_CASE_P(OpengvPoseEstimation, VariableCameraAngle,
                        ::testing::Values(0, M_PI / 6.0, M_PI / 2.0,
                                          -M_PI / 2.0, -M_PI / 6.0));

TEST_P(VariableCameraAngle, PinholeCameraP3pInterface) {
  // To make RANSAC deterministic.
  srand(1);

  // Force RANSAC to not use a random seed.
  constexpr bool kNonlinearRefinement = true;
  constexpr bool kRandomSeed = false;
  aslam::geometric_vision::PnpPoseEstimator pose_estimator(kNonlinearRefinement,
                                                           kRandomSeed);

  std::shared_ptr<CameraType> camera = createCamera();

  Eigen::Quaterniond q_G_C(
      Eigen::AngleAxisd(GetParam(), Eigen::Vector3d::UnitY()));
  Eigen::Matrix3d R_G_C = q_G_C.toRotationMatrix();
  const Eigen::Vector3d p_G_C(1, 2, 3);

  const unsigned int num_of_points = 200;
  Eigen::Matrix2Xd measurements;
  Eigen::Matrix3Xd G_landmark_positions;
  measurements.resize(Eigen::NoChange, num_of_points);
  G_landmark_positions.resize(Eigen::NoChange, num_of_points);
  for (unsigned int i = 0; i < num_of_points; ++i) {
    // We need to vary the depth -- plane may not be enough to recover
    // the camera pose.
    Eigen::Vector3d p_C_fi = camera->createRandomVisiblePoint(i + 50);
    Eigen::Vector2d keypoint_measurement;
    camera->project3(p_C_fi, &keypoint_measurement);
    measurements.col(i) = keypoint_measurement;
    G_landmark_positions.col(i) = R_G_C * p_C_fi + p_G_C;
  }
  std::vector<int> inliers;
  int num_iters;

  aslam::Transformation T_G_C;
  pose_estimator.absolutePoseRansacPinholeCam(
      measurements, G_landmark_positions, 0.8, 500, camera, &T_G_C, &inliers,
      &num_iters);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(T_G_C.getPosition(), p_G_C, 1e-5));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(T_G_C.getRotation().toImplementation().coeffs(),
                                q_G_C.coeffs(), 1e-9));
}

TEST_P(VariableCameraAngle, MultiPinholeCameraP3pInterface) {
  // To make RANSAC deterministic.
  srand(1);

  // Force RANSAC to not use a random seed.
  constexpr bool kNonlinearRefinement = true;
  constexpr bool kRandomSeed = false;
  aslam::geometric_vision::PnpPoseEstimator pose_estimator(kNonlinearRefinement,
                                                           kRandomSeed);

  // Create a complete NCamera object.
  aslam::NCameraId cam_id;
  generateId(&cam_id);

  // Cameras.
  std::vector<aslam::Camera::Ptr> cameras;
  aslam::TransformationVector T_C_Bs;

  // Create a multi-camera system.
  const int num_cams = 2;
  for (int i = 0; i < num_cams; ++i) {
    cameras.push_back(createCamera());

    // Extrinsics - have the cameras be translated and slightly rotated from
    // each other.
    aslam::Quaternion q_C_B(
        Eigen::AngleAxisd(M_PI / 6.0 * i, Eigen::Vector3d::UnitY()));
    aslam::Position3D p_C_B(2 * i - 1, i - 1, i * 5);
    T_C_Bs.emplace_back(q_C_B, p_C_B);
  }

  aslam::NCamera::Ptr ncameras(
      new aslam::NCamera(cam_id, T_C_Bs, cameras, "testrig"));

  Eigen::Quaterniond q_G_B(
      Eigen::AngleAxisd(GetParam(), Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d p_G_B(1, 2, 3);
  aslam::Transformation T_G_B(q_G_B, p_G_B);

  const unsigned int num_of_points = 800;
  const unsigned int num_of_outliers = 20;
  Eigen::Matrix2Xd measurements(2, num_of_points);
  Eigen::Matrix3Xd G_landmark_positions(3, num_of_points);
  std::vector<int> measurement_camera_indices(num_of_points);
  for (unsigned int i = 0; i < num_of_points; ++i) {
    // Same as in the single-camera case, but this time also do half the points
    // in each camera.
    int cam_index = i % num_cams;

    // We need to vary the depth -- plane may not be enough to recover
    // the camera pose.
    Eigen::Vector3d p_C_fi =
        ncameras->getCamera(cam_index).createRandomVisiblePoint(i + 50);
    Eigen::Vector2d keypoint_measurement;
    ncameras->getCamera(cam_index).project3(p_C_fi, &keypoint_measurement);
    const aslam::Transformation& T_C_B = ncameras->get_T_C_B(cam_index);
    aslam::Transformation T_G_C = T_G_B * T_C_B.inverse();

    measurement_camera_indices[i] = cam_index;
    measurements.col(i) = keypoint_measurement;
    if (i >= num_of_outliers) {
      G_landmark_positions.col(i) = T_G_C * p_C_fi;
    } else {
      // If this should be an outlier, just give it a random position instead
      // of the projected 3D position.
      G_landmark_positions.col(i) =
          T_G_C * Eigen::Vector3d(i / 10, i / 4, (i - 1) / 3);
    }
  }
  std::vector<int> inliers;
  int num_iters;

  aslam::Transformation T_G_B_out;
  pose_estimator.absoluteMultiPoseRansacPinholeCam(
      measurements, measurement_camera_indices, G_landmark_positions, 0.8, 500,
      ncameras, &T_G_B_out, &inliers, &num_iters);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(T_G_B_out.getPosition(), p_G_B, 1e-5));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(T_G_B_out.getRotation().toImplementation().coeffs(),
                                q_G_B.coeffs(), 1e-5));
  EXPECT_EQ(inliers.size(), num_of_points - num_of_outliers);
}

ASLAM_UNITTEST_ENTRYPOINT
