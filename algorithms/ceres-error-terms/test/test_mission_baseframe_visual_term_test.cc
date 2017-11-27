#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>

#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres-error-terms/visual-error-term.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

using ceres_error_terms::VisualReprojectionError;

class PosegraphErrorTerms : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;

  virtual void SetUp() {
    zero_position_.setZero();
    unit_quaternion_.setIdentity();

    distortion_param_ = 0.0;
    fu_ = 1;
    fv_ = 1;
    res_u_ = 640;
    res_v_ = 480;
    cu_ = res_u_ / 2.0;
    cv_ = res_v_ / 2.0;

    pixel_sigma_ = 0.7;

    // Same but non-zero mission baseframes.
    landmark_mission_baseframe_ << 0, sqrt(2) / 2, 0, sqrt(2) / 2, 0.2, -0.9,
        1.1;
    keyframe_mission_baseframe_ << 0, sqrt(2) / 2, 0, sqrt(2) / 2, 0.2, -0.9,
        1.1;
  }

  void constructCamera() {
    Eigen::VectorXd distortion_parameters(1);
    distortion_parameters << distortion_param_;
    aslam::Distortion::UniquePtr distortion(
        new DistortionType(distortion_parameters));

    Eigen::VectorXd intrinsics(4);
    intrinsics << fu_, fv_, cu_, cv_;

    camera_.reset(new CameraType(intrinsics, res_u_, res_v_, distortion));
  }

  void solveProblem();
  void addResidual(
      const Eigen::Vector2d& measurement, double pixel_sigma,
      double* landmark_base_pose, double* imu_pose,
      double* imu_to_camera_orientation, double* imu_to_camera_position,
      double* landmark_position);

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;
  ceres::Solver::Options options_;

  std::shared_ptr<CameraType> camera_;

  Eigen::Vector3d zero_position_;
  Eigen::Quaternion<double> unit_quaternion_;

  double distortion_param_;
  double fu_, fv_;
  double cu_, cv_;
  double res_u_, res_v_;
  double pixel_sigma_;

  // Ordering is [orientation position] -> [xyzw xyz].
  Eigen::Matrix<double, 7, 1> landmark_mission_baseframe_;
  Eigen::Matrix<double, 7, 1> keyframe_mission_baseframe_;
};

void PosegraphErrorTerms::addResidual(
    const Eigen::Vector2d& measurement, double pixel_sigma,
    double* landmark_base_pose, double* imu_pose,
    double* imu_to_camera_orientation, double* imu_to_camera_position,
    double* landmark_position) {
  ceres::CostFunction* cost_function =
      new ceres_error_terms::VisualReprojectionError<CameraType,
                                                     DistortionType>(
          measurement, pixel_sigma,
          ceres_error_terms::visual::VisualErrorType::kGlobal, camera_.get());

  problem_.AddResidualBlock(
      cost_function, NULL, landmark_position, landmark_base_pose,
      landmark_mission_baseframe_.data(), keyframe_mission_baseframe_.data(),
      imu_pose, imu_to_camera_orientation, imu_to_camera_position,
      camera_->getParametersMutable(),
      camera_->getDistortionMutable()->getParametersMutable());

  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::JplQuaternionParameterization;
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;

  if (problem_.GetParameterization(landmark_base_pose) == NULL) {
    problem_.SetParameterization(landmark_base_pose, pose_parameterization);
  }

  if (problem_.GetParameterization(imu_pose) == NULL) {
    problem_.SetParameterization(imu_pose, pose_parameterization);
  }

  if (problem_.GetParameterization(landmark_mission_baseframe_.data()) ==
      NULL) {
    problem_.SetParameterization(
        landmark_mission_baseframe_.data(), pose_parameterization);
  }

  if (problem_.GetParameterization(keyframe_mission_baseframe_.data()) ==
      NULL) {
    problem_.SetParameterization(
        keyframe_mission_baseframe_.data(), pose_parameterization);
  }

  if (problem_.GetParameterization(imu_to_camera_orientation) == NULL) {
    problem_.SetParameterization(
        imu_to_camera_orientation, quaternion_parameterization);
  }
}

void PosegraphErrorTerms::solveProblem() {
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = false;
  options_.max_num_iterations = 1e4;
  options_.parameter_tolerance = 1e-50;
  options_.gradient_tolerance = 1e-50;
  options_.function_tolerance = 1e-50;
  ceres::Solve(options_, &problem_, &summary_);

  LOG(INFO) << summary_.BriefReport();
}

// This test verifies if starting from a ground-truth initial value will make
// the optimizer immediately stop with zero cost.
TEST_F(PosegraphErrorTerms, VisualErrorTermOnePointOneCamera) {
  Eigen::Vector2d keypoint(0, 0);

  Eigen::Vector3d landmark_base_position(0, 0, -1);
  Eigen::Quaterniond landmark_base_orientation(1, 0, 0, 0);
  Eigen::Vector3d imu_position(0, 0, -1);
  Eigen::Quaterniond imu_orientation(1, 0, 0, 0);
  Eigen::Vector3d landmark_position(0, 0, 1);

  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;
  Eigen::Matrix<double, 7, 1> imu_pose;
  imu_pose << imu_orientation.coeffs(), imu_position;

  Eigen::Vector4d intrinsics;
  intrinsics << 1, 1, 0, 0;
  constructCamera();
  camera_->setParameters(intrinsics);

  addResidual(
      keypoint, pixel_sigma_, landmark_base_pose.data(), imu_pose.data(),
      unit_quaternion_.coeffs().data(), zero_position_.data(),
      landmark_position.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());
  solveProblem();

  EXPECT_EQ(summary_.final_cost, 0.0);
  EXPECT_EQ(summary_.iterations.size(), 1u);
  EXPECT_NEAR_EIGEN(
      landmark_base_pose.tail(3), Eigen::Vector3d(0, 0, -1), 1e-15);
  EXPECT_NEAR_EIGEN(imu_pose.tail(3), Eigen::Vector3d(0, 0, -1), 1e-15);
  EXPECT_NEAR_EIGEN(landmark_position, Eigen::Vector3d(0, 0, 1), 1e-15);
}

// The test verifies if a simple problem where the position of a landmark
// gets optimized if seen by 2 cameras.
TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermOnePointTwoCamerasNoisy) {
  Eigen::Vector3d landmark_base_position(0, 0, 0);
  Eigen::Quaterniond landmark_base_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  Eigen::Vector3d landmark_position(0.2, -0.1, 0.05);

  Eigen::Vector4d intrinsics;
  intrinsics << 1, 1, cu_, cv_;
  constructCamera();
  camera_->setParameters(intrinsics);

  Eigen::Vector2d camera0_keypoint(cu_, cv_);
  Eigen::Vector3d camera0_position(0, 0, -1);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  Eigen::Vector2d camera1_keypoint(cu_, cv_);
  Eigen::Vector3d camera1_position(1, 0, 0);
  Eigen::Quaterniond camera1_orientation(sqrt(2) / 2, 0, -sqrt(2) / 2, 0);
  Eigen::Matrix<double, 7, 1> camera1_pose;
  camera1_pose << camera1_orientation.coeffs(), camera1_position;

  addResidual(
      camera0_keypoint, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark_position.data());

  addResidual(
      camera1_keypoint, pixel_sigma_, landmark_base_pose.data(),
      camera1_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark_position.data());

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(camera1_pose.data());
  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_ZERO_EIGEN(landmark_position, 1e-10);
}

// This test verifies if pinhole camera intrinsic parameters will be
// properly optimized to the ground-truth values.
TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermIntrinsicsOptimization) {
  Eigen::Vector3d landmark_base_position(0, 0, 0);
  Eigen::Quaterniond landmark_base_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  Eigen::Vector3d landmark_position(0, 0, 0);

  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  fu_ = 100;
  fv_ = 100;
  intrinsics << fu_ + 18, fv_ - 90, cu_ - 10, cv_ + 13;

  Eigen::Vector2d camera0_keypoint(cu_ + 100, cv_ - 50);
  Eigen::Vector3d camera0_position(-1, 0.5, -1);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  Eigen::Vector2d camera1_keypoint(cu_ - 120, cv_ + 30);
  Eigen::Vector3d camera1_position(1, -0.3, 1.2);
  Eigen::Quaterniond camera1_orientation(sqrt(2) / 2, 0, -sqrt(2) / 2, 0);
  Eigen::Matrix<double, 7, 1> camera1_pose;
  camera1_pose << camera1_orientation.coeffs(), camera1_position;

  addResidual(
      camera0_keypoint, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark_position.data());

  addResidual(
      camera1_keypoint, pixel_sigma_, landmark_base_pose.data(),
      camera1_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark_position.data());

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(camera1_pose.data());
  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(landmark_position.data());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_NEAR_EIGEN(intrinsics, Eigen::Vector4d(fu_, fv_, cu_, cv_), 1e-5);
}

// This test verifies if distortion parameter of fisheye distortion model will
// be properly optimized to the ground-truth value. The reference projected
// keypoint coordinates were generated using Matlab script.
TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermNonZeroDistortionOptimization) {
  Eigen::Vector3d landmark_base_position(0, 0, 0);
  Eigen::Quaterniond landmark_base_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  Eigen::Vector3d landmark_position(0, 0, 0);

  distortion_param_ = 0.95;
  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  intrinsics << 100, 100, cu_, cv_;

  // values generated  in Matlab using distortion param w = 1.0
  Eigen::Vector2d camera0_keypoint(399.139364932153, 200.430317533923);
  Eigen::Vector3d camera0_position(-1, 0.5, -1);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  addResidual(
      camera0_keypoint, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark_position.data());

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(landmark_position.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_NEAR(
      *(camera_->getDistortionMutable()->getParametersMutable()), 1.0, 1e-5);
}

// This test verifies if landmark base pose will get properly optimized
// when starting from noisy initial values.
TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermLandmarkBasePoseOptimization) {
  Eigen::Vector3d landmark_base_position(0.4, -0.3, 0.2);
  Eigen::Quaterniond landmark_base_orientation(
      0.9978051316080664, 0.024225143749034013, 0.04470367401201076,
      0.04242220263937102);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  const double landmark_offset = 0.5;
  Eigen::Matrix3d landmark_positions;
  landmark_positions.col(0) = Eigen::Vector3d(-landmark_offset, 0, 0);
  landmark_positions.col(1) = Eigen::Vector3d(landmark_offset, 0, 0);
  landmark_positions.col(2) = Eigen::Vector3d(0, landmark_offset, 0);

  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  const double focal_length = 100;
  intrinsics << focal_length, focal_length, cu_, cv_;

  Eigen::Matrix<double, 2, 3> camera_keypoints;
  camera_keypoints.col(0) =
      Eigen::Vector2d(-focal_length * landmark_offset + cu_, cv_);
  camera_keypoints.col(1) =
      Eigen::Vector2d(focal_length * landmark_offset + cu_, cv_);
  camera_keypoints.col(2) =
      Eigen::Vector2d(cu_, focal_length * landmark_offset + cv_);

  Eigen::Vector3d camera0_position(0, 0, -1);

  // ~5deg in each axis
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  for (unsigned int i = 0; i < camera_keypoints.cols(); ++i) {
    addResidual(
        camera_keypoints.col(i), pixel_sigma_, landmark_base_pose.data(),
        camera0_pose.data(), unit_quaternion_.coeffs().data(),
        zero_position_.data(), landmark_positions.col(i).data());

    problem_.SetParameterBlockConstant(landmark_positions.col(i).data());
  }

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_ZERO_EIGEN(landmark_base_pose.tail(3), 1e-4);
  EXPECT_ZERO_EIGEN(landmark_base_pose.head(3), 1e-5);
  EXPECT_NEAR(landmark_base_pose.head(4).norm(), 1.0, 1e-10);
}

// This test verifies if IMU pose will get properly optimized starting
// from noisy initial values.
TEST_F(
    PosegraphErrorTerms, MissionBaseFrameVisualErrorTermImuPoseOptimization) {
  Eigen::Vector3d landmark_base_position(0, 0, 0);
  Eigen::Quaterniond landmark_base_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  const double landmark_offset = 0.5;
  Eigen::Matrix3d landmark_positions;
  landmark_positions.col(0) = Eigen::Vector3d(-landmark_offset, 0, 0);
  landmark_positions.col(1) = Eigen::Vector3d(landmark_offset, 0, 0);
  landmark_positions.col(2) = Eigen::Vector3d(0, landmark_offset, 0);

  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  const double focal_length = 100;
  intrinsics << focal_length, focal_length, cu_, cv_;

  Eigen::Matrix<double, 2, 3> camera_keypoints;
  camera_keypoints.col(0) =
      Eigen::Vector2d(-focal_length * landmark_offset + cu_, cv_);
  camera_keypoints.col(1) =
      Eigen::Vector2d(focal_length * landmark_offset + cu_, cv_);
  camera_keypoints.col(2) =
      Eigen::Vector2d(cu_, focal_length * landmark_offset + cv_);

  Eigen::Vector3d camera0_position(0.2, 2.1, -1.5);
  // ~5deg in each axis
  Eigen::Quaterniond camera0_orientation(
      0.9978051316080664, 0.024225143749034013, 0.04470367401201076,
      0.04242220263937102);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  for (unsigned int i = 0; i < camera_keypoints.cols(); ++i) {
    addResidual(
        camera_keypoints.col(i), pixel_sigma_, landmark_base_pose.data(),
        camera0_pose.data(), unit_quaternion_.coeffs().data(),
        zero_position_.data(), landmark_positions.col(i).data());

    problem_.SetParameterBlockConstant(landmark_positions.col(i).data());
  }

  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_NEAR_EIGEN(camera0_pose.tail(3), Eigen::Vector3d(0, 0, -1), 1e-4);
  EXPECT_ZERO_EIGEN(camera0_pose.head(3), 1e-5);
  EXPECT_NEAR(camera0_pose.head(4).norm(), 1.0, 1e-10);
}

// This test verifies if noisy initial Camera to IMU transformation will get
// properly optimized to ground-truth values. Note that reference rotation is
// non-zero.
TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermCamToImuPoseOptimization) {
  Eigen::Vector3d landmark_base_position(0, 0, 0);
  Eigen::Quaterniond landmark_base_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  const double landmark_offset = 0.5;
  Eigen::Matrix3d landmark_positions;
  landmark_positions.col(0) = Eigen::Vector3d(0, 0, -landmark_offset);
  landmark_positions.col(1) = Eigen::Vector3d(0, 0, landmark_offset);
  landmark_positions.col(2) = Eigen::Vector3d(0, landmark_offset, 0);

  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  const double focal_length = 100;
  intrinsics << focal_length, focal_length, cu_, cv_;

  Eigen::Matrix<double, 2, 3> camera_keypoints;
  camera_keypoints.col(0) =
      Eigen::Vector2d(-focal_length * landmark_offset + cu_, cv_);
  camera_keypoints.col(1) =
      Eigen::Vector2d(focal_length * landmark_offset + cu_, cv_);
  camera_keypoints.col(2) =
      Eigen::Vector2d(cu_, focal_length * landmark_offset + cv_);

  Eigen::Quaterniond cam_to_imu_rot(
      0.6882920059910812, 0.013627671275108364, -0.7236496655350155,
      -0.0489853308190429);
  Eigen::Vector3d cam_to_imu_pos(0.2, 0.1, -0.3);

  Eigen::Vector3d camera0_position(1, 0, 0);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  for (unsigned int i = 0; i < camera_keypoints.cols(); ++i) {
    addResidual(
        camera_keypoints.col(i), pixel_sigma_, landmark_base_pose.data(),
        camera0_pose.data(), cam_to_imu_rot.coeffs().data(),
        cam_to_imu_pos.data(), landmark_positions.col(i).data());

    problem_.SetParameterBlockConstant(landmark_positions.col(i).data());
  }

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_ZERO_EIGEN(cam_to_imu_pos, 1e-10);
  EXPECT_NEAR_KINDR_QUATERNION(
      pose::Quaternion(cam_to_imu_rot),
      pose::Quaternion(sqrt(2) / 2, 0, -sqrt(2) / 2, 0), 1e-10);
}

// This test verifies if a non-zero landmark base pose, including noisy
// initial rotation, will get optimized to the ground-truth values. The test
// can be used to check if the coordinate transformations are right,
// if Jacobians w.r.t. landmark base pose are right and finally if quaternion
// Parameterization is properly formulated.
TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermNonzeroLandmarkBasePose) {
  Eigen::Vector3d landmark_base_position(0, 0, 0.5);
  Eigen::Quaterniond landmark_base_orientation(
      0.6882920059910812, 0.013627671275108364, -0.7236496655350155,
      -0.0489853308190429);
  landmark_base_orientation.normalize();
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  Eigen::Vector3d landmark0_position(99.5, 1, 1);
  Eigen::Vector3d landmark1_position(0, 1, 1);
  Eigen::Vector3d landmark2_position(4.5, 0, 2);

  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  intrinsics << 100, 100, cu_, cv_;

  Eigen::Vector2d camera0_keypoint0(cu_, cv_);
  Eigen::Vector2d camera0_keypoint1(cu_, cv_);
  Eigen::Vector2d camera0_keypoint2(cu_ - 20, cv_ - 20);
  Eigen::Vector3d camera0_position(-1, 1, 0);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  Eigen::Vector2d camera1_keypoint0(cu_, cv_);
  Eigen::Vector3d camera1_position(-20, 1, 100);
  Eigen::Quaterniond camera1_orientation(sqrt(2) / 2, 0, sqrt(2) / 2, 0);
  Eigen::Matrix<double, 7, 1> camera1_pose;
  camera1_pose << camera1_orientation.coeffs(), camera1_position;

  addResidual(
      camera0_keypoint0, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark0_position.data());

  addResidual(
      camera0_keypoint1, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark1_position.data());

  addResidual(
      camera0_keypoint2, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark2_position.data());

  addResidual(
      camera1_keypoint0, pixel_sigma_, landmark_base_pose.data(),
      camera1_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark0_position.data());

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(camera1_pose.data());
  problem_.SetParameterBlockConstant(landmark0_position.data());
  problem_.SetParameterBlockConstant(landmark1_position.data());
  problem_.SetParameterBlockConstant(landmark2_position.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_NEAR_KINDR_QUATERNION(
      pose::Quaternion(
          Eigen::Map<Eigen::Quaternion<double>>(
              landmark_base_pose.head(4).data())),
      pose::Quaternion(sqrt(2) / 2, 0, -sqrt(2) / 2, 0), 1e-10);
  EXPECT_NEAR(landmark_base_pose.head(4).norm(), 1.0, 1e-15);
}

TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermLandmarkMissionBaseFrameOptimization) {
  Eigen::Vector3d landmark_base_position(0, 0, 0.5);
  Eigen::Quaterniond landmark_base_orientation(sqrt(2) / 2, 0, -sqrt(2) / 2, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  Eigen::Vector3d landmark0_position(99.5, 1, 1);
  Eigen::Vector3d landmark1_position(0, 1, 1);
  Eigen::Vector3d landmark2_position(4.5, 0, 2);

  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  intrinsics << 100, 100, cu_, cv_;

  Eigen::Vector2d camera0_keypoint0(cu_, cv_);
  Eigen::Vector2d camera0_keypoint1(cu_, cv_);
  Eigen::Vector2d camera0_keypoint2(cu_ - 20, cv_ - 20);
  Eigen::Vector3d camera0_position(-1, 1, 0);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  Eigen::Vector2d camera1_keypoint0(cu_, cv_);
  Eigen::Vector3d camera1_position(-20, 1, 100);
  Eigen::Quaterniond camera1_orientation(sqrt(2) / 2, 0, sqrt(2) / 2, 0);
  Eigen::Matrix<double, 7, 1> camera1_pose;
  camera1_pose << camera1_orientation.coeffs(), camera1_position;

  Eigen::Quaterniond landmark_mission_baseframe_rot(
      Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitY()));
  landmark_mission_baseframe_ << landmark_mission_baseframe_rot.coeffs(), 0.26,
      -0.5, 1.22;

  addResidual(
      camera0_keypoint0, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark0_position.data());

  addResidual(
      camera0_keypoint1, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark1_position.data());

  addResidual(
      camera0_keypoint2, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark2_position.data());

  addResidual(
      camera1_keypoint0, pixel_sigma_, landmark_base_pose.data(),
      camera1_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark0_position.data());

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(camera1_pose.data());
  problem_.SetParameterBlockConstant(landmark0_position.data());
  problem_.SetParameterBlockConstant(landmark1_position.data());
  problem_.SetParameterBlockConstant(landmark2_position.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(keyframe_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_NEAR_EIGEN(
      landmark_mission_baseframe_, keyframe_mission_baseframe_, 1e-10);
}

TEST_F(
    PosegraphErrorTerms,
    MissionBaseFrameVisualErrorTermKeyframeMissionBaseFrameOptimization) {
  Eigen::Vector3d landmark_base_position(0, 0, 0.5);
  Eigen::Quaterniond landmark_base_orientation(sqrt(2) / 2, 0, -sqrt(2) / 2, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  Eigen::Vector3d landmark0_position(99.5, 1, 1);
  Eigen::Vector3d landmark1_position(0, 1, 1);
  Eigen::Vector3d landmark2_position(4.5, 0, 2);

  constructCamera();
  Eigen::Map<Eigen::Vector4d> intrinsics(camera_->getParametersMutable());
  intrinsics << 100, 100, cu_, cv_;

  Eigen::Vector2d camera0_keypoint0(cu_, cv_);
  Eigen::Vector2d camera0_keypoint1(cu_, cv_);
  Eigen::Vector2d camera0_keypoint2(cu_ - 20, cv_ - 20);
  Eigen::Vector3d camera0_position(-1, 1, 0);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  Eigen::Vector2d camera1_keypoint0(cu_, cv_);
  Eigen::Vector3d camera1_position(-20, 1, 100);
  Eigen::Quaterniond camera1_orientation(sqrt(2) / 2, 0, sqrt(2) / 2, 0);
  Eigen::Matrix<double, 7, 1> camera1_pose;
  camera1_pose << camera1_orientation.coeffs(), camera1_position;

  Eigen::Quaterniond keyframe_mission_baseframe_rot(
      Eigen::AngleAxisd(0.47 * M_PI, Eigen::Vector3d::UnitY()));
  keyframe_mission_baseframe_ << keyframe_mission_baseframe_rot.coeffs(), 0.26,
      -0.5, 1.22;

  addResidual(
      camera0_keypoint0, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark0_position.data());

  addResidual(
      camera0_keypoint1, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark1_position.data());

  addResidual(
      camera0_keypoint2, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark2_position.data());

  addResidual(
      camera1_keypoint0, pixel_sigma_, landmark_base_pose.data(),
      camera1_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark0_position.data());

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(camera1_pose.data());
  problem_.SetParameterBlockConstant(landmark0_position.data());
  problem_.SetParameterBlockConstant(landmark1_position.data());
  problem_.SetParameterBlockConstant(landmark2_position.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(
      camera_->getDistortionMutable()->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());
  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(landmark_mission_baseframe_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_NEAR_EIGEN(
      landmark_mission_baseframe_, keyframe_mission_baseframe_, 1e-10);
}

MAPLAB_UNITTEST_ENTRYPOINT
