#include <Eigen/Core>
#include <Eigen/Dense>
#include <aslam/cameras/camera-3d-lidar.h>
#include <ceres-error-terms/lidar-error-term.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <memory>

using ceres_error_terms::LidarPositionError;

class LidarTerm : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  typedef aslam::NullDistortion DistortionType;
  typedef aslam::Camera3DLidar CameraType;
  typedef LidarPositionError<CameraType, DistortionType> ErrorTerm;

  virtual void SetUp() {
    zero_position_.setZero();
    unit_quaternion_.setIdentity();

    pixel_sigma_ = 0.7;

    // Set unit rotation and zero translation to dummy pose objects.
    dummy_7d_0_ << 0, 0, 0, 1, 0, 0, 0;
    dummy_7d_1_ << 0, 0, 0, 1, 0, 0, 0;
  }

  void constructCamera() {
    Eigen::Vector4d intrinsics(
        0.0030679615, 0.0046018515, 0.289916642, 3.1415926535);
    const uint32_t image_width = 2048;
    const uint32_t image_heigth = 128;
    camera_.reset(new CameraType(intrinsics, image_width, image_heigth));
  }

  void solveProblem();
  void addResidual(
      const Eigen::Vector3d& measurement, double pixel_sigma,
      double* landmark_base_pose, double* imu_pose,
      double* camera_to_imu_orientation, double* camera_to_imu_position,
      double* landmark_position);

  ceres::Problem problem_;
  ceres::Solver::Summary summary_;
  ceres::Solver::Options options_;

  std::shared_ptr<CameraType> camera_;

  Eigen::Vector3d zero_position_;
  Eigen::Quaterniond unit_quaternion_;

  double pixel_sigma_;

  // Ordering is [orientation position] -> [xyzw xyz].
  Eigen::Matrix<double, 7, 1> dummy_7d_0_;
  Eigen::Matrix<double, 7, 1> dummy_7d_1_;
};

void LidarTerm::addResidual(
    const Eigen::Vector3d& measurement, double pixel_sigma,
    double* landmark_base_pose, double* imu_pose,
    double* camera_to_imu_orientation, double* camera_to_imu_position,
    double* landmark_position) {
  ceres::CostFunction* cost_function = new ErrorTerm(
      measurement, pixel_sigma,
      ceres_error_terms::visual::VisualErrorType::kLocalMission, camera_.get());

  problem_.AddResidualBlock(
      cost_function, NULL, landmark_position, landmark_base_pose,
      dummy_7d_0_.data(), dummy_7d_1_.data(), imu_pose,
      camera_to_imu_orientation, camera_to_imu_position,
      camera_->getParametersMutable());

  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::JplQuaternionParameterization;
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;

  // We fix the dummy parameter blocks because
  // they have no meaning.
  problem_.SetParameterBlockConstant(dummy_7d_0_.data());
  problem_.SetParameterBlockConstant(dummy_7d_1_.data());

  if (problem_.GetParameterization(landmark_base_pose) == NULL) {
    problem_.SetParameterization(landmark_base_pose, pose_parameterization);
  }

  if (problem_.GetParameterization(imu_pose) == NULL) {
    problem_.SetParameterization(imu_pose, pose_parameterization);
  }

  if (problem_.GetParameterization(camera_to_imu_orientation) == NULL) {
    problem_.SetParameterization(
        camera_to_imu_orientation, quaternion_parameterization);
  }
}

void LidarTerm::solveProblem() {
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = false;
  options_.max_num_iterations = 1e4;
  options_.parameter_tolerance = 1e-50;
  options_.gradient_tolerance = 1e-50;
  options_.function_tolerance = 1e-50;
  ceres::Solve(options_, &problem_, &summary_);

  LOG(INFO) << summary_.BriefReport();
}

// This test verifies if starting from a ground-truth inital value will make
// the optimizer immediately stop with zero cost.
TEST_F(LidarTerm, LidarErrorTermOnePointOneCamera) {
  Eigen::Vector3d measurement(0, 0, 1);

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

  constructCamera();

  addResidual(
      measurement, pixel_sigma_, landmark_base_pose.data(), imu_pose.data(),
      unit_quaternion_.coeffs().data(), zero_position_.data(),
      landmark_position.data());
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
TEST_F(LidarTerm, LidarErrorTermOnePointTwoCamerasNoisy) {
  Eigen::Vector3d landmark_base_position(0, 0, 0);
  Eigen::Quaterniond landmark_base_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> landmark_base_pose;
  landmark_base_pose << landmark_base_orientation.coeffs(),
      landmark_base_position;

  Eigen::Vector3d landmark_position(0.2, -0.1, 0.05);

  constructCamera();

  Eigen::Vector3d camera0_measurement(0, 0, 1);
  Eigen::Vector3d camera0_position(0, 0, -1);
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  Eigen::Vector3d camera1_measurement(-1, 0, 0);
  Eigen::Vector3d camera1_position(1, 0, 0);
  Eigen::Quaterniond camera1_orientation(sqrt(2) / 2, 0, -sqrt(2) / 2, 0);
  Eigen::Matrix3d camera1_rotation_matrix =
      camera1_orientation.toRotationMatrix();
  camera1_measurement = camera1_rotation_matrix.inverse() * camera1_measurement;
  Eigen::Matrix<double, 7, 1> camera1_pose;
  camera1_pose << camera1_orientation.coeffs(), camera1_position;

  addResidual(
      camera0_measurement, pixel_sigma_, landmark_base_pose.data(),
      camera0_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark_position.data());

  addResidual(
      camera1_measurement, pixel_sigma_, landmark_base_pose.data(),
      camera1_pose.data(), unit_quaternion_.coeffs().data(),
      zero_position_.data(), landmark_position.data());

  problem_.SetParameterBlockConstant(camera0_pose.data());
  // problem_.SetParameterBlockConstant(camera1_pose.data());
  problem_.SetParameterBlockConstant(landmark_base_pose.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_ZERO_EIGEN(landmark_position, 1e-10);
}

// This test verifies if landmark base pose will get properly optimized
// when starting from noisy initial values.
TEST_F(LidarTerm, LidarErrorTermLandmarkBasePoseOptimization) {
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

  Eigen::Matrix<double, 3, 3> camera_measurements;
  camera_measurements.col(0) = Eigen::Vector3d(-landmark_offset, 0, -1);
  camera_measurements.col(1) = Eigen::Vector3d(landmark_offset, 0, -1);
  camera_measurements.col(2) = Eigen::Vector3d(0, landmark_offset, -1);

  Eigen::Vector3d camera0_position(0, 0, 1);

  // ~5deg in each axis
  Eigen::Quaterniond camera0_orientation(1, 0, 0, 0);
  Eigen::Matrix<double, 7, 1> camera0_pose;
  camera0_pose << camera0_orientation.coeffs(), camera0_position;

  for (unsigned int i = 0; i < camera_measurements.cols(); ++i) {
    addResidual(
        camera_measurements.col(i), pixel_sigma_, landmark_base_pose.data(),
        camera0_pose.data(), unit_quaternion_.coeffs().data(),
        zero_position_.data(), landmark_positions.col(i).data());

    problem_.SetParameterBlockConstant(landmark_positions.col(i).data());
  }

  problem_.SetParameterBlockConstant(camera0_pose.data());
  problem_.SetParameterBlockConstant(camera_->getParametersMutable());
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_ZERO_EIGEN(landmark_base_pose.tail(3), 1e-4);
  EXPECT_ZERO_EIGEN(landmark_base_pose.head(3), 1e-5);
  EXPECT_NEAR(landmark_base_pose.head(4).norm(), 1.0, 1e-10);
}

// This test verifies if IMU pose will get properly optimized starting
// from noisy initial values.
TEST_F(LidarTerm, VisualErrorTermImuPoseOptimization) {
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

  Eigen::Matrix<double, 3, 3> camera_keypoints;
  camera_keypoints.col(0) = Eigen::Vector3d(-landmark_offset, 0, 1);
  camera_keypoints.col(1) = Eigen::Vector3d(landmark_offset, 0, 1);
  camera_keypoints.col(2) = Eigen::Vector3d(0, landmark_offset, 1);

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
  problem_.SetParameterBlockConstant(unit_quaternion_.coeffs().data());
  problem_.SetParameterBlockConstant(zero_position_.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_NEAR_EIGEN(camera0_pose.tail(3), Eigen::Vector3d(0, 0, -1), 1e-4);
  EXPECT_ZERO_EIGEN(camera0_pose.head(3), 1e-5);
  EXPECT_NEAR(camera0_pose.head(4).norm(), 1.0, 1e-10);
}

// This test verifies if noisy initial Camera to IMU transformation will get
// properly optimized to ground-truth values. Note that reference rotation is
// non-zero.
TEST_F(LidarTerm, VisualErrorTermCamToImuPoseOptimization) {
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

  Eigen::Matrix<double, 3, 3> camera_keypoints;
  camera_keypoints.col(0) = Eigen::Vector3d(-1, 0, -landmark_offset);
  camera_keypoints.col(1) = Eigen::Vector3d(-1, 0, landmark_offset);
  camera_keypoints.col(2) = Eigen::Vector3d(-1, landmark_offset, 0);

  Eigen::Quaterniond rotation_camera(sqrt(2) / 2, 0, -sqrt(2) / 2, 0);
  Eigen::Matrix3d rotation_matrix_camera = rotation_camera.toRotationMatrix();

  camera_keypoints.col(0) =
      rotation_matrix_camera.inverse() * camera_keypoints.col(0);
  camera_keypoints.col(1) =
      rotation_matrix_camera.inverse() * camera_keypoints.col(1);
  camera_keypoints.col(2) =
      rotation_matrix_camera.inverse() * camera_keypoints.col(2);

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
  problem_.SetParameterBlockConstant(camera0_pose.data());

  solveProblem();

  EXPECT_LT(summary_.final_cost, 1e-15);
  EXPECT_ZERO_EIGEN(cam_to_imu_pos, 1e-10);
  EXPECT_NEAR_KINDR_QUATERNION(
      pose::Quaternion(cam_to_imu_rot),
      pose::Quaternion(sqrt(2) / 2, 0, -sqrt(2) / 2, 0), 1e-10);
}

MAPLAB_UNITTEST_ENTRYPOINT
