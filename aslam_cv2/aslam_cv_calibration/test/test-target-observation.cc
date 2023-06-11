#include <random>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/common/entrypoint.h>
#include <aslam/common/pose-types.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>

#include "aslam/calibration/target-algorithms.h"
#include "aslam/calibration/target-aprilgrid.h"
#include "aslam/calibration/target-observation.h"

class TargetObservationTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    const aslam::calibration::TargetAprilGrid::TargetConfiguration april_config;
    april_grid = aslam::calibration::TargetAprilGrid::Ptr(
        new aslam::calibration::TargetAprilGrid(april_config));
    camera = aslam::PinholeCamera::ConstPtr(
        aslam::PinholeCamera::createTestCamera<aslam::EquidistantDistortion>());
    setTargetObservation();
  }

  aslam::calibration::TargetBase::Ptr april_grid;
  aslam::Camera::ConstPtr camera;
  aslam::Transformation T_G_C;
  aslam::calibration::TargetObservation::ConstPtr april_grid_observation;

 private:
  void setTargetObservation() {
    CHECK(april_grid);
    CHECK(camera);
    Eigen::Matrix2Xd reprojected_corners;
    Eigen::VectorXi corner_ids;
    getReprojectedCornersAndIds(&reprojected_corners, &corner_ids);
    april_grid_observation = aslam::calibration::TargetObservation::ConstPtr(
        new aslam::calibration::TargetObservation(
            april_grid, camera->imageHeight(), camera->imageWidth(), corner_ids,
            reprojected_corners));
  }

  void getReprojectedCornersAndIds(
      Eigen::Matrix2Xd* reprojected_corners, Eigen::VectorXi* corner_ids) {
    CHECK(april_grid);
    CHECK(camera);
    CHECK_NOTNULL(reprojected_corners);
    CHECK_NOTNULL(corner_ids);
    setTargetTransformation();
    const Eigen::Matrix3Xd corner_points_G = april_grid->points();
    const Eigen::Matrix3Xd corner_points_C =
        T_G_C.inverse().transformVectorized(corner_points_G);
    Eigen::Matrix2Xd corner_points_reprojected;
    std::vector<aslam::ProjectionResult> projection_results;
    Eigen::Matrix2Xd reprojected_corners_without_noise;
    camera->project3Vectorized(
        corner_points_C, &reprojected_corners_without_noise,
        &projection_results);
    CHECK(isReprojectionValid(projection_results));
    corruptWithGaussianNoise(
        reprojected_corners_without_noise, reprojected_corners);
    *corner_ids = Eigen::VectorXi(reprojected_corners->cols());
    for (int index = 0; index < corner_ids->size(); ++index) {
      (*corner_ids)(index) = index;
    }
    CHECK_EQ(reprojected_corners->cols(), corner_ids->size());
  }

  void setTargetTransformation() {
    constexpr double kDistanceFromTargetMeters = 0.5;
    T_G_C = aslam::Transformation(
        aslam::Quaternion(0.0, 1.0, 0.0, 0.0),
        aslam::Position3D(
            0.5 * april_grid->width(), 0.5 * april_grid->height(),
            kDistanceFromTargetMeters));
  }

  bool isReprojectionValid(
      const std::vector<aslam::ProjectionResult>& projection_results) const {
    for (const aslam::ProjectionResult& projection_result :
         projection_results) {
      if (!projection_result) {
        return false;
      }
    }
    return true;
  }

  void corruptWithGaussianNoise(
      const Eigen::Matrix2Xd& image_points,
      Eigen::Matrix2Xd* image_points_corrupted) {
    const size_t num_rows = image_points.rows();
    CHECK_EQ(num_rows, 2u);
    const size_t num_cols = image_points.cols();
    *CHECK_NOTNULL(image_points_corrupted) = image_points;
    std::random_device random_device{};
    std::mt19937 random_generator{random_device()};
    constexpr double kStandardDeviationPixels = 0.5;
    std::normal_distribution<> normal_distribution{0.0,
                                                   kStandardDeviationPixels};
    for (size_t row_index = 0u; row_index < num_rows; ++row_index) {
      for (size_t col_index = 0u; col_index < num_cols; ++col_index) {
        (*image_points_corrupted)(row_index, col_index) +=
            normal_distribution(random_generator);
      }
    }
  }

  void drawCornersIntoImage() {
    cv::Mat out_image(
        camera->imageHeight(), camera->imageWidth(), CV_8UC3,
        cv::Scalar(255, 255, 255));
    april_grid_observation->drawCornersIntoImage(&out_image);
    cv::imwrite("simulated_grid_corners.png", out_image);
  }
};

// Simulates a pinhole camera observing an AprilTag grid. The reprojected
// corners of the target are corrupted with Gaussian noise, and this test
// verifies that the pnp problem achieves a reasonable accuracy when estimating
// the transform of the camera w.r.t. the target origin.
TEST_F(TargetObservationTest, AprilGridPoseEstimation) {
  constexpr double kTolerancePositionMeters = 0.03;
  constexpr double kToleranceRotationRad = 0.03;
  aslam::Transformation T_G_Cest;
  ASSERT_TRUE(
      aslam::calibration::estimateTargetTransformation(
          *april_grid_observation, camera, &T_G_Cest));
  const aslam::Transformation T_C_Cest = T_G_C.inverse() * T_G_Cest;
  const double position_error_meters = T_C_Cest.getPosition().norm();
  EXPECT_LT(position_error_meters, kTolerancePositionMeters);
  VLOG(1) << "Position error of target pose estimation in meters: "
          << position_error_meters;
  const aslam::AngleAxis angle_axis(T_C_Cest.getRotation());
  const double angle_error_rad = angle_axis.angle();
  EXPECT_LT(angle_error_rad, kToleranceRotationRad);
  VLOG(1) << "Angle error of target pose estimation in radians: "
          << angle_error_rad;
}

ASLAM_UNITTEST_ENTRYPOINT
