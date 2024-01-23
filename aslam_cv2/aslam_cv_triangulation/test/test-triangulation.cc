#include <Eigen/Eigen>
#include <gtest/gtest.h>

#include <aslam/common/entrypoint.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/triangulation/test/triangulation-fixture.h>

constexpr size_t kNumObservations = 20;


TYPED_TEST(TriangulationFixture, LinearTriangulateFromNViews) {
  this->setLandmark(kGPoint);
  this->fillMeasurements(kNumObservations);
  this->expectSuccess();
}

class TriangulationMultiviewTest : public TriangulationFixture<Vector2dList> {};

TEST_F(TriangulationMultiviewTest, linearTriangulateFromNViewsMultiCam) {
  Aligned<std::vector, Eigen::Vector2d> measurements;
  Aligned<std::vector, aslam::Transformation> T_G_I;
  Aligned<std::vector, aslam::Transformation> T_I_C;
  std::vector<size_t> measurement_camera_indices;
  Eigen::Vector3d G_point;

  // To make the test simple to write, create 2 cameras and fill observations
  // for both, then simply append the vectors together.
  const unsigned int num_cameras = 2;
  T_I_C.resize(num_cameras);

  for (size_t i = 0; i < T_I_C.size(); ++i) {
    T_I_C[i].setRandom(0.2, 0.1);
    Aligned<std::vector, Eigen::Vector2d> cam_measurements;
    Aligned<std::vector, aslam::Transformation> cam_T_G_I;
    fillObservations(kNumObservations, T_I_C[i], &cam_measurements, &cam_T_G_I);

    // Append to the end of the vectors.
    measurements.insert(measurements.end(), cam_measurements.begin(),
                        cam_measurements.end());
    T_G_I.insert(T_G_I.end(), cam_T_G_I.begin(), cam_T_G_I.end());

    // Fill in the correct size of camera indices also.
    measurement_camera_indices.resize(measurements.size(), i);
  }

  aslam::linearTriangulateFromNViewsMultiCam(measurements,
      measurement_camera_indices, T_G_I, T_I_C, &G_point);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(kGPoint, G_point, kDoubleTolerance));
}

TEST_F(TriangulationMultiviewTest, iterativeGaussNewtonTriangulateFromNViews) {
  Aligned<std::vector, Eigen::Vector2d> measurements;
  Aligned<std::vector, aslam::Transformation> T_G_B;
  Aligned<std::vector, aslam::Transformation> T_B_C;
  std::vector<size_t> measurement_camera_indices;
  Eigen::Vector3d G_point;

  // To make the test simple to write, create 1 camera and fill observations,
  // then simply append the vectors together.
  const unsigned int num_cameras = 1;
  T_B_C.resize(num_cameras);

  for (size_t i = 0; i < T_B_C.size(); ++i) {
    T_B_C[i].setRandom(0.2, 0.1);
    Aligned<std::vector, Eigen::Vector2d> cam_measurements;
    Aligned<std::vector, aslam::Transformation> cam_T_G_B;
    fillObservations(kNumObservations, T_B_C[i], &cam_measurements, &cam_T_G_B);

    // Append to the end of the vectors.
    measurements.insert(measurements.end(), cam_measurements.begin(),
                        cam_measurements.end());
    T_G_B.insert(T_G_B.end(), cam_T_G_B.begin(), cam_T_G_B.end());

    // Fill in the correct size of camera indices also.
    measurement_camera_indices.resize(measurements.size(), i);
  }

  aslam::iterativeGaussNewtonTriangulateFromNViews(measurements, T_G_B, T_B_C[0], &G_point);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(kGPoint, G_point, kDoubleTolerance));
}

TYPED_TEST(TriangulationFixture, RandomPoses) {
  constexpr size_t kNumCameraPoses = 5;
  this->setNMeasurements(kNumCameraPoses);

  // Create a landmark.
  const double depth = 5.0;
  this->setLandmark(Eigen::Vector3d(1.0, 1.0, depth));

  // Generate some random camera poses and project the landmark into it.
  constexpr double kRandomTranslationNorm = 0.1;
  constexpr double kRandomRotationAngleRad = 20 / 180.0 * M_PI;

  for (size_t pose_idx = 0; pose_idx < kNumCameraPoses; ++pose_idx) {
    this->T_G_I_[pose_idx].setRandom(kRandomTranslationNorm, kRandomRotationAngleRad);
  }

  this->inferMeasurements();
  this->expectSuccess();
}

TYPED_TEST(TriangulationFixture, TwoParallelRays) {
  constexpr size_t kNumCameraPoses = 3;
  this->setNMeasurements(kNumCameraPoses);

  // Create a landmark.
  const double depth = 5.0;
  this->setLandmark(Eigen::Vector3d(1.0, 1.0, depth));

  for (size_t pose_idx = 0; pose_idx < kNumCameraPoses; ++pose_idx) {
    this->T_G_I_[pose_idx].setIdentity();
  }

  this->inferMeasurements();
  this->expectFailue();
}

TYPED_TEST(TriangulationFixture, TwoNearParallelRays) {
  constexpr size_t kNumCameraPoses = 2;
  this->setNMeasurements(kNumCameraPoses);

  // Create a landmark.
  const double depth = 5.0;
  this->setLandmark(Eigen::Vector3d(1.0, 1.0, depth));

  // Create near parallel rays.
  aslam::Transformation noise;
  const double disparity_angle_rad = 0.1 / 180.0 * M_PI;

  const double camera_shift = std::atan(disparity_angle_rad) * depth;
  noise.setRandom(camera_shift, 0.0);
  this->T_G_I_[1] = this->T_G_I_[1] * noise;

  this->inferMeasurements();
  this->expectFailue();
}

TYPED_TEST(TriangulationFixture, CombinedParallelAndGoodRays) {
  constexpr size_t kNumCameraPoses = 3;
  this->setNMeasurements(kNumCameraPoses);

  // Create near parallel rays.
  aslam::Transformation noise;
  noise.setRandom(0.01, 0.1);
  this->T_G_I_[1] = this->T_G_I_[1] * noise;

  this->T_G_I_[2].setRandom(0.5, 0.2);

  // Create a landmark.
  const double depth = 5.0;
  this->setLandmark(Eigen::Vector3d(1.0, 1.0, depth));

  this->inferMeasurements();
  this->expectSuccess();
}

ASLAM_UNITTEST_ENTRYPOINT
