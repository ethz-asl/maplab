#include <Eigen/Core>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <typeinfo>

#include <aslam/cameras/camera-3d-lidar.h>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/distortion.h>
#include <aslam/common/entrypoint.h>
#include <aslam/common/memory.h>
#include <aslam/common/numdiff-jacobian-tester.h>
#include <aslam/common/yaml-serialization.h>

///////////////////////////////////////////////
// Types to test
///////////////////////////////////////////////
template <typename Camera, typename Distortion>
struct CameraDistortion {
  typedef Camera CameraType;
  typedef Distortion DistortionType;
};

using testing::Types;
typedef Types<CameraDistortion<aslam::Camera3DLidar, aslam::NullDistortion>>
    Implementations;

///////////////////////////////////////////////
// Test fixture
///////////////////////////////////////////////
template <class CameraDistortion>
class TestCameras : public testing::Test {
 public:
  typedef typename CameraDistortion::CameraType CameraType;
  typedef typename CameraDistortion::DistortionType DistortionType;

 protected:
  TestCameras()
      : camera_(CameraType::template createTestCamera<DistortionType>()){};
  virtual ~TestCameras(){};
  typename CameraType::Ptr camera_;
};

TYPED_TEST_CASE(TestCameras, Implementations);

TYPED_TEST(TestCameras, isVisible) {
  const double ru = this->camera_->imageWidth();
  const double rv = this->camera_->imageHeight();

  Eigen::Vector2d keypoint1(0, 0);
  EXPECT_TRUE(this->camera_->isKeypointVisible(keypoint1))
      << "Keypoint1: " << keypoint1;

  Eigen::Vector2d keypoint2(ru - 1, rv - 1);
  EXPECT_TRUE(this->camera_->isKeypointVisible(keypoint2))
      << "Keypoint2: " << keypoint2;

  Eigen::Vector2d keypoint4(-1, 0);
  EXPECT_FALSE(this->camera_->isKeypointVisible(keypoint4))
      << "Keypoint4: " << keypoint4;

  Eigen::Vector2d keypoint5(-1, -1);
  EXPECT_FALSE(this->camera_->isKeypointVisible(keypoint5))
      << "Keypoint5: " << keypoint5;

  Eigen::Vector3d keypoint6(ru, rv, 1);
  EXPECT_FALSE(this->camera_->isKeypointVisible(keypoint6.head<2>()))
      << "Keypoint6: " << keypoint6;
}

TYPED_TEST(TestCameras, isVisibleWithMargin) {
  const double ru = this->camera_->imageWidth();
  const double rv = this->camera_->imageHeight();
  const double margin = 5.0;

  Eigen::Vector2d keypoint1(margin, margin);
  EXPECT_TRUE(this->camera_->isKeypointVisibleWithMargin(keypoint1, margin))
      << "Keypoint1: " << keypoint1;

  Eigen::Vector2d keypoint2(ru - 1 - margin, rv - 1 - margin);
  EXPECT_TRUE(this->camera_->isKeypointVisibleWithMargin(keypoint2, margin))
      << "Keypoint2: " << keypoint2;

  Eigen::Vector2d keypoint4(0, 0);
  EXPECT_FALSE(this->camera_->isKeypointVisibleWithMargin(keypoint4, margin))
      << "Keypoint4: " << keypoint4;

  Eigen::Vector2f keypoint5(ru - 1, rv - 1);
  EXPECT_FALSE(this->camera_->isKeypointVisibleWithMargin(keypoint5, 5.0f))
      << "Keypoint5: " << keypoint5;

  Eigen::Vector3i keypoint6(ru, rv, 1);
  EXPECT_FALSE(
      this->camera_->isKeypointVisibleWithMargin(keypoint6.head<2>(), 5))
      << "Keypoint6: " << keypoint6;
}

TYPED_TEST(TestCameras, isProjectable) {
  EXPECT_TRUE(this->camera_->isProjectable3(
      Eigen::Vector3d(2, -1, 20)));  // Approx. upper left corner.
  EXPECT_TRUE(this->camera_->isProjectable3(
      Eigen::Vector3d(2, 1, 20)));  // Approx. lower left corner.

  EXPECT_TRUE(this->camera_->isProjectable3(
      Eigen::Vector3d(0.001, -5.734907715, 20)));  // Approx. upper left corner.
  EXPECT_TRUE(this->camera_->isProjectable3(
      Eigen::Vector3d(0.001, 5.734907715, 20)));  // Approx. lower left corner.

  EXPECT_FALSE(this->camera_->isProjectable3(
      Eigen::Vector3d(0.001, -5.734907715, 2)));  // Outside image (too high)
  EXPECT_FALSE(this->camera_->isProjectable3(
      Eigen::Vector3d(0.001, 5.734907715, 2)));  // Outside image (too low)

  EXPECT_TRUE(this->camera_->isProjectable3(
      Eigen::Vector3d(0.001, 0, 1)));  // Left border
  EXPECT_TRUE(this->camera_->isProjectable3(
      Eigen::Vector3d(-0.001, 0, 1)));  // Right border
  EXPECT_TRUE(
      this->camera_->isProjectable3(Eigen::Vector3d(0.001, 0, -1)));  // Center
  EXPECT_TRUE(
      this->camera_->isProjectable3(Eigen::Vector3d(-0.001, 0, -1)));  // Center
  EXPECT_TRUE(
      this->camera_->isProjectable3(Eigen::Vector3d(1, 0, 0)));  // Right half
  EXPECT_TRUE(
      this->camera_->isProjectable3(Eigen::Vector3d(-1, 0, 0)));  // Left half.
}

TYPED_TEST(TestCameras, CameraTest_isInvertible) {
  const int N = 100;
  const double depth = 10.0;
  Eigen::Matrix3Xd points1(3, N);
  Eigen::Matrix2Xd projections1(2, N);
  Eigen::Matrix3Xd points2(3, N);
  Eigen::Matrix3Xd points3(3, N);
  Eigen::Matrix2Xd projections3(2, N);
  Eigen::Vector3d point;
  Eigen::Vector2d keypoint;

  // N times, project and back-project a random point at a known depth.
  // Then check that the back projection matches the projection.
  for (size_t n = 0; n < N; ++n) {
    points1.col(n) = this->camera_->createRandomVisiblePoint(depth);
    if (this->camera_->project3(points1.col(n), &keypoint)
            .getDetailedStatus() !=
        aslam::ProjectionResult::Status::KEYPOINT_VISIBLE) {
      LOG(ERROR) << "Projected point " << n << " is not in the image!";
    }
    projections1.col(n) = keypoint;

    bool success = this->camera_->backProject3(keypoint, &point);
    ASSERT_TRUE(success);

    points2.col(n) = point * depth;
  }
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(points1, points2, 1e-6));

  // Do the same with the vectorized functions.
  std::vector<aslam::ProjectionResult> result;
  this->camera_->project3Vectorized(points1, &projections3, &result);
  for (size_t n = 0; n < N; ++n) {
    if (result[n].getDetailedStatus() !=
        aslam::ProjectionResult::Status::KEYPOINT_VISIBLE) {
      LOG(ERROR) << "Projected point " << n << " is not in the image!";
    }
  }
  std::vector<unsigned char> success;
  this->camera_->backProject3Vectorized(projections3, &points3, &success);
  for (size_t n = 0; n < N; ++n) {
    ASSERT_TRUE(success[n]);
    points3.col(n) *= depth;
  }
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(points1, points3, 1e-6));
}

TYPED_TEST(TestCameras, TestClone) {
  aslam::Camera::Ptr cam1(this->camera_->clone());

  EXPECT_TRUE(this->camera_.get() != nullptr);  // Check both are valid objects.
  EXPECT_TRUE(cam1.get() != nullptr);
  EXPECT_TRUE(
      this->camera_.get() !=
      cam1.get());  // Check those are two different instances.
  EXPECT_TRUE(
      *(this->camera_) ==
      *cam1);  // Check that the copy is the same as the original.

  // Check that the distortion object is copied aswell.
  EXPECT_TRUE(&this->camera_->getDistortion() != &cam1->getDistortion());
  // Check that the copy is the same as the original.
  EXPECT_TRUE(this->camera_->getDistortion() == cam1->getDistortion());
}

TYPED_TEST(TestCameras, invalidMaskTest) {
  // Die on empty mask.
  cv::Mat mask;
  EXPECT_DEATH(this->camera_->setMask(mask), "^");

  // Die on wrong type.
  mask = cv::Mat::zeros(
      cv::Size2i(this->camera_->imageWidth(), this->camera_->imageHeight()),
      CV_8UC2);
  EXPECT_DEATH(this->camera_->setMask(mask), "^");

  // Die on wrong size.
  mask = cv::Mat::zeros(
      cv::Size2i(this->camera_->imageWidth() - 1, this->camera_->imageHeight()),
      CV_8UC1);
  EXPECT_DEATH(this->camera_->setMask(mask), "^");
}

TYPED_TEST(TestCameras, validMaskTest) {
  cv::Mat mask = cv::Mat::ones(
      cv::Size2i(this->camera_->imageWidth(), this->camera_->imageHeight()),
      CV_8UC1);
  mask.at<uint8_t>(20, 10) = 0;

  EXPECT_FALSE(this->camera_->hasMask());
  this->camera_->setMask(mask);
  EXPECT_TRUE(this->camera_->hasMask());

  typedef Eigen::Vector2d Vec2;
  EXPECT_TRUE(this->camera_->isMasked(Vec2(-1., -1.)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(0., 0.)));
  EXPECT_TRUE(this->camera_->isMasked(
      Vec2(this->camera_->imageWidth(), this->camera_->imageHeight())));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(
      this->camera_->imageWidth() - 1.1, this->camera_->imageHeight() - 1.1)));
  EXPECT_FALSE(
      this->camera_->isMasked(Vec2(0.0, this->camera_->imageHeight() - 1.1)));
  EXPECT_FALSE(
      this->camera_->isMasked(Vec2(this->camera_->imageWidth() - 1.1, 0.0)));

  // Check the valid/invalid.
  EXPECT_FALSE(this->camera_->isMasked(Vec2(2.0, 2.0)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(9.0, 19.0)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(10.0, 19.0)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(10.0, 21.0)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(11.0, 21.0)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(9.0, 21.0)));
  EXPECT_TRUE(this->camera_->isMasked(Vec2(10.0, 20.0)));
  EXPECT_TRUE(this->camera_->isMasked(Vec2(10.5, 20.5)));
}

TYPED_TEST(TestCameras, YamlSerialization) {
  ASSERT_NE(this->camera_, nullptr);
  this->camera_->serializeToFile("test.yaml");
  typename TypeParam::CameraType::Ptr camera =
      aligned_shared<typename TypeParam::CameraType>();
  bool success = camera->deserializeFromFile("test.yaml");

  EXPECT_TRUE(success);
  EXPECT_TRUE(camera->isEqual(*this->camera_.get(), true));
}

ASLAM_UNITTEST_ENTRYPOINT
