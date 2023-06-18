#include <Eigen/Core>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <typeinfo>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/common/entrypoint.h>
#include <aslam/common/memory.h>
#include <aslam/common/numdiff-jacobian-tester.h>
#include <aslam/common/yaml-serialization.h>

///////////////////////////////////////////////
// Types to test
///////////////////////////////////////////////
template<typename Camera, typename Distortion>
struct CameraDistortion {
  typedef Camera CameraType;
  typedef Distortion DistortionType;
};

using testing::Types;
typedef Types<CameraDistortion<aslam::PinholeCamera,aslam::FisheyeDistortion>,
    CameraDistortion<aslam::UnifiedProjectionCamera,aslam::FisheyeDistortion>,
    CameraDistortion<aslam::PinholeCamera,aslam::EquidistantDistortion>,
    CameraDistortion<aslam::UnifiedProjectionCamera,aslam::EquidistantDistortion>,
    CameraDistortion<aslam::PinholeCamera,aslam::RadTanDistortion>,
    CameraDistortion<aslam::UnifiedProjectionCamera,aslam::RadTanDistortion>,
    CameraDistortion<aslam::PinholeCamera,aslam::NullDistortion>,
    CameraDistortion<aslam::UnifiedProjectionCamera,aslam::NullDistortion>>
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
    TestCameras() : camera_(CameraType::template createTestCamera<DistortionType>() ) {};
    virtual ~TestCameras() {};
    typename CameraType::Ptr camera_;
};

TYPED_TEST_CASE(TestCameras, Implementations);

///////////////////////////////////////////////
// Generic test cases (run for all models)
///////////////////////////////////////////////
struct Point3dFunctor : public aslam::common::NumDiffFunctor<2, 3> {
  Point3dFunctor(aslam::Camera::ConstPtr camera) : camera_(camera) {
    CHECK(camera);
  };

  virtual ~Point3dFunctor() {};

  bool functional(const typename NumDiffFunctor::InputType& x,
                  typename NumDiffFunctor::ValueType& fvec,
                  typename NumDiffFunctor::JacobianType* Jout) const {
    typename NumDiffFunctor::ValueType out_keypoint;
    Eigen::VectorXd dist_coeffs = camera_->getDistortion().getParameters();
    aslam::ProjectionResult res = camera_->project3Functional(x, nullptr, &dist_coeffs,
                                                              &out_keypoint, Jout, nullptr,
                                                              nullptr);
    fvec = out_keypoint;
    bool success = static_cast<bool>(res);

    if(!success) {
      LOG(ERROR) << "Point: " << x.transpose() << " does not project: " << res
          << ", keypoint: " << out_keypoint.transpose();
    }

    return success;
  };
  aslam::Camera::ConstPtr camera_;
};

TYPED_TEST(TestCameras, JacobianWrtPoint3d) {
  for (int i=0; i<10; i++) {
    Eigen::Vector3d x = this->camera_->createRandomVisiblePoint(10);
    TEST_JACOBIAN_FINITE_DIFFERENCE(Point3dFunctor, x, 1e-6, 1e-5, this->camera_);
  }
}

/// Wrapper that brings the distortion function to the form needed by the differentiator.
template<int numIntrinsics>
struct IntrinsicJacobianFunctor : public aslam::common::NumDiffFunctor<2, numIntrinsics> {

  IntrinsicJacobianFunctor(aslam::Camera::ConstPtr camera, Eigen::Vector3d point_3d)
      : camera_(camera),
        point_3d_(point_3d) {
    CHECK(camera);
  };

  virtual ~IntrinsicJacobianFunctor() {};

  virtual bool functional(
      const typename aslam::common::NumDiffFunctor<2, numIntrinsics>::InputType& x,
      typename aslam::common::NumDiffFunctor<2, numIntrinsics>::ValueType& fvec,
      typename aslam::common::NumDiffFunctor<2, numIntrinsics>::JacobianType* Jout) const {

    Eigen::VectorXd dist_coeffs = camera_->getDistortion().getParameters();

    typename aslam::common::NumDiffFunctor<2, numIntrinsics>::ValueType out_keypoint;
    Eigen::Matrix<double, 2, Eigen::Dynamic> JoutDynamic;

    aslam::ProjectionResult res;
    Eigen::Matrix<double, Eigen::Dynamic, 1> xDynamic = x;

    if (!Jout) {
      res = camera_->project3Functional(point_3d_, &xDynamic, &dist_coeffs, &out_keypoint);
    } else {
      res = camera_->project3Functional(point_3d_, &xDynamic, &dist_coeffs, &out_keypoint,
                                        nullptr, &JoutDynamic, nullptr);
      (*Jout) = JoutDynamic;
    }

    fvec = out_keypoint;
    return static_cast<bool>(res);
  };

  aslam::Camera::ConstPtr camera_;
  Eigen::Vector3d point_3d_;
};

TYPED_TEST(TestCameras, JacobianWrtIntrinsics) {
  // Test on a random point...
  Eigen::Vector3d point_3d = this->camera_->createRandomVisiblePoint(3);

  // Using the test intrinsic parameters.
  Eigen::VectorXd intrinsics = this->camera_->getParameters();

  TEST_JACOBIAN_FINITE_DIFFERENCE(IntrinsicJacobianFunctor<TypeParam::CameraType::parameterCount()>,
                                  intrinsics, 1e-8, 1e-6, this->camera_, point_3d);
}

/// Wrapper that brings the distortion function to the form needed by the differentiator.
template<int numParams>
struct DistortionJacobianFunctor : public aslam::common::NumDiffFunctor<2, numParams> {
  DistortionJacobianFunctor(aslam::Camera::ConstPtr camera, const Eigen::Vector3d& point_3d)
      : camera_(camera),
        point_3d_(point_3d) {
    CHECK(camera);
  };
  typedef aslam::common::NumDiffFunctor<2, numParams> Functor;

  virtual ~DistortionJacobianFunctor() {};

  virtual bool functional(
      const typename Functor::InputType& x,
      typename Functor::ValueType& fvec,
      typename Functor::JacobianType* Jout) const {

    Eigen::VectorXd dist_coeffs = camera_->getDistortion().getParameters();

    typename Functor::ValueType out_keypoint;

    // Convert to dynamic sized matrix types.
    Eigen::Matrix<double, 2, Eigen::Dynamic> JoutDynamic;
    Eigen::Matrix<double, Eigen::Dynamic, 1> dist_coeff_dynamic = x;

    aslam::ProjectionResult res;
    if (!Jout) {
      res = camera_->project3Functional(point_3d_, nullptr, &dist_coeff_dynamic,
                                        &out_keypoint);
    } else {
      res = camera_->project3Functional(point_3d_, nullptr, &dist_coeff_dynamic,
                                        &out_keypoint, nullptr, nullptr, &JoutDynamic);
      (*Jout) = JoutDynamic;
    }

    fvec = out_keypoint;
    return static_cast<bool>(res);
  };

  aslam::Camera::ConstPtr camera_;
  Eigen::Vector3d point_3d_;
};

TYPED_TEST(TestCameras, JacobianWrtDistortion) {
  // Test on a random point...
  Eigen::Vector3d point_3d = this->camera_->createRandomVisiblePoint(3);

  // Using the test distortion parameters.
  Eigen::VectorXd dist_coeffs = this->camera_->getDistortion().getParameters();

  TEST_JACOBIAN_FINITE_DIFFERENCE(DistortionJacobianFunctor<TypeParam::DistortionType::kNumOfParams>,
                                  dist_coeffs, 1e-8, 1e-4 , this->camera_, point_3d);

}

TYPED_TEST(TestCameras, EuclideanToOnAxisKeypoint) {
  Eigen::Vector3d euclidean(0, 0, 1);
  Eigen::Vector2d keypoint;
  this->camera_->project3(euclidean, &keypoint);

  Eigen::Vector2d image_center(this->camera_->cu(), this->camera_->cv());
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(image_center, keypoint, 1e-15));
}

TYPED_TEST(TestCameras, isVisible) {
  const double ru = this->camera_->imageWidth();
  const double rv = this->camera_->imageHeight();
  const double cu = this->camera_->cu();
  const double cv = this->camera_->cv();

  Eigen::Vector2d keypoint1(0, 0);
  EXPECT_TRUE(this->camera_->isKeypointVisible(keypoint1)) << "Keypoint1: " << keypoint1;

  Eigen::Vector2d keypoint2(ru - 1, rv - 1);
  EXPECT_TRUE(this->camera_->isKeypointVisible(keypoint2)) << "Keypoint2: " << keypoint2;

  Eigen::Vector2d keypoint3(cu, cv);
  EXPECT_TRUE(this->camera_->isKeypointVisible(keypoint3)) << "Keypoint3: " << keypoint3;

  Eigen::Vector2d keypoint4(-1, 0);
  EXPECT_FALSE(this->camera_->isKeypointVisible(keypoint4)) << "Keypoint4: " << keypoint4;

  Eigen::Vector2d keypoint5(-1, -1);
  EXPECT_FALSE(this->camera_->isKeypointVisible(keypoint5)) << "Keypoint5: " << keypoint5;

  Eigen::Vector3d keypoint6(ru, rv, 1);
  EXPECT_FALSE(this->camera_->isKeypointVisible(keypoint6.head<2>())) << "Keypoint6: " << keypoint6;
}

TYPED_TEST(TestCameras, isVisibleWithMargin) {
  const double ru = this->camera_->imageWidth();
  const double rv = this->camera_->imageHeight();
  const double cu = this->camera_->cu();
  const double cv = this->camera_->cv();
  const double margin = 5.0;

  Eigen::Vector2d keypoint1(margin, margin);
  EXPECT_TRUE(this->camera_->isKeypointVisibleWithMargin(keypoint1, margin)) << "Keypoint1: " << keypoint1;

  Eigen::Vector2d keypoint2(ru - 1 - margin, rv - 1 - margin);
  EXPECT_TRUE(this->camera_->isKeypointVisibleWithMargin(keypoint2, margin)) << "Keypoint2: " << keypoint2;

  Eigen::Vector2d keypoint3(cu, cv);
  EXPECT_TRUE(this->camera_->isKeypointVisibleWithMargin(keypoint3, margin)) << "Keypoint3: " << keypoint3;

  Eigen::Vector2d keypoint4(0, 0);
  EXPECT_FALSE(this->camera_->isKeypointVisibleWithMargin(keypoint4, margin)) << "Keypoint4: " << keypoint4;

  Eigen::Vector2f keypoint5(ru-1, rv-1);
  EXPECT_FALSE(this->camera_->isKeypointVisibleWithMargin(keypoint5, 5.0f)) << "Keypoint5: " << keypoint5;

  Eigen::Vector3i keypoint6(ru, rv, 1);
  EXPECT_FALSE(this->camera_->isKeypointVisibleWithMargin(keypoint6.head<2>(), 5)) << "Keypoint6: " << keypoint6;
}

TYPED_TEST(TestCameras, isProjectable) {
  EXPECT_TRUE(this->camera_->isProjectable3(Eigen::Vector3d(0, 0, 1)));       // Center.
  EXPECT_TRUE(this->camera_->isProjectable3(Eigen::Vector3d(5, -5, 30)));     // In front of cam and visible.
  EXPECT_FALSE(this->camera_->isProjectable3(Eigen::Vector3d(5000, -5, 1)));  // In front of cam, outside range.
  EXPECT_FALSE(this->camera_->isProjectable3(Eigen::Vector3d(-10, -10, -10))); // Behind cam.
  EXPECT_FALSE(this->camera_->isProjectable3(Eigen::Vector3d(0, 0, -1)));     // Behind, center.
}

TYPED_TEST(TestCameras, CameraTest_isInvertible) {
  const int N = 100;
  const double depth = 10.0;
  Eigen::Matrix3Xd points1(3,N);
  Eigen::Matrix2Xd projections1(2,N);
  Eigen::Matrix3Xd points2(3,N);
  Eigen::Matrix3Xd points3(3,N);
  Eigen::Matrix2Xd projections3(2,N);
  Eigen::Vector3d point;
  Eigen::Vector2d keypoint;

  // N times, project and back-project a random point at a known depth.
  // Then check that the back projection matches the projection.
  for(size_t n = 0; n < N; ++n) {
    points1.col(n) = this->camera_->createRandomVisiblePoint(depth);
    aslam::ProjectionResult result = this->camera_->project3(points1.col(n), &keypoint);
    projections1.col(n) = keypoint;
    ASSERT_EQ(aslam::ProjectionResult::Status::KEYPOINT_VISIBLE, result.getDetailedStatus());
    bool success = this->camera_->backProject3(keypoint, &point);
    ASSERT_TRUE(success);
    point.normalize();
    points2.col(n) = point * depth;
  }
  // The distortion models are not invertible so we have to be generous here.
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(points1, points2, 1e-2));

  // Do the same with the vectorized functions.
  std::vector<aslam::ProjectionResult> result;
  this->camera_->project3Vectorized(points1, &projections3, &result);
  for(size_t n = 0; n < N; ++n) {
    ASSERT_EQ(aslam::ProjectionResult::Status::KEYPOINT_VISIBLE, result[n].getDetailedStatus());
  }
  std::vector<unsigned char> success;
  this->camera_->backProject3Vectorized(projections3, &points3, &success);
  for(size_t n = 0; n < N; ++n) {
    ASSERT_TRUE(success[n]);
    points3.col(n).normalize();
    points3.col(n) *= depth;
  }
  // The distortion models are not invertible so we have to be generous here.
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(points1, points3, 1e-2));
}

TYPED_TEST(TestCameras, TestClone) {
  aslam::Camera::Ptr cam1(this->camera_->clone());

  EXPECT_TRUE(this->camera_.get() != nullptr); // Check both are valid objects.
  EXPECT_TRUE(cam1.get() != nullptr);
  EXPECT_TRUE(this->camera_.get() != cam1.get());  // Check those are two different instances.
  EXPECT_TRUE(*(this->camera_) == *cam1);  // Check that the copy is the same as the original.

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
  mask = cv::Mat::zeros(cv::Size2i(this->camera_->imageWidth(), this->camera_->imageHeight()), CV_8UC2);
  EXPECT_DEATH(this->camera_->setMask(mask), "^");

  // Die on wrong size.
  mask = cv::Mat::zeros(cv::Size2i(this->camera_->imageWidth() - 1, this->camera_->imageHeight()), CV_8UC1);
  EXPECT_DEATH(this->camera_->setMask(mask), "^");
}

TYPED_TEST(TestCameras, validMaskTest) {
  cv::Mat mask = cv::Mat::ones(cv::Size2i(this->camera_->imageWidth(), this->camera_->imageHeight()), CV_8UC1);
  mask.at<uint8_t>(20, 10) = 0;

  EXPECT_FALSE(this->camera_->hasMask());
  this->camera_->setMask(mask);
  EXPECT_TRUE(this->camera_->hasMask());

  typedef Eigen::Vector2d Vec2;
  EXPECT_TRUE(this->camera_->isMasked(Vec2(-1.,-1.)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(0.,0.)));
  EXPECT_TRUE(this->camera_->isMasked(Vec2(this->camera_->imageWidth(),
                                           this->camera_->imageHeight())));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(this->camera_->imageWidth() - 1.1,
                                            this->camera_->imageHeight() - 1.1)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(0.0,
                                            this->camera_->imageHeight() - 1.1)));
  EXPECT_FALSE(this->camera_->isMasked(Vec2(this->camera_->imageWidth() - 1.1,
                                            0.0)));

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

TYPED_TEST(TestCameras, YamlSerialization){
  ASSERT_NE(this->camera_, nullptr);
  this->camera_->serializeToFile("test.yaml");
  typename TypeParam::CameraType::Ptr camera =
      aligned_shared<typename TypeParam::CameraType>();
  bool success = camera->deserializeFromFile("test.yaml");

  EXPECT_TRUE(success);
  EXPECT_TRUE(*camera.get() == *this->camera_.get());
}

///////////////////////////////////////////////
// Model specific test cases
///////////////////////////////////////////////

TEST(TestCameraPinhole, ManualProjectionWithoutDistortion) {
  //Create camera without distortion
  aslam::PinholeCamera::Ptr camera = aslam::PinholeCamera::createTestCamera();

  double kx =  1.0,
         ky =  1.2,
         kz = 10.0;

  Eigen::Vector3d euclidean(kx, ky, kz);
  Eigen::Vector2d keypoint;

  // Manually project
  Eigen::Vector2d keypoint_manual(camera->fu() * (kx / kz) + camera->cu(),
                                  camera->fv() * (ky / kz) + camera->cv());

  // Project using the camera methods
  aslam::ProjectionResult res;
  res = camera->project3(euclidean, &keypoint);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();

  Eigen::Matrix<double, 2, 3> out_jacobian;
  res = camera->project3(euclidean, &keypoint, &out_jacobian);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();

  res = camera->project3Functional(euclidean, nullptr, nullptr, &keypoint);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();

  res = camera->project3Functional(euclidean, nullptr, nullptr, &keypoint,
                                   nullptr, nullptr, nullptr);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();
}

TEST(TestCameraUnifiedProjection, ManualProjectionWithoutDistortion) {
  //Create camera without distortion
  aslam::UnifiedProjectionCamera::Ptr camera = aslam::UnifiedProjectionCamera::createTestCamera();

  double kx =  1.0,
         ky =  1.2,
         kz = 10.0;

  Eigen::Vector3d euclidean(kx, ky, kz);
  Eigen::Vector2d keypoint;

  // Manually project
  const double rz = 1.0 / (kz + camera->xi() * euclidean.norm());
  Eigen::Vector2d keypoint_manual(camera->fu() * (kx * rz) + camera->cu(),
                                  camera->fv() * (ky * rz) + camera->cv());

  // Project using the camera methods
  aslam::ProjectionResult res;
  res = camera->project3(euclidean, &keypoint);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();

  Eigen::Matrix<double, 2, 3> out_jacobian;
  res = camera->project3(euclidean, &keypoint, &out_jacobian);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();

  res = camera->project3Functional(euclidean, nullptr, nullptr, &keypoint);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();

  res = camera->project3Functional(euclidean, nullptr,
                                   nullptr, &keypoint, nullptr, nullptr, nullptr);
  EXPECT_TRUE(res.getDetailedStatus() == aslam::ProjectionResult::KEYPOINT_VISIBLE);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint_manual, keypoint, 1e-15));
  keypoint.setZero();
}

TEST(TestCameraUnifiedProjection, ProjectionResult) {
  // Create camera without distortion.
  aslam::UnifiedProjectionCamera::Ptr cam = aslam::UnifiedProjectionCamera::createTestCamera();

  Eigen::Vector2d keypoint;
  aslam::ProjectionResult ret;

  // In front of cam -> visible.
  ret = cam->project3(Eigen::Vector3d(1, 1, 10), &keypoint);
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::KEYPOINT_VISIBLE);
  EXPECT_TRUE(static_cast<bool>(ret));

  // Outside image box.
  ret = cam->project3(Eigen::Vector3d(0.0, 10, -10), &keypoint);
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX);
  EXPECT_FALSE(static_cast<bool>(ret));

  // Invalid projection (invalid if: z <= -(fov_parameter(xi()) * d) )
  ret = cam->project3(Eigen::Vector3d(0, 0, -10), &keypoint);
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::PROJECTION_INVALID);
  EXPECT_FALSE(static_cast<bool>(ret));
}

TEST(TestCameraPinhole, ProjectionResult) {
  // Create camera without distortion.
  aslam::PinholeCamera::Ptr cam = aslam::PinholeCamera::createTestCamera();

  Eigen::Vector2d keypoint;
  aslam::ProjectionResult ret;

  // Check initialization value of aslam::ProjectionResult
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::UNINITIALIZED);

  // In front of cam -> visible.
  ret = cam->project3(Eigen::Vector3d(1, 1, 10), &keypoint);
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::KEYPOINT_VISIBLE);
  EXPECT_TRUE(static_cast<bool>(ret));

  // Behind cam -> not visible.
  ret = cam->project3(Eigen::Vector3d(1, 1, -10), &keypoint);
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::POINT_BEHIND_CAMERA);
  EXPECT_FALSE(static_cast<bool>(ret));

  // Outside image box
  ret = cam->project3(Eigen::Vector3d(1000, 1000, 10), &keypoint);
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX);
  EXPECT_FALSE(static_cast<bool>(ret));

  // Invalid projection
  ret = cam->project3(Eigen::Vector3d(1, 1, 0.0), &keypoint);
  EXPECT_EQ(ret.getDetailedStatus(), aslam::ProjectionResult::Status::PROJECTION_INVALID);
  EXPECT_FALSE(static_cast<bool>(ret));
}

TEST(CameraComparison, TestEquality) {
  aslam::Camera::Ptr pinhole_nodist = aslam::PinholeCamera::createTestCamera();
  aslam::Camera::Ptr pinhole_radtan = aslam::PinholeCamera::createTestCamera<aslam::RadTanDistortion>();
  aslam::Camera::Ptr pinhole_equi = aslam::PinholeCamera::createTestCamera<aslam::EquidistantDistortion>();

  EXPECT_TRUE(*pinhole_nodist == *pinhole_nodist);  // Same camera, should be equal.
  EXPECT_FALSE(*pinhole_radtan == *pinhole_nodist); // Different distortion model.
  EXPECT_FALSE(*pinhole_radtan == *pinhole_equi);   // Different distortion model.

  aslam::Camera::Ptr udc_nodist = aslam::UnifiedProjectionCamera::createTestCamera();
  aslam::Camera::Ptr udc_radtan = aslam::UnifiedProjectionCamera::createTestCamera<aslam::RadTanDistortion>();
  aslam::Camera::Ptr udc_fisheye = aslam::UnifiedProjectionCamera::createTestCamera<aslam::FisheyeDistortion>();

  EXPECT_FALSE(*pinhole_nodist == *udc_nodist);  // Different camera model.
  EXPECT_FALSE(*pinhole_nodist == *udc_fisheye); // Different camera and distortion model.

  aslam::PinholeCamera::Ptr pinhole_A =
      aslam::createCamera<aslam::PinholeCamera, aslam::RadTanDistortion>(
          Eigen::Vector4d(400, 400, 300, 200),  // intrinsics
          720, 480,  //resolution
          Eigen::Vector4d(0.1, 0.2, 0.3, 0.4)); // distortion coeffs

  aslam::PinholeCamera::Ptr pinhole_B =
      aslam::createCamera<aslam::PinholeCamera, aslam::RadTanDistortion>(
          Eigen::Vector4d(500, 400, 300, 200),  // intrinsics
          720, 480,  //resolution
          Eigen::Vector4d(0.1, 0.2, 0.3, 0.4)); // distortion coeffs

  aslam::PinholeCamera::Ptr pinhole_C =
      aslam::createCamera<aslam::PinholeCamera, aslam::RadTanDistortion>(
          Eigen::Vector4d(400, 400, 300, 200),  // intrinsics
          720, 480,  //resolution
          Eigen::Vector4d(0.3, 0.2, 0.3, 0.4)); // distortion coeffs

  aslam::PinholeCamera::Ptr pinhole_D =
      aslam::createCamera<aslam::PinholeCamera, aslam::RadTanDistortion>(
          Eigen::Vector4d(400, 460, 300, 200),  // intrinsics
          720, 480,  //resolution
          Eigen::Vector4d(0.4, 0.2, 0.3, 0.4)); // distortion coeffs

  EXPECT_TRUE(*pinhole_C == *pinhole_C);   // Same camera, should be equal.
  EXPECT_FALSE(*pinhole_A == *pinhole_B);  // Different intrinsics.
  EXPECT_FALSE(*pinhole_A == *pinhole_C);  // Different distortion coeffs.
  EXPECT_FALSE(*pinhole_C == *pinhole_D);  // Different intrinsics and distortion coeffs.
}

///////////////////////////////////////////////
// Test valid parameters
///////////////////////////////////////////////
TEST(TestParameters, testPinholeParameters) {
  // Check num parameters:
  Eigen::Vector3d invalid1 = Eigen::Vector3d::Zero();
  EXPECT_FALSE(aslam::PinholeCamera::areParametersValid(invalid1));

  Eigen::Matrix<double, 5, 1> invalid2 = Eigen::Matrix<double, 5, 1>::Zero();
  EXPECT_FALSE(aslam::PinholeCamera::areParametersValid(invalid2));

  Eigen::Vector4d invalid3 = Eigen::Vector4d::Zero();
  EXPECT_FALSE(aslam::PinholeCamera::areParametersValid(invalid3));

  // Check any of the paramters below 0:
  Eigen::Vector4d invalid4 = Eigen::Vector4d(0.0, 1.0, 1.0, 1.0);
  EXPECT_FALSE(aslam::PinholeCamera::areParametersValid(invalid4));

  Eigen::Vector4d invalid5 = Eigen::Vector4d(1.0, 0.0, 1.0, 1.0);
  EXPECT_FALSE(aslam::PinholeCamera::areParametersValid(invalid5));

  Eigen::Vector4d invalid6 = Eigen::Vector4d(1.0, 1.0, 0.0, 1.0);
  EXPECT_FALSE(aslam::PinholeCamera::areParametersValid(invalid6));

  Eigen::Vector4d invalid7 = Eigen::Vector4d(1.0, 1.0, 1.0, 0.0);
  EXPECT_FALSE(aslam::PinholeCamera::areParametersValid(invalid7));

  Eigen::Vector4d valid = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
  EXPECT_TRUE(aslam::PinholeCamera::areParametersValid(valid));
}

TEST(TestParameters, testUnifiedProjectionParameters) {
  // Check num parameters:
  Eigen::Vector4d invalid1 = Eigen::Vector4d::Zero();
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid1));

  Eigen::Matrix<double, 6, 1> invalid2 = Eigen::Matrix<double, 6, 1>::Zero();
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid2));

  typedef Eigen::Matrix<double, 5, 1> Intrinsics;
  Intrinsics invalid3 = Intrinsics::Zero();
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid3));

  // Check any of the paramters below 0:
  Intrinsics invalid4;
  invalid4 << -1.0, 1.0, 1.0, 1.0, 1.0;
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid4));

  Intrinsics invalid5;
  invalid5 << 1.0, 0.0, 1.0, 1.0, 1.0;
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid5));

  Intrinsics invalid6;
  invalid6 << 1.0, 1.0, 0.0, 1.0, 1.0;
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid6));

  Intrinsics invalid7;
  invalid7 << 1.0, 1.0, 1.0, 0.0, 1.0;
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid7));

  Intrinsics invalid8;
  invalid8 << 1.0, 1.0, 1.0, 1.0, 0.0;
  EXPECT_FALSE(aslam::UnifiedProjectionCamera::areParametersValid(invalid8));

  Intrinsics valid;
  valid << 0.0, 1.0, 1.0, 1.0, 1.0;
  EXPECT_TRUE(aslam::UnifiedProjectionCamera::areParametersValid(valid));
}

TEST(TestCameraFactory, testEmptyYaml) {
  YAML::Node node = YAML::Load("{}");
  aslam::Camera::Ptr camera = aslam::createCamera(node);
  EXPECT_EQ(camera, nullptr);
}

TEST(TestCameraFactory, testNoDistortionYaml) {
  YAML::Node node = YAML::Load("{type: pinhole, intrinsics: {rows: 4, cols: 1, data: [100,110,320,240]}, image_height: 480, image_width: 640}");
  aslam::Camera::Ptr camera = aslam::createCamera(node);
  EXPECT_TRUE(camera.get() != nullptr);
  EXPECT_EQ(typeid(*camera.get()), typeid(aslam::PinholeCamera));
  aslam::PinholeCamera::Ptr pincam = std::dynamic_pointer_cast<aslam::PinholeCamera>(camera);
  EXPECT_EQ(100, pincam->fu());
  EXPECT_EQ(110, pincam->fv());
  EXPECT_EQ(320, pincam->cu());
  EXPECT_EQ(240, pincam->cv());
  EXPECT_EQ(480u, pincam->imageHeight());
  EXPECT_EQ(640u, pincam->imageWidth());
  // To keep the yaml simple, if there is no id, a random id is created.
  EXPECT_TRUE(pincam->getId().isValid());
  EXPECT_EQ(0u, pincam->getLineDelayNanoSeconds());
  EXPECT_EQ(aslam::NullDistortion(), pincam->getDistortion());
}

TEST(TestCameraFactory, testBadIntrinsics1) {
  // Data size doesn't match the rows/cols
  YAML::Node node = YAML::Load("{type: pinhole, intrinsics: {rows: 4, cols: 1, data: [100,110,320]}, image_height: 480, image_width: 640}");
  aslam::Camera::Ptr camera = aslam::createCamera(node);
  EXPECT_EQ(camera, nullptr);
}

TEST(TestCameraFactory, testBadIntrinsics2) {
  // Intrinsics size doesn't match the required size.
  YAML::Node node = YAML::Load("{type: pinhole, intrinsics: {rows: 3, cols: 1, data: [100,110,320]}, image_height: 480, image_width: 640}");
  aslam::Camera::Ptr camera = aslam::createCamera(node);
  EXPECT_EQ(camera, nullptr);
}

ASLAM_UNITTEST_ENTRYPOINT
