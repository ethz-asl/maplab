#include <Eigen/Core>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <aslam/common/entrypoint.h>
#include <aslam/common/memory.h>
#include <aslam/cameras/distortion.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/common/numdiff-jacobian-tester.h>

///////////////////////////////////////////////
// Types to test
///////////////////////////////////////////////
using testing::Types;
typedef Types<aslam::RadTanDistortion,
              aslam::FisheyeDistortion,
              aslam::EquidistantDistortion> ImplementationsNoNull;

typedef Types<aslam::RadTanDistortion,
              aslam::FisheyeDistortion,
              aslam::EquidistantDistortion,
              aslam::NullDistortion> Implementations;

///////////////////////////////////////////////
// Test fixture
///////////////////////////////////////////////
template <class DistortionType>
class TestDistortions : public testing::Test {
 protected:
  TestDistortions() : distortion_(DistortionType::createTestDistortion()) {};
  virtual ~TestDistortions() {};
  typename DistortionType::Ptr distortion_;
};

template <class DistortionType>
class TestDistortionsNotNull : public TestDistortions<DistortionType> { };

TYPED_TEST_CASE(TestDistortions, Implementations);
TYPED_TEST_CASE(TestDistortionsNotNull, ImplementationsNoNull);

///////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////
TYPED_TEST(TestDistortions, DistortAndUndistortUsingInternalParametersRandom) {
  const size_t kNumSamples = 1e5;
  for (int i = 0; i < kNumSamples; ++i) {
    Eigen::Vector2d keypoint = 5 * Eigen::Vector2d::Random();
    Eigen::Vector2d keypoint2 = keypoint;

    this->distortion_->distort(&keypoint2);
    this->distortion_->undistort(&keypoint2);

    EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint2, keypoint, 1e-5));
  }
}

TYPED_TEST(TestDistortions, DistortAndUndistortUsingInternalParameters) {
  // Box on the normalized image plane corresponds to
  // a pinhole camera with a resolution of 2000x2000, f=200 and c=1000.
  static constexpr double box_side = 5.0;
  static constexpr double step_size = 0.1;

  for (double u = -box_side / 2.0; u < box_side / 2.0; u += step_size) {
    for (double v = -box_side / 2.0; v < box_side / 2.0; v += step_size) {
      Eigen::Vector2d keypoint(u, v);
      Eigen::Vector2d keypoint2 = keypoint;
      this->distortion_->distort(&keypoint2);
      this->distortion_->undistort(&keypoint2);

      EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint2, keypoint, 1e-5));
    }
  }
}

TYPED_TEST(TestDistortions, DistortAndUndistortUsingExternalParameters) {
  // Set new parameters.
  Eigen::VectorXd dist_coeff = this->distortion_->getParameters();
  this->distortion_->setParameters(dist_coeff / 2.0);

  // Box on the normalized image plane corresponds to
  // a pinhole camera with a resolution of 2000x2000, f=200 and c=1000.
  static constexpr double box_side = 5.0;
  static constexpr double step_size = 0.1;

  for (double u = -box_side / 2.0; u < box_side / 2.0; u += step_size) {
    for (double v = -box_side / 2.0; v < box_side / 2.0; v += step_size) {
      Eigen::Vector2d keypoint(u, v);
      Eigen::Vector2d keypoint2 = keypoint;

      this->distortion_->distortUsingExternalCoefficients(&dist_coeff, &keypoint2, nullptr);
      this->distortion_->undistortUsingExternalCoefficients(dist_coeff, &keypoint2);

      EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint2, keypoint, 1e-5));
    }
  }
}

TYPED_TEST(TestDistortions, DistortAndUndistortImageCenter) {
  Eigen::Vector2d keypoint(0.0, 0.0);

  Eigen::Vector2d keypoint2 = keypoint;
  this->distortion_->undistort(&keypoint2);
  this->distortion_->distort(&keypoint2);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint2, keypoint, 1e-12));

  keypoint2 = keypoint;
  this->distortion_->distort(&keypoint2);
  this->distortion_->undistort(&keypoint2);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(keypoint2, keypoint, 1e-12));
}


/// Wrapper that brings the distortion function to the form needed by the differentiator.
struct Point3dJacobianFunctor : public aslam::common::NumDiffFunctor<2, 2> {

  Point3dJacobianFunctor(const aslam::Distortion& distortion, const Eigen::VectorXd& dist_coeffs)
      : distortion_(distortion),
        dist_coeffs_(dist_coeffs){};

  virtual ~Point3dJacobianFunctor() {};

  virtual bool functional(const typename NumDiffFunctor::InputType& x,
                          typename NumDiffFunctor::ValueType& fvec,
                          typename NumDiffFunctor::JacobianType* Jout) const {
    Eigen::Vector2d out_keypoint = x;

    // Get function value and Jacobian
    if(Jout)
      distortion_.distortUsingExternalCoefficients(&dist_coeffs_, &out_keypoint, Jout);
    else
      distortion_.distortUsingExternalCoefficients(&dist_coeffs_, &out_keypoint, nullptr);

    fvec = out_keypoint;
    return true;
  };

  const aslam::Distortion& distortion_;
  const Eigen::VectorXd dist_coeffs_;
};

TYPED_TEST(TestDistortions, JacobianWrtKeypoint) {
  Eigen::Vector2d keypoint(0.3, -0.2);
  Eigen::VectorXd dist_coeffs = this->distortion_->getParameters();

  TEST_JACOBIAN_FINITE_DIFFERENCE(Point3dJacobianFunctor, keypoint, 1e-5, 1e-4,
                                  *(this->distortion_), dist_coeffs);
}

/// Wrapper that brings the distortion function to the form needed by the differentiator.
template<int numDistortion>
struct DistortionJacobianFunctor : public aslam::common::NumDiffFunctor<2, numDistortion> {

  DistortionJacobianFunctor(const aslam::Distortion& distortion, const Eigen::Vector2d& keypoint)
      : distortion_(distortion),
        keypoint_(keypoint) {};

  virtual ~DistortionJacobianFunctor() {};

  virtual bool functional(
      const typename aslam::common::NumDiffFunctor<2, numDistortion>::InputType& x,
      typename aslam::common::NumDiffFunctor<2, numDistortion>::ValueType& fvec,
      typename aslam::common::NumDiffFunctor<2, numDistortion>::JacobianType* Jout) const {

    Eigen::Vector2d out_keypoint = keypoint_;
    Eigen::Matrix<double, 2, Eigen::Dynamic> JoutDynamic;
    JoutDynamic.setZero();
    Eigen::Matrix<double, Eigen::Dynamic, 1> xDynamic = x;

    // Get value
    distortion_.distortUsingExternalCoefficients(&xDynamic, &out_keypoint, nullptr);

    fvec = out_keypoint;

    // Get Jacobian wrt distortion coeffs.
    if(Jout) {
      distortion_.distortParameterJacobian(&xDynamic, keypoint_, &JoutDynamic);
      (*Jout) = JoutDynamic;
    }

    return true;
  };

  const aslam::Distortion& distortion_;
  const Eigen::Vector2d keypoint_;
};

TYPED_TEST(TestDistortionsNotNull, JacobianWrtDistortion) {
  Eigen::Vector2d keypoint(0.3, -0.2);
  Eigen::VectorXd dist_coeffs = this->distortion_->getParameters();

  TEST_JACOBIAN_FINITE_DIFFERENCE(DistortionJacobianFunctor<TypeParam::parameterCount()>,
                                  dist_coeffs, 1e-5, 1e-4, *(this->distortion_), keypoint);
}

///////
// Test parameters
///////
TEST(TestParameter, testEquidistantDistortionParameters) {
  Eigen::Vector3d invalid1 = Eigen::Vector3d::Zero();
  EXPECT_FALSE(aslam::EquidistantDistortion::areParametersValid(invalid1));

  Eigen::Matrix<double, 5, 1> invalid2 = Eigen::Matrix<double, 5, 1>::Zero();
  EXPECT_FALSE(aslam::EquidistantDistortion::areParametersValid(invalid2));

  Eigen::Vector4d valid = Eigen::Vector4d::Zero();
  EXPECT_TRUE(aslam::EquidistantDistortion::areParametersValid(valid));
}

namespace aslam {
TEST(TestParameter, testFisheyeDistortionParameters) {
  Eigen::Matrix<double, 0, 1> invalid1 = Eigen::Matrix<double, 0, 1>::Zero();
  EXPECT_FALSE(FisheyeDistortion::areParametersValid(invalid1));

  Eigen::Vector2d invalid2 = Eigen::Vector2d::Zero();
  EXPECT_FALSE(FisheyeDistortion::areParametersValid(invalid2));

  /// todo(mbuerki): enable once https://github.com/ethz-asl/aslam_cv2/issues/245 is resolved.
  //Eigen::Matrix<double, 1, 1> invalid3 = Eigen::Matrix<double, 1, 1>::Zero();
  //EXPECT_FALSE(aslam::FisheyeDistortion::areParametersValid(invalid3));

  const double min_w = FisheyeDistortion::getMinValidW();
  const double max_w = FisheyeDistortion::getMaxValidW();
  double invalid_w_5 = min_w - 1e-6;
  double invalid_w_6 = max_w + 1e-6;

  Eigen::Matrix<double, 1, 1> invalid5;
  invalid5 << invalid_w_5;
  EXPECT_FALSE(FisheyeDistortion::areParametersValid(invalid5));

  invalid5(0) = invalid_w_6;
  EXPECT_FALSE(FisheyeDistortion::areParametersValid(invalid5));

  Eigen::Matrix<double, 1, 1> valid;
  valid << min_w;
  EXPECT_TRUE(FisheyeDistortion::areParametersValid(valid));

  valid(0) =  max_w;
  EXPECT_TRUE(FisheyeDistortion::areParametersValid(valid));

  valid(0) = (min_w + max_w) / 2.0;
  Eigen::Matrix<double, 1, 1> valid1;
  valid1 << min_w;
  EXPECT_TRUE(FisheyeDistortion::areParametersValid(valid));
}
}  // namespace aslam

TEST(TestParameter, testRadTanParameters) {
  Eigen::Vector3d invalid1 = Eigen::Vector3d::Zero();
  EXPECT_FALSE(aslam::RadTanDistortion::areParametersValid(invalid1));

  Eigen::Matrix<double, 5, 1> invalid2 = Eigen::Matrix<double, 5, 1>::Zero();
  EXPECT_FALSE(aslam::RadTanDistortion::areParametersValid(invalid2));

  Eigen::Vector4d valid = Eigen::Vector4d::Zero();
  EXPECT_TRUE(aslam::RadTanDistortion::areParametersValid(valid));
}

ASLAM_UNITTEST_ENTRYPOINT
