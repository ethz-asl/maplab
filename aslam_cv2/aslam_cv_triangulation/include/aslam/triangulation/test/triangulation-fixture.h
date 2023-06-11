#ifndef ASLAM_TEST_TRIANGULATION_FIXTURE_H_
#define ASLAM_TEST_TRIANGULATION_FIXTURE_H_

#include <Eigen/Eigen>
#include <gtest/gtest.h>

#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/triangulation/triangulation.h>
#include <eigen-checks/gtest.h>

constexpr double kDoubleTolerance = 1e-9;
const Eigen::Vector3d kGPoint(0, 0, 5);

typedef Aligned<std::vector, Eigen::Vector2d> Vector2dList;

void fillObservations(
    size_t n_observations,
    const aslam::Transformation& T_I_C,
    Eigen::Matrix3Xd* C_bearing_vectors,
    Aligned<std::vector, aslam::Transformation>* T_G_I) {
  CHECK_NOTNULL(C_bearing_vectors);
  CHECK_NOTNULL(T_G_I)->clear();

  Eigen::Vector3d position_start(-2,-2,-1);
  Eigen::Vector3d position_end(2,2,1);

  Eigen::Vector3d position_step((position_end - position_start) / (n_observations - 1));

  C_bearing_vectors->resize(3, n_observations);
  // Move along line from position_start to position_end.
  for(size_t i = 0; i < n_observations; ++i) {
    Eigen::Vector3d test_pos(position_start + i * position_step);
    aslam::Transformation T_G_I_current(test_pos, Eigen::Quaterniond::Identity());
    T_G_I->push_back(T_G_I_current);

    aslam::Transformation T_C_G = (T_G_I_current * T_I_C).inverse();
    C_bearing_vectors->block<3, 1>(0, i) = T_C_G.transform(kGPoint);
  }
}

void fillObservations(
    size_t n_observations, const aslam::Transformation& T_I_C,
    Aligned<std::vector, Eigen::Vector2d>* measurements,
    Aligned<std::vector, aslam::Transformation>* T_G_I) {
  CHECK_NOTNULL(measurements);
  CHECK_NOTNULL(T_G_I);
  Eigen::Matrix3Xd C_bearing_vectors;
  fillObservations(n_observations, T_I_C, &C_bearing_vectors, T_G_I);
  measurements->resize(C_bearing_vectors.cols());
  for (int i = 0; i < C_bearing_vectors.cols(); ++i) {
    (*measurements)[i] = C_bearing_vectors.block<2, 1>(0, i) / C_bearing_vectors(2, i);
  }
}

template <typename MeasurementsType>
class TriangulationFixture : public testing::Test {
 protected:
  virtual void SetUp() {
    T_I_C_.setIdentity();
  }

  void fillMeasurements(
      size_t n_observations) {
    fillObservations(n_observations, T_I_C_, &measurements_, &T_G_I_);
  }

  aslam::TriangulationResult triangulate(Eigen::Vector3d* result) const;

  void setNMeasurements(const size_t n) {
    C_bearing_measurements_.resize(3, n);
    T_G_I_.resize(n);
  }

  void setLandmark(const Eigen::Vector3d& p_G) {
    p_G_ = p_G;
  }

  void inferMeasurements(double angle_noise = 0.) {
    for (size_t i = 0; i < T_G_I_.size(); ++i) {
      // Ignoring IMU to camera transformation (set to identity in SetUp()).
      C_bearing_measurements_.block<3, 1>(0, i) =
          T_G_I_[i].inverse().transform(p_G_);
      if (angle_noise > 0.) {
        aslam::Transformation perturbation;
        perturbation.setRandom(0., angle_noise);
        C_bearing_measurements_.block<3, 1>(0, i) = perturbation *
            C_bearing_measurements_.block<3, 1>(0, i);
      }
    }
    setMeasurements(C_bearing_measurements_);
  }

  void setMeasurements(const Eigen::Matrix3Xd& measurements);

  void expectSuccess() {
    Eigen::Vector3d p_G_estimate;
    EXPECT_TRUE(triangulate(&p_G_estimate).wasTriangulationSuccessful());
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(p_G_, p_G_estimate, kDoubleTolerance));
  }

  void expectFailue() {
    Eigen::Vector3d p_G_estimate;
    EXPECT_FALSE(triangulate(&p_G_estimate).wasTriangulationSuccessful());
  }

  aslam::Transformation T_I_C_;
  MeasurementsType measurements_;
  Eigen::Matrix3Xd C_bearing_measurements_;
  Aligned<std::vector, aslam::Transformation> T_G_I_;
  Eigen::Vector3d p_G_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<>
aslam::TriangulationResult TriangulationFixture<Vector2dList>::triangulate(
    Eigen::Vector3d* result) const {
  return aslam::linearTriangulateFromNViews(measurements_, T_G_I_, T_I_C_, result);
}

template<>
aslam::TriangulationResult TriangulationFixture<Eigen::Matrix3Xd>::triangulate(
    Eigen::Vector3d* result) const {
  Eigen::Matrix3Xd G_measurements(3, measurements_.cols()),
      p_G_C(3, measurements_.cols());
  for (int i = 0; i < measurements_.cols(); ++i) {
    G_measurements.block<3, 1>(0, i) = T_G_I_[i].getRotationMatrix() *
        T_I_C_.getRotationMatrix() * measurements_.block<3, 1>(0, i);
    p_G_C.block<3, 1>(0, i) = T_G_I_[i] * T_I_C_.getPosition();
  }
  return aslam::linearTriangulateFromNViews(G_measurements, p_G_C, result);
}

template <>
void TriangulationFixture<Vector2dList>::setMeasurements(const Eigen::Matrix3Xd& measurements) {
  measurements_.resize(measurements.cols());
  for (int i = 0; i < measurements.cols(); ++i) {
    measurements_[i] = measurements.block<2, 1>(0, i) / measurements(2, i);
  }
}

template <>
void TriangulationFixture<Eigen::Matrix3Xd>::setMeasurements(const Eigen::Matrix3Xd& measurements) {
  measurements_ = measurements;
}

typedef ::testing::Types<Vector2dList, Eigen::Matrix3Xd>
TestTypes;
TYPED_TEST_CASE(TriangulationFixture, TestTypes);

#endif  // ASLAM_TEST_TRIANGULATION_FIXTURE_H_
