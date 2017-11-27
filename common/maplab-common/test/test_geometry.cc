#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "maplab-common/geometry.h"
#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/test/testing-predicates.h"

namespace common {
class GeometryTransformationRansac : public ::testing::Test {
 protected:
  virtual void SetUp() {
    seed_ = 20;
  }

  void addInliers(int num_samples) {
    std::mt19937 generator(++seed_);
    std::uniform_real_distribution<> distribution_meters(-0.01, 0.01);
    std::uniform_real_distribution<> distribution_rad(-0.01, 0.01);

    for (int i = 0; i < num_samples; ++i) {
      pose::Transformation T_A_B_sample;
      T_A_B_sample.getPosition() << distribution_meters(generator),
          distribution_meters(generator), distribution_meters(generator);
      T_A_B_sample.getRotation().toImplementation() = Eigen::Quaterniond(
          Eigen::AngleAxisd(
              distribution_rad(generator), Eigen::Vector3d::UnitX()));
      T_A_B_samples.emplace_back(T_A_B_sample);
    }
  }

  void addOutlier() {
    std::mt19937 generator(++seed_);
    std::uniform_real_distribution<> distribution_meters(-5, 5);
    std::uniform_real_distribution<> distribution_rad(-0.5, 0.5);

    pose::Transformation T_A_B_sample;
    T_A_B_sample.getPosition() << distribution_meters(generator),
        distribution_meters(generator), distribution_meters(generator);
    T_A_B_sample.getRotation().toImplementation() = Eigen::Quaterniond(
        Eigen::AngleAxisd(
            distribution_rad(generator), Eigen::Vector3d::UnitX()));
    T_A_B_samples.emplace_back(T_A_B_sample);
  }

 public:
  int seed_;
  Aligned<std::vector, pose::Transformation> T_A_B_samples;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(GeometryTransformationRansac, OnlyInliers) {
  constexpr int kNumInliers = 20;
  addInliers(kNumInliers);
  constexpr double kPositionThresholdMeters = 0.1;
  constexpr double kOrientationThresholdRadians = 0.1;
  pose::Transformation T_A_B_ransac;
  int num_inliers = 0;
  constexpr int kNumIterations = 40;
  constexpr int kSeed = 42;
  common::transformationRansac(
      this->T_A_B_samples, kNumIterations, kOrientationThresholdRadians,
      kPositionThresholdMeters, kSeed, &T_A_B_ransac, &num_inliers);
  EXPECT_EQ(kNumInliers, num_inliers);
  EXPECT_NEAR_ASLAM_TRANSFORMATION(T_A_B_ransac, pose::Transformation(), 1e-1);
}

TEST_F(GeometryTransformationRansac, InliersAndOutliers) {
  constexpr int kNumInliers = 20;
  addInliers(kNumInliers);
  addOutlier();
  addOutlier();
  addOutlier();
  constexpr double kPositionThresholdMeters = 0.1;
  constexpr double kOrientationThresholdRadians = 0.1;
  pose::Transformation T_A_B_ransac;
  int num_inliers = 0;
  constexpr int kNumIterations = 40;
  constexpr int kSeed = 42;
  common::transformationRansac(
      this->T_A_B_samples, kNumIterations, kOrientationThresholdRadians,
      kPositionThresholdMeters, kSeed, &T_A_B_ransac, &num_inliers);
  EXPECT_EQ(20, num_inliers);
  EXPECT_NEAR_ASLAM_TRANSFORMATION(T_A_B_ransac, pose::Transformation(), 1e-1);
}

TEST_F(GeometryTransformationRansac, OnlyOutliers) {
  constexpr int kNumOutliers = 20;
  for (int i = 0; i < kNumOutliers; ++i) {
    addOutlier();
  }
  constexpr double kPositionThresholdMeters = 0.1;
  constexpr double kOrientationThresholdRadians = 0.1;
  pose::Transformation T_A_B_ransac;
  int num_inliers = 0;
  constexpr int kNumIterations = 40;
  constexpr int kSeed = 42;
  common::transformationRansac(
      this->T_A_B_samples, kNumIterations, kOrientationThresholdRadians,
      kPositionThresholdMeters, kSeed, &T_A_B_ransac, &num_inliers);
  // Should only have one supporter for every hypothesis.
  EXPECT_GE(1, num_inliers);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
