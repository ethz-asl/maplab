#include <memory>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

TEST(OpengvPoseEstimation, KneipP3p) {
  opengv::bearingVectors_t bearing_vectors;
  opengv::points_t points;

  points.push_back(opengv::point_t(0, -1, 5));
  points.push_back(opengv::point_t(-1, 0, 0));
  points.push_back(opengv::point_t(1, 0, 10));

  bearing_vectors.push_back(opengv::bearingVector_t(0, -1, 6).normalized());
  bearing_vectors.push_back(opengv::bearingVector_t(-1, 0, 1).normalized());
  bearing_vectors.push_back(opengv::bearingVector_t(1, 0, 11).normalized());

  opengv::absolute_pose::CentralAbsoluteAdapter adapter(
      bearing_vectors, points);

  opengv::transformations_t p3p_kneip_transformations;
  p3p_kneip_transformations = opengv::absolute_pose::p3p_kneip(adapter);

  CHECK_EQ(4u, p3p_kneip_transformations.size());

  Eigen::Matrix<double, 3, 4> expected_transform;
  expected_transform << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -1;

  const bool is_1st_guess_matching =
      p3p_kneip_transformations[0].isApprox(expected_transform, 1e-10);
  const bool is_2nd_guess_matching =
      p3p_kneip_transformations[1].isApprox(expected_transform, 1e-10);
  const bool is_3rd_guess_matching =
      p3p_kneip_transformations[2].isApprox(expected_transform, 1e-10);
  const bool is_4th_guess_matching =
      p3p_kneip_transformations[3].isApprox(expected_transform, 1e-10);

  EXPECT_TRUE(
      is_1st_guess_matching || is_2nd_guess_matching || is_3rd_guess_matching ||
      is_4th_guess_matching);
}

TEST(OpengvPoseEstimation, KneipP3pRansac) {
  opengv::bearingVectors_t bearing_vectors;
  opengv::points_t points;

  points.push_back(opengv::point_t(0, -1, 0));
  points.push_back(opengv::point_t(-1, 0, 0));
  points.push_back(opengv::point_t(1, 0, 1));
  points.push_back(opengv::point_t(1, 1, 8));

  bearing_vectors.push_back(opengv::bearingVector_t(0, -1, 1).normalized());
  bearing_vectors.push_back(opengv::bearingVector_t(-1, 0, 1).normalized());
  bearing_vectors.push_back(opengv::bearingVector_t(1, 0, 2).normalized());
  bearing_vectors.push_back(opengv::bearingVector_t(1, 1, 9).normalized());

  opengv::absolute_pose::CentralAbsoluteAdapter adapter(
      bearing_vectors, points);

  opengv::sac::Ransac<
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      ransac;
  std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr(
          new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
              adapter, opengv::sac_problems::absolute_pose::
                           AbsolutePoseSacProblem::KNEIP));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0) * 0.5 / 800.0));
  ransac.max_iterations_ = 50;

  ransac.computeModel();
  Eigen::Matrix<double, 3, 4> expected_transform;
  expected_transform << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -1;
  EXPECT_NEAR_EIGEN(expected_transform, ransac.model_coefficients_, 1e-10);
}

MAPLAB_UNITTEST_ENTRYPOINT
