#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "ceres-error-terms/parameterization/pose-param-jpl.h"

struct CostFunctor {
  explicit CostFunctor(const pose::Transformation& reference)
      : reference_(reference) {}
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    typedef kindr::minimal::RotationQuaternionTemplate<T> QuaternionT;

    // x is currently passed from x.coeffs() Eigen function that returns
    // (x, y, z, w) JPL convention so we need to map it first to Eigen
    // Quaternion and then construct a kindr quaternion on top
    const Eigen::Map<const Eigen::Quaternion<T>> current_rot(x);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> current_pos(x + 4);
    Eigen::Map<Eigen::Matrix<T, 6, 1>> error(residual);

    QuaternionT error_quaternion = common::signedQuaternionProductHamilton(
        QuaternionT(current_rot),
        QuaternionT(
            reference_.getRotation().inverse().toImplementation().cast<T>()));

    error.head(3) = Eigen::Matrix<T, 3, 1>(
        T(2.0) * error_quaternion.x(), T(2.0) * error_quaternion.y(),
        T(2.0) * error_quaternion.z());
    error.tail(3) = current_pos - reference_.getPosition().cast<T>();

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  pose::Transformation reference_;
};

TEST(JplPoseParameterization, SimpleMinimization) {
  // Initial state values.
  pose::Quaternion x_rot(1, 0, 0, 0);
  pose::Position3D x_pos(1, 3.7, 0);
  // Create contiguous pose memory.
  Eigen::Matrix<double, 7, 1> x_pose;
  x_pose << x_rot.toImplementation().coeffs(), x_pos;

  // Reference transformation the solver should arrive at.
  pose::Quaternion reference_rot(sqrt(2) / 2, sqrt(2) / 2, 0, 0);
  pose::Position3D reference_pos(0.2, -0.3, 0.7);
  pose::Transformation reference(reference_pos, reference_rot);

  ceres::Problem problem;
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 6, 7>(
          new CostFunctor(reference));
  problem.AddResidualBlock(cost_function, NULL, x_pose.data());
  ceres::LocalParameterization* pose_parameterization =
      new ceres_error_terms::JplPoseParameterization;
  problem.SetParameterization(x_pose.data(), pose_parameterization);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  pose::Quaternion out_rot(
      Eigen::Map<Eigen::Quaternion<double>>(x_pose.head(4).data()));
  Eigen::Vector3d out_pos(x_pose.tail(3));
  EXPECT_NEAR_KINDR_QUATERNION(out_rot, reference_rot, 1e-6);
  EXPECT_NEAR_EIGEN(out_pos, reference_pos, 1e-10);
  LOG(INFO) << summary.BriefReport() << std::endl;
}

MAPLAB_UNITTEST_ENTRYPOINT
