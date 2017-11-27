#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

using namespace ceres_error_terms;  // NOLINT

struct CostFunctor {
  explicit CostFunctor(const pose::Quaternion& reference)
      : reference_(reference) {}
  template <typename T>
  bool operator()(const T* const x1, const T* const x2, T* residual) const {
    typedef kindr::minimal::RotationQuaternionTemplate<T> QuaternionT;

    const Eigen::Map<const Eigen::Quaternion<T> > quaternion1(x1);
    const Eigen::Map<const Eigen::Quaternion<T> > quaternion2(x2);
    Eigen::Map<Eigen::Matrix<T, 3, 1> > error(residual);

    QuaternionT error_quaternion = common::signedQuaternionProductHamilton(
        common::signedQuaternionProductHamilton(
            QuaternionT(quaternion2), QuaternionT(quaternion1).inverse()),
        QuaternionT(reference_.toImplementation().cast<T>()).inverse());
    error = Eigen::Matrix<T, 3, 1>(
        T(2.0) * error_quaternion.x(), T(2.0) * error_quaternion.y(),
        T(2.0) * error_quaternion.z());
    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  pose::Quaternion reference_;
};

// The test verifies if 4 close-to-unit quaternions (with some random noise),
// tied with relative rotations among x-axis:
// * q1->q2 = +90deg
// * q2->q3 = +90deg
// * q3->q4 = +90deg
// * q4->q1 = +90deg
// and a predefined value of q1(1, 0, 0, 0) will end up with correct relative
// and absolute rotations, i.e.:
// * q1 = 0deg
// * q2 = 90deg
// * q3 = 180deg
// * q4 = 270deg
TEST(PosegraphErrorTerms, TwoPointsRotationSolving_Minimization) {
  pose::Quaternion q1(1, 0, 0, 0);
  pose::Quaternion q2(
      0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
      0.042290245850220926);
  pose::Quaternion q3(
      0.9990182922767228, -0.007804703449117966, -0.026464453222036003,
      0.03465791419330294);
  pose::Quaternion q4(
      0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
      0.01705454355380988);

  // 90deg rotation as relative transformation between nodes
  pose::Quaternion reference(sqrt(2) / 2, sqrt(2) / 2, 0, 0);

  ceres::Problem problem;
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 3, 4, 4>(
          new CostFunctor(reference));

  problem.AddResidualBlock(
      cost_function, NULL, q1.toImplementation().coeffs().data(),
      q2.toImplementation().coeffs().data());
  problem.AddResidualBlock(
      cost_function, NULL, q2.toImplementation().coeffs().data(),
      q3.toImplementation().coeffs().data());
  problem.AddResidualBlock(
      cost_function, NULL, q3.toImplementation().coeffs().data(),
      q4.toImplementation().coeffs().data());
  problem.AddResidualBlock(
      cost_function, NULL, q4.toImplementation().coeffs().data(),
      q1.toImplementation().coeffs().data());

  ceres::LocalParameterization* quaternion_parameterization =
      new ceres_error_terms::JplQuaternionParameterization;
  problem.SetParameterization(
      q1.toImplementation().coeffs().data(), quaternion_parameterization);
  problem.SetParameterization(
      q2.toImplementation().coeffs().data(), quaternion_parameterization);
  problem.SetParameterization(
      q3.toImplementation().coeffs().data(), quaternion_parameterization);
  problem.SetParameterization(
      q4.toImplementation().coeffs().data(), quaternion_parameterization);

  problem.SetParameterBlockConstant(q1.toImplementation().coeffs().data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-16;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  EXPECT_NEAR_KINDR_QUATERNION(
      pose::Quaternion(q2 * q1.inverse()),
      pose::Quaternion(sqrt(2) / 2, sqrt(2) / 2, 0, 0), 1e-8);
  EXPECT_NEAR_KINDR_QUATERNION(
      pose::Quaternion(q3 * q2.inverse()),
      pose::Quaternion(sqrt(2) / 2, sqrt(2) / 2, 0, 0), 1e-8);
  EXPECT_NEAR_KINDR_QUATERNION(
      pose::Quaternion(q4 * q3.inverse()),
      pose::Quaternion(sqrt(2) / 2, sqrt(2) / 2, 0, 0), 1e-8);
  EXPECT_NEAR_KINDR_QUATERNION(
      pose::Quaternion(q1 * q4.inverse()),
      pose::Quaternion(sqrt(2) / 2, sqrt(2) / 2, 0, 0), 1e-8);

  EXPECT_NEAR_KINDR_QUATERNION(q1, pose::Quaternion(1, 0, 0, 0), 1e-8);
  EXPECT_NEAR_KINDR_QUATERNION(
      q2, pose::Quaternion(sqrt(2) / 2, sqrt(2) / 2, 0, 0), 1e-8);
  EXPECT_NEAR_KINDR_QUATERNION(q3, pose::Quaternion(0, 1, 0, 0), 1e-8);
  EXPECT_NEAR_KINDR_QUATERNION(
      q4, pose::Quaternion(-sqrt(2) / 2, sqrt(2) / 2, 0, 0), 1e-8);

  LOG(INFO) << summary.BriefReport();
}

MAPLAB_UNITTEST_ENTRYPOINT
