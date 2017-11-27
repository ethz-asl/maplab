#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <ceres-error-terms/block-pose-prior-error-term.h>
#include <ceres-error-terms/common.h>
#include <ceres-error-terms/loop-closure-block-pose-error-term.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/switch-prior-error-term.h>
#include <ceres-error-terms/test_posegraph/loop_closure_constraint.h>
#include <ceres-error-terms/test_posegraph/posegraph.h>
#include <ceres-error-terms/test_posegraph/posegraph_constraint.h>

using namespace ceres_error_terms;  // NOLINT

void fillPosegraphWithLoopClosure(BlockPosegraph& posegraph) {  // NOLINT
  const int num_poses = 4;
  const int num_loop_closure_constraints = 1;
  const int num_baseframes = 2;
  posegraph.loop_closure_constraints.resize(num_loop_closure_constraints);
  posegraph.vertex_poses = Eigen::MatrixXd::Zero(7, num_poses);

  posegraph.baseframe_poses = Eigen::MatrixXd::Zero(7, num_baseframes);
  pose::Transformation T_G_M1;
  T_G_M1.setRandom(1e4);
  posegraph.baseframe_poses.block<4, 1>(0, 0)
      << T_G_M1.getRotation().toImplementation().inverse().coeffs();
  posegraph.baseframe_poses.block<3, 1>(4, 0) << T_G_M1.getPosition();

  pose::Transformation T_G_M2;
  T_G_M2.setRandom(1e4);
  posegraph.baseframe_poses.block<4, 1>(0, 1)
      << T_G_M2.getRotation().toImplementation().inverse().coeffs();
  posegraph.baseframe_poses.block<3, 1>(4, 1) << T_G_M2.getPosition();

  const pose::Transformation T_G_V0(
      pose::Quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
      pose::Position3D(0, 0, 0));
  const pose::Transformation T_G_V1(
      pose::Quaternion(
          Eigen::Quaterniond(
              0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
              0.042290245850220926)),
      pose::Position3D(1.1, 0, 0));
  const pose::Transformation T_G_V2(
      pose::Quaternion(
          Eigen::Quaterniond(
              0.9990182922767228, -0.007804703449117966, -0.026464453222036003,
              0.03465791419330294)),
      pose::Position3D(1, 2.5, 0));
  const pose::Transformation T_G_V3(
      pose::Quaternion(
          Eigen::Quaterniond(
              0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
              0.01705454355380988)),
      pose::Position3D(0, 2, 0));
  const pose::Transformation T_M1_V0 = T_G_M1.inverse() * T_G_V0;
  const pose::Transformation T_M1_V1 = T_G_M1.inverse() * T_G_V1;
  const pose::Transformation T_M2_V2 = T_G_M2.inverse() * T_G_V2;
  const pose::Transformation T_M2_V3 = T_G_M2.inverse() * T_G_V3;

  const pose::Transformation T_G_V0_true(
      pose::Quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
      pose::Position3D(0, 0, 0));
  const pose::Transformation T_G_V1_true(
      pose::Quaternion(Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      pose::Position3D(1, 0, 0));
  const pose::Transformation T_G_V2_true(
      pose::Quaternion(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
      pose::Position3D(1, 1, 0));
  const pose::Transformation T_G_V3_true(
      pose::Quaternion(Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      pose::Position3D(0, 1, 0));
  const pose::Transformation T_M1_V0_true = T_G_M1.inverse() * T_G_V0_true;
  const pose::Transformation T_M1_V1_true = T_G_M1.inverse() * T_G_V1_true;
  const pose::Transformation T_M2_V2_true = T_G_M2.inverse() * T_G_V2_true;
  const pose::Transformation T_M2_V3_true = T_G_M2.inverse() * T_G_V3_true;

  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(0, 0);
  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(1, 0);
  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(2, 1);
  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(3, 1);

  posegraph.vertex_poses.block<4, 1>(0, 0) =
      T_M1_V0.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 0) = T_M1_V0.getPosition();
  posegraph.vertex_poses.block<4, 1>(0, 1) =
      T_M1_V1.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 1) = T_M1_V1.getPosition();
  posegraph.vertex_poses.block<4, 1>(0, 2) =
      T_M2_V2.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 2) = T_M2_V2.getPosition();
  posegraph.vertex_poses.block<4, 1>(0, 3) =
      T_M2_V3.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 3) = T_M2_V3.getPosition();

  // Add odometry constraints.
  Eigen::Matrix<double, 6, 6> dummy_covariance;
  dummy_covariance.setIdentity();
  posegraph.pose_prior_constriants.emplace_back(
      0, T_M1_V0_true, dummy_covariance);
  posegraph.pose_prior_constriants.emplace_back(
      1, T_M1_V1_true, dummy_covariance);
  posegraph.pose_prior_constriants.emplace_back(
      2, T_M2_V2_true, dummy_covariance);
  posegraph.pose_prior_constriants.emplace_back(
      3, T_M2_V3_true, dummy_covariance);

  // Add loop closure constraints.
  const double initial_switch_variable = 1.0;
  const double switch_variable_covariance = 1.0;
  const pose::Transformation T_V0_V2_lc(
      pose::Position3D(1, 1, 0), pose::Quaternion(0, 0, 0, 1));
  posegraph.loop_closure_constraints[0] = LoopClosureConstraint(
      0, 2, T_V0_V2_lc, dummy_covariance, switch_variable_covariance,
      initial_switch_variable);
}

void fillPosegraphWithIncorrectLoopClosure(
    BlockPosegraph& posegraph) {  // NOLINT
  const int num_poses = 4;
  const int num_loop_closure_constraints = 1;
  const int num_baseframes = 2;
  posegraph.loop_closure_constraints.resize(num_loop_closure_constraints);
  posegraph.vertex_poses = Eigen::MatrixXd::Zero(7, num_poses);

  posegraph.baseframe_poses = Eigen::MatrixXd::Zero(7, num_baseframes);
  pose::Transformation T_G_M1;
  T_G_M1.setRandom(1e4);
  posegraph.baseframe_poses.block<4, 1>(0, 0)
      << T_G_M1.getRotation().toImplementation().inverse().coeffs();
  posegraph.baseframe_poses.block<3, 1>(4, 0) << T_G_M1.getPosition();

  pose::Transformation T_G_M2;
  T_G_M2.setRandom(1e4);
  posegraph.baseframe_poses.block<4, 1>(0, 1)
      << T_G_M2.getRotation().toImplementation().inverse().coeffs();
  posegraph.baseframe_poses.block<3, 1>(4, 1) << T_G_M2.getPosition();

  const pose::Transformation T_G_V0(
      pose::Quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
      pose::Position3D(0, 0, 0));
  const pose::Transformation T_G_V1(
      pose::Quaternion(
          Eigen::Quaterniond(
              0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
              0.042290245850220926)),
      pose::Position3D(1.1, 0, 0));
  const pose::Transformation T_G_V2(
      pose::Quaternion(
          Eigen::Quaterniond(
              0.9990182922767228, -0.007804703449117966, -0.026464453222036003,
              0.03465791419330294)),
      pose::Position3D(1, 2.5, 0));
  const pose::Transformation T_G_V3(
      pose::Quaternion(
          Eigen::Quaterniond(
              0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
              0.01705454355380988)),
      pose::Position3D(0, 2, 0));
  const pose::Transformation T_M1_V0 = T_G_M1.inverse() * T_G_V0;
  const pose::Transformation T_M1_V1 = T_G_M1.inverse() * T_G_V1;
  const pose::Transformation T_M2_V2 = T_G_M2.inverse() * T_G_V2;
  const pose::Transformation T_M2_V3 = T_G_M2.inverse() * T_G_V3;

  const pose::Transformation T_G_V0_true(
      pose::Quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)),
      pose::Position3D(0, 0, 0));
  const pose::Transformation T_G_V1_true(
      pose::Quaternion(Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      pose::Position3D(1, 0, 0));
  const pose::Transformation T_G_V2_true(
      pose::Quaternion(Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)),
      pose::Position3D(1, 1, 0));
  const pose::Transformation T_G_V3_true(
      pose::Quaternion(Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      pose::Position3D(0, 1, 0));
  const pose::Transformation T_M1_V0_true = T_G_M1.inverse() * T_G_V0_true;
  const pose::Transformation T_M1_V1_true = T_G_M1.inverse() * T_G_V1_true;
  const pose::Transformation T_M2_V2_true = T_G_M2.inverse() * T_G_V2_true;
  const pose::Transformation T_M2_V3_true = T_G_M2.inverse() * T_G_V3_true;

  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(0, 0);
  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(1, 0);
  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(2, 1);
  posegraph.vertex_pose_idx_to_baseframe_idx.emplace(3, 1);

  posegraph.vertex_poses.block<4, 1>(0, 0) =
      T_M1_V0.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 0) = T_M1_V0.getPosition();
  posegraph.vertex_poses.block<4, 1>(0, 1) =
      T_M1_V1.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 1) = T_M1_V1.getPosition();
  posegraph.vertex_poses.block<4, 1>(0, 2) =
      T_M2_V2.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 2) = T_M2_V2.getPosition();
  posegraph.vertex_poses.block<4, 1>(0, 3) =
      T_M2_V3.getRotation().toImplementation().coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 3) = T_M2_V3.getPosition();

  // Add odometry constraints.
  Eigen::Matrix<double, 6, 6> dummy_covariance;
  dummy_covariance.setIdentity();
  posegraph.pose_prior_constriants.emplace_back(
      0, T_M1_V0_true, dummy_covariance);
  posegraph.pose_prior_constriants.emplace_back(
      1, T_M1_V1_true, dummy_covariance);
  posegraph.pose_prior_constriants.emplace_back(
      2, T_M2_V2_true, dummy_covariance);
  posegraph.pose_prior_constriants.emplace_back(
      3, T_M2_V3_true, dummy_covariance);

  // Add loop closure constraints.
  const double initial_switch_variable = 1.0;
  const double switch_variable_covariance = 100;
  pose::Quaternion loop_quaternion(0.6916, 0.6350, 0.0321, 0.3426);
  loop_quaternion.normalize();
  posegraph.loop_closure_constraints[0] = LoopClosureConstraint(
      0, 2, pose::Transformation(pose::Position3D(-2, 0, 0.2), loop_quaternion),
      0.1 * dummy_covariance, switch_variable_covariance,
      initial_switch_variable);
}

void solveWithSwitchableConstraints(BlockPosegraph& posegraph) {  // NOLINT
  ceres::Problem problem;
  ceres::LocalParameterization* pose_parametrization(
      new JplPoseParameterization);

  for (const PosePriorConstraint& ppc : posegraph.pose_prior_constriants) {
    ceres::CostFunction* pose_prior_cost(
        new ceres_error_terms::BlockPosePriorErrorTerm(
            ppc.T_G_I_.getRotation().toImplementation().coeffs(),
            ppc.T_G_I_.getPosition(), ppc.T_G_I_covariance_));

    problem.AddResidualBlock(
        pose_prior_cost, nullptr, posegraph.vertex_poses.col(ppc.node_).data());
  }

  pose::Quaternion G_q_M;
  G_q_M.setIdentity();
  pose::Position3D G_p_M;
  G_p_M.setZero();

  CHECK_EQ(posegraph.baseframe_poses.cols(), 2);

  const double switch_prior = 1.0;
  for (LoopClosureConstraint& lcc : posegraph.loop_closure_constraints) {
    ceres::CostFunction* loop_closure_cost = new ceres::AutoDiffCostFunction<
        LoopClosureBlockPoseErrorTerm,
        LoopClosureBlockPoseErrorTerm::residualBlockSize, poseblocks::kPoseSize,
        poseblocks::kPoseSize, poseblocks::kPoseSize, poseblocks::kPoseSize,
        LoopClosureBlockPoseErrorTerm::switchVariableBlockSize>(
        new LoopClosureBlockPoseErrorTerm(lcc.T_A_B_, lcc.T_A_B_covariance_));
    ceres::CostFunction* switch_variable_cost = new ceres::AutoDiffCostFunction<
        SwitchPriorErrorTermLegacy,
        SwitchPriorErrorTermLegacy::residualBlockSize,
        SwitchPriorErrorTermLegacy::switchVariableBlockSize>(
        new SwitchPriorErrorTermLegacy(
            switch_prior, lcc.switch_variable_covariance_));

    std::unordered_map<int, int>::iterator baseframe_from_iterator =
        posegraph.vertex_pose_idx_to_baseframe_idx.find(lcc.from_node_);
    CHECK(
        baseframe_from_iterator !=
        posegraph.vertex_pose_idx_to_baseframe_idx.end());
    const int baseframe_from_idx = baseframe_from_iterator->second;
    CHECK_GE(baseframe_from_idx, 0);
    CHECK_LT(baseframe_from_idx, posegraph.baseframe_poses.cols());

    std::unordered_map<int, int>::iterator baseframe_to_iterator =
        posegraph.vertex_pose_idx_to_baseframe_idx.find(lcc.to_node_);
    CHECK(
        baseframe_to_iterator !=
        posegraph.vertex_pose_idx_to_baseframe_idx.end());
    const int baseframe_to_idx = baseframe_to_iterator->second;
    CHECK_GE(baseframe_to_idx, 0);
    CHECK_LT(baseframe_to_idx, posegraph.baseframe_poses.cols());

    problem.AddResidualBlock(
        loop_closure_cost, NULL,
        posegraph.baseframe_poses.col(baseframe_from_idx).data(),
        posegraph.vertex_poses.col(lcc.from_node_).data(),
        posegraph.baseframe_poses.col(baseframe_to_idx).data(),
        posegraph.vertex_poses.col(lcc.to_node_).data(), &lcc.switch_variable_);

    problem.AddResidualBlock(switch_variable_cost, NULL, &lcc.switch_variable_);

    problem.SetParameterization(
        posegraph.baseframe_poses.col(baseframe_from_idx).data(),
        pose_parametrization);
    problem.SetParameterization(
        posegraph.baseframe_poses.col(baseframe_to_idx).data(),
        pose_parametrization);
    problem.SetParameterBlockConstant(
        posegraph.baseframe_poses.col(baseframe_from_idx).data());
    problem.SetParameterBlockConstant(
        posegraph.baseframe_poses.col(baseframe_to_idx).data());
  }

  // Add parameterization for quaternion of each graph node.
  for (int col_idx = 0; col_idx < posegraph.vertex_poses.cols(); ++col_idx) {
    problem.SetParameterization(
        posegraph.vertex_poses.col(col_idx).data(), pose_parametrization);
  }

  problem.SetParameterBlockConstant(posegraph.vertex_poses.col(0).data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-16;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  LOG(INFO) << summary.BriefReport();
}

// The test verifies if a posegraph with:
// * noisy initial guesses
// * perfect odometry constraints
// * perfect loop closure constraint
// will end up as perfect square. The switch variable should be equal to 1
// (i.e. the loop closure should be treated as valid).
TEST(PosegraphErrorTerms, SwitchableConstraints_SimpleLoopClosure) {
  BlockPosegraph posegraph;
  fillPosegraphWithLoopClosure(posegraph);
  solveWithSwitchableConstraints(posegraph);

  CHECK_EQ(posegraph.baseframe_poses.cols(), 2);

  const Eigen::Quaterniond q_G_M1_JPL(
      posegraph.baseframe_poses.col(0).head(4).data());
  const pose::Transformation T_G_M1(
      q_G_M1_JPL.inverse(), posegraph.baseframe_poses.col(0).tail(3));

  const Eigen::Quaterniond q_G_M2_JPL(
      posegraph.baseframe_poses.col(1).head(4).data());
  const pose::Transformation T_G_M2(
      q_G_M2_JPL.inverse(), posegraph.baseframe_poses.col(1).tail(3));

  pose::Quaternion q_M1_V0;
  q_M1_V0.toImplementation().coeffs() << posegraph.vertex_poses.col(0).head(4);
  const pose::Transformation T_M1_V0(
      q_M1_V0, posegraph.vertex_poses.col(0).tail(3));
  const pose::Transformation T_G_V0 = T_G_M1 * T_M1_V0;

  pose::Quaternion q_M1_V1;
  q_M1_V1.toImplementation().coeffs() << posegraph.vertex_poses.col(1).head(4);
  const pose::Transformation T_M1_V1(
      q_M1_V1, posegraph.vertex_poses.col(1).tail(3));
  const pose::Transformation T_G_V1 = T_G_M1 * T_M1_V1;

  pose::Quaternion q_M2_V2;
  q_M2_V2.toImplementation().coeffs() << posegraph.vertex_poses.col(2).head(4);
  const pose::Transformation T_M2_V2(
      q_M2_V2, posegraph.vertex_poses.col(2).tail(3));
  const pose::Transformation T_G_V2 = T_G_M2 * T_M2_V2;

  pose::Quaternion q_M2_V3;
  q_M2_V3.toImplementation().coeffs() << posegraph.vertex_poses.col(3).head(4);
  const pose::Transformation T_M2_V3(
      q_M2_V3, posegraph.vertex_poses.col(3).tail(3));
  const pose::Transformation T_G_V3 = T_G_M2 * T_M2_V3;

  const pose::Transformation T_V0_V2 = T_G_V0.inverse() * T_G_V2;

  EXPECT_ZERO_EIGEN(T_G_V0.getPosition(), 1e-8);
  EXPECT_NEAR_EIGEN(T_G_V1.getPosition(), pose::Position3D(1, 0, 0), 1e-8);
  EXPECT_NEAR_EIGEN(T_G_V2.getPosition(), pose::Position3D(1, 1, 0), 1e-8);
  EXPECT_NEAR_EIGEN(T_G_V3.getPosition(), pose::Position3D(0, 1, 0), 1e-8);

  EXPECT_NEAR_EIGEN(
      T_G_V0.getRotation().toImplementation().coeffs(),
      Eigen::Quaterniond(1, 0, 0, 0).coeffs(), 1e-6);

  EXPECT_NEAR_EIGEN(
      T_G_V1.getRotation().toImplementation().coeffs(),
      Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2).coeffs(), 1e-6);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_G_V2.getRotation(),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(0, 0, 0, 1)), 1e-6);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_G_V3.getRotation(),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      1e-6);

  EXPECT_NEAR(posegraph.loop_closure_constraints[0].switch_variable_, 1, 1e-5);
}

// The test verifies if a posegraph with:
// * noisy initial guesses
// * perfect odometry constraints
// * wrong loop closure with large variance
// will end up as perfect square. The switch variable should be equal to 0
// (i.e. the loop closure should be completely ignored).
TEST(PosegraphErrorTerms, SwitchableConstraints_IncorrectLoopClosure) {
  BlockPosegraph posegraph;
  fillPosegraphWithIncorrectLoopClosure(posegraph);
  solveWithSwitchableConstraints(posegraph);

  CHECK_EQ(posegraph.baseframe_poses.cols(), 2);

  const Eigen::Quaterniond q_G_M1_JPL(
      posegraph.baseframe_poses.col(0).head(4).data());
  const pose::Transformation T_G_M1(
      q_G_M1_JPL.inverse(), posegraph.baseframe_poses.col(0).tail(3));

  const Eigen::Quaterniond q_G_M2_JPL(
      posegraph.baseframe_poses.col(1).head(4).data());
  const pose::Transformation T_G_M2(
      q_G_M2_JPL.inverse(), posegraph.baseframe_poses.col(1).tail(3));

  pose::Quaternion q_M1_V0;
  q_M1_V0.toImplementation().coeffs() << posegraph.vertex_poses.col(0).head(4);
  const pose::Transformation T_M1_V0(
      q_M1_V0, posegraph.vertex_poses.col(0).tail(3));
  const pose::Transformation T_G_V0 = T_G_M1 * T_M1_V0;

  pose::Quaternion q_M1_V1;
  q_M1_V1.toImplementation().coeffs() << posegraph.vertex_poses.col(1).head(4);
  const pose::Transformation T_M1_V1(
      q_M1_V1, posegraph.vertex_poses.col(1).tail(3));
  const pose::Transformation T_G_V1 = T_G_M1 * T_M1_V1;

  pose::Quaternion q_M2_V2;
  q_M2_V2.toImplementation().coeffs() << posegraph.vertex_poses.col(2).head(4);
  const pose::Transformation T_M2_V2(
      q_M2_V2, posegraph.vertex_poses.col(2).tail(3));
  const pose::Transformation T_G_V2 = T_G_M2 * T_M2_V2;

  pose::Quaternion q_M2_V3;
  q_M2_V3.toImplementation().coeffs() << posegraph.vertex_poses.col(3).head(4);
  const pose::Transformation T_M2_V3(
      q_M2_V3, posegraph.vertex_poses.col(3).tail(3));
  const pose::Transformation T_G_V3 = T_G_M2 * T_M2_V3;

  const pose::Transformation T_V0_V2 = T_G_V0.inverse() * T_G_V2;

  EXPECT_ZERO_EIGEN(T_G_V0.getPosition(), 1e-5);
  EXPECT_NEAR_EIGEN(T_G_V1.getPosition(), pose::Position3D(1, 0, 0), 1e-5);
  EXPECT_NEAR_EIGEN(T_G_V2.getPosition(), pose::Position3D(1, 1, 0), 1e-5);
  EXPECT_NEAR_EIGEN(T_G_V3.getPosition(), pose::Position3D(0, 1, 0), 1e-5);

  EXPECT_NEAR_EIGEN(
      T_G_V0.getRotation().toImplementation().coeffs(),
      Eigen::Quaterniond(1, 0, 0, 0).coeffs(), 1e-6);

  EXPECT_NEAR_EIGEN(
      T_G_V1.getRotation().toImplementation().coeffs(),
      Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2).coeffs(), 1e-6);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_G_V2.getRotation(),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(0, 0, 0, 1)), 1e-6);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_G_V3.getRotation(),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      1e-6);

  EXPECT_NEAR(posegraph.loop_closure_constraints[0].switch_variable_, 0, 1e-4);
}

MAPLAB_UNITTEST_ENTRYPOINT
