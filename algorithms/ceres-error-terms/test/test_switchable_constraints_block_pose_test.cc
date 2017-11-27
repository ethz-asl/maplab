#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <ceres-error-terms/common.h>
#include <ceres-error-terms/loop-closure-block-pose-error-term.h>
#include <ceres-error-terms/parameterization/pose-param-jpl.h>
#include <ceres-error-terms/six-dof-block-pose-error-term-autodiff.h>
#include <ceres-error-terms/switch-prior-error-term.h>
#include <ceres-error-terms/test_posegraph/loop_closure_constraint.h>
#include <ceres-error-terms/test_posegraph/posegraph.h>
#include <ceres-error-terms/test_posegraph/posegraph_constraint.h>

using namespace ceres_error_terms;  // NOLINT

void fillPosegraphWithLoopClosure(BlockPosegraph& posegraph) {  // NOLINT
  const int num_poses = 4;
  const int num_odometry_constraints = 4;
  const int num_loop_closure_constraints = 1;
  posegraph.constraints.resize(num_odometry_constraints);
  posegraph.loop_closure_constraints.resize(num_loop_closure_constraints);
  posegraph.vertex_poses = Eigen::MatrixXd::Zero(7, num_poses);

  posegraph.vertex_poses.block<4, 1>(0, 0) =
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 0) = pose::Position3D(0, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 1) =
      Eigen::Quaterniond(
          0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
          0.042290245850220926)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 1) = pose::Position3D(1.1, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 2) =
      Eigen::Quaterniond(
          0.9990182922767228, -0.007804703449117966, -0.026464453222036003,
          0.03465791419330294)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 2) = pose::Position3D(1, 2.5, 0);
  posegraph.vertex_poses.block<4, 1>(0, 3) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 3) = pose::Position3D(0, 2, 0);

  // add odometry constraints
  Eigen::Matrix<double, 6, 6> dummy_covariance;
  dummy_covariance.setIdentity();
  posegraph.constraints[0] = PosegraphConstraint(
      0, 1, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[1] = PosegraphConstraint(
      1, 2, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[2] = PosegraphConstraint(
      2, 3, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[3] = PosegraphConstraint(
      3, 0, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  // add loop closure constraints
  const double initial_switch_variable = 1.0;
  const double switch_variable_covariance = 1.0;
  posegraph.loop_closure_constraints[0] = LoopClosureConstraint(
      0, 2, pose::Transformation(
                pose::Position3D(1, 1, 0), pose::Quaternion(0, 0, 0, 1)),
      dummy_covariance, switch_variable_covariance, initial_switch_variable);
}

void fillPosegraphWithIncorrectLoopClosure(
    BlockPosegraph& posegraph) {  // NOLINT
  const int num_poses = 4;
  const int num_odometry_constraints = 4;
  const int num_loop_closure_constraints = 1;
  posegraph.constraints.resize(num_odometry_constraints);
  posegraph.vertex_poses = Eigen::MatrixXd::Zero(7, num_poses);
  posegraph.loop_closure_constraints.resize(num_loop_closure_constraints);

  posegraph.vertex_poses.block<4, 1>(0, 0) =
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 0) = pose::Position3D(0, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 1) =
      Eigen::Quaterniond(
          0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
          0.042290245850220926)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 1) = pose::Position3D(1.1, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 2) =
      Eigen::Quaterniond(
          0.9990182922767228, -0.007804703449117966, -0.026464453222036003,
          0.03465791419330294)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 2) = pose::Position3D(1, 2.5, 0);
  posegraph.vertex_poses.block<4, 1>(0, 3) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 3) = pose::Position3D(0, 2, 0);

  // add odometry constraints
  Eigen::Matrix<double, 6, 6> dummy_covariance;
  dummy_covariance.setIdentity();
  posegraph.constraints[0] = PosegraphConstraint(
      0, 1, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[1] = PosegraphConstraint(
      1, 2, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[2] = PosegraphConstraint(
      2, 3, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[3] = PosegraphConstraint(
      3, 0, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);

  // add loop closure constraints
  const double initial_switch_variable = 1.0;
  const double switch_variable_covariance = 100;
  pose::Quaternion loop_quaternion(0.6916, 0.6350, 0.0321, 0.3426);
  loop_quaternion.normalize();
  posegraph.loop_closure_constraints[0] = LoopClosureConstraint(
      0, 2, pose::Transformation(pose::Position3D(-2, 0, 0.2), loop_quaternion),
      0.1 * dummy_covariance, switch_variable_covariance,
      initial_switch_variable);
}

void fillPosegraphWith7NodesMatched(BlockPosegraph& posegraph) {  // NOLINT
  const int num_poses = 7;
  const int num_odometry_constraints = 6;
  const int num_loop_closure_constraints = 1;
  posegraph.constraints.resize(num_odometry_constraints);
  posegraph.vertex_poses = Eigen::MatrixXd::Zero(7, num_poses);
  posegraph.loop_closure_constraints.resize(num_loop_closure_constraints);

  posegraph.vertex_poses.block<4, 1>(0, 0) =
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 0) = pose::Position3D(0, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 1) =
      Eigen::Quaterniond(
          0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
          0.042290245850220926)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 1) = pose::Position3D(1.13, 0, -0.2);
  posegraph.vertex_poses.block<4, 1>(0, 2) =
      Eigen::Quaterniond(
          0.9990182922767228, -0.007804703449117966, -0.026464453222036003,
          0.03465791419330294)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 2) = pose::Position3D(2.1, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 3) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 3) = pose::Position3D(1.9, 1.2, 0);
  posegraph.vertex_poses.block<4, 1>(0, 4) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 4) = pose::Position3D(1, 1.1, 0.1);
  posegraph.vertex_poses.block<4, 1>(0, 5) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 5) = pose::Position3D(1.3, -0.1, 0);
  posegraph.vertex_poses.block<4, 1>(0, 6) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 6) = pose::Position3D(0.7, -0.9, -0.34);

  // add odometry constraints
  Eigen::Matrix<double, 6, 6> dummy_covariance;
  dummy_covariance.setIdentity();
  posegraph.constraints[0] = PosegraphConstraint(
      0, 1, pose::Transformation(
                pose::Position3D(1, 0, 0), pose::Quaternion(1, 0, 0, 0)),
      dummy_covariance);
  posegraph.constraints[1] = PosegraphConstraint(
      1, 2, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[2] = PosegraphConstraint(
      2, 3, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[3] = PosegraphConstraint(
      3, 4, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[4] = PosegraphConstraint(
      4, 5, pose::Transformation(
                pose::Position3D(1, 0, 0), pose::Quaternion(1, 0, 0, 0)),
      dummy_covariance);
  posegraph.constraints[5] = PosegraphConstraint(
      5, 6, pose::Transformation(
                pose::Position3D(1, 0, 0), pose::Quaternion(1, 0, 0, 0)),
      dummy_covariance);
  // add loop closure constraints
  const double initial_switch_variable = 1.0;
  const double switch_variable_covariance = 1.0;
  posegraph.loop_closure_constraints[0] = LoopClosureConstraint(
      5, 1, pose::Transformation(
                pose::Position3D(0, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance, switch_variable_covariance, initial_switch_variable);
}

void fillPosegraphWith7NodesUnmatched(BlockPosegraph& posegraph) {  // NOLINT
  const int num_poses = 7;
  const int num_odometry_constraints = 6;
  const int num_loop_closure_constraints = 1;
  posegraph.constraints.resize(num_odometry_constraints);
  posegraph.vertex_poses = Eigen::MatrixXd::Zero(7, num_poses);
  posegraph.loop_closure_constraints.resize(num_loop_closure_constraints);

  posegraph.vertex_poses.block<4, 1>(0, 0) =
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 0) = pose::Position3D(0, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 1) =
      Eigen::Quaterniond(
          0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
          0.042290245850220926)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 1) = pose::Position3D(1.13, 0, -0.2);
  posegraph.vertex_poses.block<4, 1>(0, 2) =
      Eigen::Quaterniond(
          0.9990182922767228, -0.007804703449117966, -0.026464453222036003,
          0.03465791419330294)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 2) = pose::Position3D(2.1, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 3) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 3) = pose::Position3D(1.9, 1.2, 0);
  posegraph.vertex_poses.block<4, 1>(0, 4) =
      Eigen::Quaterniond(
          0.9988646699884802, -0.00795566765132752, -0.043763237371076937,
          0.01705454355380988)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 4) = pose::Position3D(1, 1.1, 0.1);
  posegraph.vertex_poses.block<4, 1>(0, 5) =
      Eigen::Quaterniond(
          0.349782539544818, -0.1191617011054724, 0.019291722755743392,
          0.9290212556515289)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 5) = pose::Position3D(1.3, -0.1, 0);
  posegraph.vertex_poses.block<4, 1>(0, 6) =
      Eigen::Quaterniond(
          0.349782539544818, -0.1191617011054724, 0.019291722755743392,
          -0.9290212556515289)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 6) = pose::Position3D(0.7, -0.9, -0.34);

  // add odometry constraints
  Eigen::Matrix<double, 6, 6> dummy_covariance;
  dummy_covariance.setIdentity();
  Eigen::Matrix<double, 6, 6> large_rotation_covariance = dummy_covariance;
  large_rotation_covariance(3, 3) = 1e10;
  large_rotation_covariance(4, 4) = 1e10;
  large_rotation_covariance(5, 5) = 1e10;
  posegraph.constraints[0] = PosegraphConstraint(
      0, 1, pose::Transformation(
                pose::Position3D(1, 0, 0), pose::Quaternion(1, 0, 0, 0)),
      dummy_covariance);
  posegraph.constraints[1] = PosegraphConstraint(
      1, 2, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[2] = PosegraphConstraint(
      2, 3, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      dummy_covariance);
  posegraph.constraints[3] = PosegraphConstraint(
      3, 4, pose::Transformation(
                pose::Position3D(1, 0, 0),
                pose::Quaternion(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      large_rotation_covariance);
  posegraph.constraints[4] = PosegraphConstraint(
      4, 5, pose::Transformation(
                pose::Position3D(sqrt(2), 0, 0), pose::Quaternion(1, 0, 0, 0)),
      dummy_covariance);
  posegraph.constraints[5] = PosegraphConstraint(
      5, 6, pose::Transformation(
                pose::Position3D(sqrt(2), 0, 0), pose::Quaternion(1, 0, 0, 0)),
      dummy_covariance);
  // add loop closure constraints
  const double initial_switch_variable = 1.0;
  const double switch_variable_covariance = 0.01;
  pose::Quaternion loop_quaternion(0.38268, 0, 0, 0.92387);
  loop_quaternion.normalize();
  posegraph.loop_closure_constraints[0] = LoopClosureConstraint(
      5, 0, pose::Transformation(pose::Position3D(0, 0, 0), loop_quaternion),
      dummy_covariance, switch_variable_covariance, initial_switch_variable);
}

void fillPosegraphWith2ContradictoryConstraints(
    BlockPosegraph& posegraph) {  // NOLINT
  const int num_poses = 2;
  const int num_odometry_constraints = 1;
  const int num_loop_closure_constraints = 1;
  posegraph.constraints.resize(num_odometry_constraints);
  posegraph.vertex_poses = Eigen::MatrixXd::Zero(7, num_poses);
  posegraph.loop_closure_constraints.resize(num_loop_closure_constraints);

  posegraph.vertex_poses.block<4, 1>(0, 0) =
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 0) = pose::Position3D(0, 0, 0);
  posegraph.vertex_poses.block<4, 1>(0, 1) =
      Eigen::Quaterniond(
          0.9965159041385382, 0.06892005802323266, 0.020435594117957374,
          0.042290245850220926)
          .coeffs();
  posegraph.vertex_poses.block<3, 1>(4, 1) = pose::Position3D(1.13, 0, -0.2);

  // add odometry constraints
  Eigen::Matrix<double, 6, 6> dummy_covariance;
  dummy_covariance.setIdentity();
  posegraph.constraints[0] = PosegraphConstraint(
      0, 1, pose::Transformation(
                pose::Position3D(1, 0, 0), pose::Quaternion(0, 0, 0, 1)),
      dummy_covariance);
  // add loop closure constraints
  const double initial_switch_variable = 1.0;
  const double switch_variable_covariance = 1e-10;
  posegraph.loop_closure_constraints[0] = LoopClosureConstraint(
      1, 0, pose::Transformation(
                pose::Position3D(2, 0, 0), pose::Quaternion(0, 0, 0, 1)),
      dummy_covariance, switch_variable_covariance, initial_switch_variable);
}

void solveWithSwitchableConstraints(BlockPosegraph& posegraph) {  // NOLINT
  ceres::Problem problem;
  // ceres::LocalParameterization* quaternion_parameterization =
  //    new JplQuaternionParameterization;
  ceres::LocalParameterization* pose_parametrization(
      new JplPoseParameterization);

  for (const PosegraphConstraint& pgc : posegraph.constraints) {
    ceres::CostFunction* odometry_cost = new ceres::AutoDiffCostFunction<
        SixDoFBlockPoseErrorTerm, SixDoFBlockPoseErrorTerm::residualBlockSize,
        poseblocks::kPoseSize, poseblocks::kPoseSize>(
        new SixDoFBlockPoseErrorTerm(pgc.T_A_B_, pgc.T_A_B_covariance_));

    problem.AddResidualBlock(
        odometry_cost, nullptr,
        posegraph.vertex_poses.col(pgc.from_node_).data(),
        posegraph.vertex_poses.col(pgc.to_node_).data());
  }

  pose::Quaternion G_q_M;
  G_q_M.setIdentity();
  pose::Position3D G_p_M;
  G_p_M.setZero();

  const double switch_prior = 1.0;
  for (LoopClosureConstraint& lcc : posegraph.loop_closure_constraints) {
    ceres::CostFunction* loop_closure_cost = new ceres::AutoDiffCostFunction<
        LoopClosureBlockPoseErrorTerm,
        LoopClosureBlockPoseErrorTerm::residualBlockSize, poseblocks::kPoseSize,
        poseblocks::kPoseSize,
        LoopClosureBlockPoseErrorTerm::switchVariableBlockSize>(
        new LoopClosureBlockPoseErrorTerm(lcc.T_A_B_, lcc.T_A_B_covariance_));
    ceres::CostFunction* switch_variable_cost = new ceres::AutoDiffCostFunction<
        SwitchPriorErrorTermLegacy,
        SwitchPriorErrorTermLegacy::residualBlockSize,
        SwitchPriorErrorTermLegacy::switchVariableBlockSize>(
        new SwitchPriorErrorTermLegacy(
            switch_prior, lcc.switch_variable_covariance_));

    problem.AddResidualBlock(
        loop_closure_cost, NULL,
        posegraph.vertex_poses.col(lcc.from_node_).data(),
        posegraph.vertex_poses.col(lcc.to_node_).data(), &lcc.switch_variable_);

    problem.AddResidualBlock(switch_variable_cost, NULL, &lcc.switch_variable_);
  }

  // add parameterization for quaternion of each graph node
  LOG(INFO) << "Num cols: " << posegraph.vertex_poses.cols();
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

  EXPECT_ZERO_EIGEN(posegraph.vertex_poses.col(0).tail(3), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(1).tail(3), pose::Position3D(1, 0, 0), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(2).tail(3), pose::Position3D(1, 1, 0), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(3).tail(3), pose::Position3D(0, 1, 0), 1e-8);

  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(0).head(4),
      Eigen::Quaterniond(1, 0, 0, 0).coeffs(), 1e-6);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(1).head(4),
      Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2).coeffs(), 1e-6);
  Eigen::Quaterniond q_G_I;
  q_G_I.coeffs() = posegraph.vertex_poses.col(2).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(0, 0, 0, 1)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(3).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
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

  EXPECT_ZERO_EIGEN(posegraph.vertex_poses.col(0).tail(3), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(1).tail(3), pose::Position3D(1, 0, 0), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(2).tail(3), pose::Position3D(1, 1, 0), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(3).tail(3), pose::Position3D(0, 1, 0), 1e-5);
  Eigen::Quaterniond q_G_I;
  q_G_I.coeffs() = posegraph.vertex_poses.col(0).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(1).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(2).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(0, 0, 0, 1)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(3).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      1e-6);

  EXPECT_NEAR(posegraph.loop_closure_constraints[0].switch_variable_, 0, 1e-4);
}

// The test verifies if a posegraph with:
// * noisy initial guesses
// * perfect odometry constraints
// * perfect loop closure constraint
// will end up with an expected shape, i.e.:
//     >---v
//     |   |
// *---^---<
//     |
//     ^
// As the loop closure constraint is accurate, the switch variable should
// stay at 1.
TEST(PosegraphErrorTerms, SwitchableConstraints_7NodeGraphMatching) {
  BlockPosegraph posegraph;
  fillPosegraphWith7NodesMatched(posegraph);
  solveWithSwitchableConstraints(posegraph);

  EXPECT_ZERO_EIGEN(posegraph.vertex_poses.col(0).tail(3), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(1).tail(3), pose::Position3D(1, 0, 0), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(2).tail(3), pose::Position3D(2, 0, 0), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(3).tail(3), pose::Position3D(2, 1, 0), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(4).tail(3), pose::Position3D(1, 1, 0), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(5).tail(3), pose::Position3D(1, 0, 0), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(6).tail(3), pose::Position3D(1, -1, 0), 1e-8);

  Eigen::Quaterniond q_G_I;
  q_G_I.coeffs() = posegraph.vertex_poses.col(0).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(1).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(2).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(3).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(0, 0, 0, 1)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(4).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(5).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(6).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, -sqrt(2) / 2)),
      1e-6);

  EXPECT_NEAR(posegraph.loop_closure_constraints[0].switch_variable_, 1, 1e-5);
}

// The test verifies if a posegraph with:
// * noisy initial guesses
// * correct odometry constraints apart from edge 3->4, where rotations is wrong
//   and associated covariance very large
//        >---3
//        |   |
//    *---^---4
//        |
//        ^
// * perfect loop closure constraint
// will end up with an expected shape, i.e.:
//        >---3
//        |   |
//        ^   4
//        | /
//        ^
//      /
//     * (-1, -1)
//
// As the loop closure constraint is accurate, the switch variable should
// stay at 1.
TEST(PosegraphErrorTerms, SwitchableConstraints_7NodeGraphUnmatched) {
  BlockPosegraph posegraph;
  fillPosegraphWith7NodesUnmatched(posegraph);
  solveWithSwitchableConstraints(posegraph);

  EXPECT_ZERO_EIGEN(posegraph.vertex_poses.col(0).tail(3), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(1).tail(3), pose::Position3D(1, 0, 0), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(2).tail(3), pose::Position3D(2, 0, 0), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(3).tail(3), pose::Position3D(2, 1, 0), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(4).tail(3), pose::Position3D(1, 1, 0), 1e-5);
  EXPECT_ZERO_EIGEN(posegraph.vertex_poses.col(5).tail(3), 1e-5);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(6).tail(3), pose::Position3D(-1, -1, 0), 1e-5);

  Eigen::Quaterniond q_G_I;
  q_G_I.coeffs() = posegraph.vertex_poses.col(0).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(1).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(2).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(sqrt(2) / 2, 0, 0, sqrt(2) / 2)),
      1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(3).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(0, 0, 0, 1)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(4).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(0.38268343236508984, 0, 0, -0.9238795325112866)),
      1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(5).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(0.38268343236508984, 0, 0, -0.92387953251128662)),
      1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(6).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(
          Eigen::Quaterniond(0.38268343236508984, 0, 0, -0.9238795325112866)),
      1e-6);

  EXPECT_NEAR(posegraph.loop_closure_constraints[0].switch_variable_, 1, 1e-5);
}

// The test checks if a two nodes tied together with two constraints:
// * odometry constraint with translation (1, 0, 0)
// * loop closure constraint with translation (2, 0, 0)
// and exactly the same covariances will end up exactly "in the middle"
// between two values, i.e. (1.5, 0, 0).
//
// As the switch variable covariance is very low, the switch variable should
// stay at 1.
TEST(PosegraphErrorTerms, SwitchableConstraints_ContradictoryConstraints) {
  BlockPosegraph posegraph;
  fillPosegraphWith2ContradictoryConstraints(posegraph);
  solveWithSwitchableConstraints(posegraph);

  EXPECT_ZERO_EIGEN(posegraph.vertex_poses.col(0).tail(3), 1e-8);
  EXPECT_NEAR_EIGEN(
      posegraph.vertex_poses.col(1).tail(3), pose::Position3D(1.5, 0, 0), 1e-8);
  Eigen::Quaterniond q_G_I;
  q_G_I.coeffs() = posegraph.vertex_poses.col(0).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0)), 1e-6);
  q_G_I.coeffs() = posegraph.vertex_poses.col(1).head(4);
  EXPECT_NEAR_KINDR_QUATERNION(
      kindr::minimal::RotationQuaternion(q_G_I),
      kindr::minimal::RotationQuaternion(Eigen::Quaterniond(0, 0, 0, 1)), 1e-6);

  // switch variable should stay at 1 as it has very low covariance
  EXPECT_NEAR(posegraph.loop_closure_constraints[0].switch_variable_, 1, 1e-5);
}

MAPLAB_UNITTEST_ENTRYPOINT
