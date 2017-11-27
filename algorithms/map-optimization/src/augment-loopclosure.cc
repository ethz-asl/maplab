#include "map-optimization/augment-loopclosure.h"

#include <ceres-error-terms/loop-closure-edge-error-term.h>
#include <ceres-error-terms/switch-prior-error-term.h>

namespace map_optimization {

namespace {

void addLoopclosureEdges(
    const pose_graph::EdgeIdList& provided_edges,
    const bool use_switchable_constraints, const double cauchy_loss,
    OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  if (use_switchable_constraints) {
    LOG(INFO) << "Adding LC edges with switchable constraints.";
  } else {
    LOG(INFO) << "Adding LC edges with Cauchy loss.";
  }

  OptimizationStateBuffer* buffer =
      CHECK_NOTNULL(problem->getOptimizationStateBufferMutable());
  vi_map::VIMap* map = CHECK_NOTNULL(problem->getMapMutable());
  ceres_error_terms::ProblemInformation* problem_information =
      CHECK_NOTNULL(problem->getProblemInformationMutable());

  size_t num_residual_blocks_added = 0u;
  for (const pose_graph::EdgeId& edge_id : provided_edges) {
    pose_graph::Edge* edge =
        CHECK_NOTNULL(map->getEdgePtrAs<vi_map::Edge>(edge_id));
    CHECK(edge->getType() == pose_graph::Edge::EdgeType::kLoopClosure);

    vi_map::LoopClosureEdge& loop_closure_edge =
        edge->getAs<vi_map::LoopClosureEdge>();

    vi_map::Vertex& vertex_from = map->getVertex(loop_closure_edge.from());
    vi_map::Vertex& vertex_to = map->getVertex(loop_closure_edge.to());
    CHECK_NE(vertex_from.id(), vertex_to.id());

    const aslam::Transformation& T_A_B = loop_closure_edge.getT_A_B();
    const aslam::TransformationCovariance& T_A_B_covariance =
        loop_closure_edge.getT_A_BCovariance();

    // The error terms require a JPL convention for the rotation so let's
    // create a T_A_B transform with a rotation in this convention.
    Eigen::Matrix<double, 7, 1> q_AB__A_p_AB;
    q_AB__A_p_AB << T_A_B.getRotation().toImplementation().inverse().coeffs(),
        T_A_B.getPosition();

    double* vertex_from_q_IM__M_p_MI =
        buffer->get_vertex_q_IM__M_p_MI_JPL(loop_closure_edge.from());
    double* vertex_to_q_IM__M_p_MI =
        buffer->get_vertex_q_IM__M_p_MI_JPL(loop_closure_edge.to());

    problem->getProblemInformationMutable()->replaceParameterization(
        vertex_from_q_IM__M_p_MI,
        problem->getLocalParameterizations().pose_parameterization);
    problem->getProblemInformationMutable()->replaceParameterization(
        vertex_to_q_IM__M_p_MI,
        problem->getLocalParameterizations().pose_parameterization);

    vi_map::MissionBaseFrameId from_baseframe_id =
        map->getMissionForVertex(loop_closure_edge.from()).getBaseFrameId();
    vi_map::MissionBaseFrameId to_baseframe_id =
        map->getMissionForVertex(loop_closure_edge.to()).getBaseFrameId();
    const bool vertices_are_in_same_baseframe =
        from_baseframe_id == to_baseframe_id;

    // We only add one baseframe transformation for both vertices if
    // they are part of the same mission.
    using ceres_error_terms::LoopClosureEdgeErrorTerm;

    // In the current implementation, we reject outliers either using
    // switchable constraints or the Cauchy loss function. We never use both.
    std::shared_ptr<ceres::LossFunction> loss_function = nullptr;
    if (!use_switchable_constraints) {
      if (cauchy_loss > 0.0) {
        loss_function.reset(new ceres::CauchyLoss(cauchy_loss));
      }
      problem_information->setParameterBlockConstant(
          loop_closure_edge.getSwitchVariableMutable());
    }

    if (vertices_are_in_same_baseframe) {
      std::shared_ptr<ceres::CostFunction> loop_closure_cost(
          new ceres::AutoDiffCostFunction<
              LoopClosureEdgeErrorTerm,
              LoopClosureEdgeErrorTerm::kResidualBlockSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              LoopClosureEdgeErrorTerm::kSwitchVariableBlockSize>(
              new LoopClosureEdgeErrorTerm(q_AB__A_p_AB, T_A_B_covariance)));
      problem_information->addResidualBlock(
          ceres_error_terms::ResidualType::kLoopClosure, loop_closure_cost,
          loss_function, {vertex_from_q_IM__M_p_MI, vertex_to_q_IM__M_p_MI,
                          loop_closure_edge.getSwitchVariableMutable()});
    } else {
      std::shared_ptr<ceres::CostFunction> loop_closure_cost(
          new ceres::AutoDiffCostFunction<
              LoopClosureEdgeErrorTerm,
              LoopClosureEdgeErrorTerm::kResidualBlockSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              ceres_error_terms::poseblocks::kPoseSize,
              LoopClosureEdgeErrorTerm::kSwitchVariableBlockSize>(
              new LoopClosureEdgeErrorTerm(q_AB__A_p_AB, T_A_B_covariance)));

      double* baseframe_from_q_GM__G_p_GM =
          buffer->get_baseframe_q_GM__G_p_GM_JPL(from_baseframe_id);
      double* baseframe_to_q_GM__G_p_GM =
          buffer->get_baseframe_q_GM__G_p_GM_JPL(to_baseframe_id);
      problem_information->addResidualBlock(
          ceres_error_terms::ResidualType::kLoopClosure, loop_closure_cost,
          loss_function, {baseframe_from_q_GM__G_p_GM, vertex_from_q_IM__M_p_MI,
                          baseframe_to_q_GM__G_p_GM, vertex_to_q_IM__M_p_MI,
                          loop_closure_edge.getSwitchVariableMutable()});

      problem->getProblemInformationMutable()->setParameterBlockConstant(
          baseframe_from_q_GM__G_p_GM);
      problem->getProblemInformationMutable()->setParameterBlockConstant(
          baseframe_to_q_GM__G_p_GM);
    }

    if (use_switchable_constraints) {
      constexpr double kSwitchVariablePrior = 1.0;
      // The switch variable variance is taken from a flag. In case
      // it's not working, 1e-7 seems to be a reasonable starting point
      // for tuning.
      const double kSwitchVariableVariance =
          loop_closure_edge.getSwitchVariableVariance();

      using ceres_error_terms::SwitchPriorErrorTerm;
      std::shared_ptr<ceres::CostFunction> switch_variable_cost(
          new ceres::AutoDiffCostFunction<
              SwitchPriorErrorTerm, SwitchPriorErrorTerm::residualBlockSize,
              SwitchPriorErrorTerm::switchVariableBlockSize>(
              new SwitchPriorErrorTerm(
                  kSwitchVariablePrior, kSwitchVariableVariance)));
      problem_information->addResidualBlock(
          ceres_error_terms::ResidualType::kSwitchVariable,
          switch_variable_cost, nullptr,
          {loop_closure_edge.getSwitchVariableMutable()});

      constexpr int kSwitchVariableIndexInParamBlock = 0;
      problem_information->setParameterBlockBounds(
          kSwitchVariableIndexInParamBlock, 0., 1.,
          loop_closure_edge.getSwitchVariableMutable());
    }

    ++num_residual_blocks_added;
  }

  // Open the initial yaw and position DoF.
  MissionClusterGaugeFixes relaxation_fix;
  relaxation_fix.position_dof_fixed = false;
  relaxation_fix.rotation_dof_fixed = FixedRotationDoF::kNone;
  relaxation_fix.scale_fixed = false;

  const size_t num_clusters = problem->getMissionCoobservationClusters().size();
  std::vector<MissionClusterGaugeFixes> relaxation_fixes(
      num_clusters, relaxation_fix);

  const std::vector<MissionClusterGaugeFixes>* already_applied_fixes =
      problem->getAppliedGaugeFixesForInitialVertices();
  if (already_applied_fixes) {
    std::vector<MissionClusterGaugeFixes> merged_fixes;
    mergeGaugeFixes(relaxation_fixes, *already_applied_fixes, &merged_fixes);
    problem->applyGaugeFixesForInitialVertices(merged_fixes);
  } else {
    problem->applyGaugeFixesForInitialVertices(relaxation_fixes);
  }

  LOG(INFO) << "Added " << num_residual_blocks_added
            << " loop-closure error terms.";
}

}  // namespace

void augmentViProblemWithLoopclosureEdges(OptimizationProblem* problem) {
  CHECK_NOTNULL(problem);

  const vi_map::VIMap& map = *CHECK_NOTNULL(problem->getMapMutable());
  const vi_map::MissionIdSet& missions_to_optimize = problem->getMissionIds();

  pose_graph::EdgeIdList edges;
  map.getAllEdgeIds(&edges);

  pose_graph::EdgeIdList lc_edges;
  for (const pose_graph::EdgeId& edge_id : edges) {
    if (map.getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kLoopClosure) {
      const vi_map::Edge& edge = map.getEdgeAs<vi_map::Edge>(edge_id);
      const vi_map::Vertex& vertex_from = map.getVertex(edge.from());
      const vi_map::Vertex& vertex_to = map.getVertex(edge.to());

      // Only optimize the edge if it links vertices of missions that are
      // suppposed to be optimized.
      if (missions_to_optimize.count(vertex_from.getMissionId()) > 0u &&
          missions_to_optimize.count(vertex_to.getMissionId()) > 0u) {
        lc_edges.push_back(edge_id);
      }
    }
  }

  addLoopclosureEdges(lc_edges, true, 0, problem);
}

}  // namespace map_optimization
