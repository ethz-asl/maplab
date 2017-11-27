#include "map-optimization/vi-map-relaxation.h"

#include <functional>
#include <string>
#include <unordered_map>

#include <loop-closure-handler/loop-detector-node.h>
#include <map-optimization/augment-loopclosure.h>
#include <map-optimization/callbacks.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/solver.h>
#include <map-optimization/vi-optimization-builder.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace map_optimization {

VIMapRelaxation::VIMapRelaxation(
    visualization::ViwlsGraphRvizPlotter* plotter, bool signal_handler_enabled)
    : plotter_(plotter), signal_handler_enabled_(signal_handler_enabled) {}

void VIMapRelaxation::visualizePosegraph(const vi_map::VIMap& map) const {
  if (plotter_ == nullptr) {
    VLOG(3) << "You need to initialize with a plotter";
    return;
  }

  plotter_->visualizeMap(map);
}

int VIMapRelaxation::numLoopclosureEdges(const vi_map::VIMap& map) const {
  pose_graph::EdgeIdList edges;
  map.getAllEdgeIds(&edges);

  int num_lc_edges = 0;
  for (const pose_graph::EdgeId edge_id : edges) {
    if (map.getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kLoopClosure) {
      ++num_lc_edges;
    }
  }

  return num_lc_edges;
}

void VIMapRelaxation::detectLoopclosures(
    vi_map::MissionIdSet mission_ids, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  loop_detector_node::LoopDetectorNode loop_detector;

  for (const vi_map::MissionId& mission_id : mission_ids) {
    loop_detector.addMissionToDatabase(mission_id, *map);
  }

  constexpr bool kMergeLandmarks = false;
  constexpr bool kAddLoopclosureEdges = true;

  int num_vertex_candidate_links;
  double summary_landmark_match_inlier_ratio;

  pose::Transformation T_G_M2;
  vi_map::LoopClosureConstraintVector inlier_constraints;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    loop_detector.detectLoopClosuresMissionToDatabase(
        mission_id, kMergeLandmarks, kAddLoopclosureEdges,
        &num_vertex_candidate_links, &summary_landmark_match_inlier_ratio, map,
        &T_G_M2, &inlier_constraints);
  }
}

bool VIMapRelaxation::relax(
    const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  ceres::Solver::Options solver_options =
      map_optimization::initSolverOptionsFromFlags();

  return relax(solver_options, mission_id_list, map);
}

bool VIMapRelaxation::relax(
    const ceres::Solver::Options& solver_options,
    const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  vi_map::MissionIdSet mission_ids(
      mission_id_list.begin(), mission_id_list.end());

  detectLoopclosures(mission_ids, map);

  const int num_lc_edges = numLoopclosureEdges(*map);
  if (num_lc_edges == 0) {
    LOG(WARNING) << "No loop closure edges found, relaxation cannot proceed.";
    return false;
  }
  LOG(INFO) << num_lc_edges << " loopclosure edges found.";

  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();

  // Specific relaxation options.
  options.fix_accel_bias = true;
  options.fix_gyro_bias = true;
  options.fix_velocity = true;

  options.fix_intrinsics = true;
  options.fix_extrinsics_rotation = true;
  options.fix_extrinsics_translation = true;
  options.fix_landmark_positions = true;

  map_optimization::OptimizationProblem* optimization_problem =
      map_optimization::constructViProblem(mission_ids, options, map);
  CHECK_NOTNULL(optimization_problem);

  augmentViProblemWithLoopclosureEdges(optimization_problem);

  std::vector<std::shared_ptr<ceres::IterationCallback>> callbacks;
  map_optimization::appendSignalHandlerCallback(&callbacks);

  if (plotter_) {
    constexpr int kVisualizeEveryN = 1;
    map_optimization::appendVisualizationCallbacks(
        kVisualizeEveryN,
        *optimization_problem->getOptimizationStateBufferMutable(), *plotter_,
        map, &callbacks);
  }
  ceres::Solver::Options solver_options_with_callbacks = solver_options;
  map_optimization::addCallbacksToSolverOptions(
      callbacks, &solver_options_with_callbacks);

  map_optimization::solve(solver_options_with_callbacks, optimization_problem);

  visualizePosegraph(*map);

  optimization_problem->getOptimizationStateBufferMutable()
      ->copyAllStatesBackToMap(map);

  const size_t number_of_loop_closure_edges_removed =
      map->removeLoopClosureEdges();
  LOG(INFO) << "Removed " << number_of_loop_closure_edges_removed
            << " loop closures edges.";

  return true;
}

}  // namespace map_optimization
