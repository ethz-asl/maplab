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

DEFINE_bool(
    lc_relax_merge_landmarks, false,
    "If enabled, the loop closure part of the pose graph relaxation algorithm "
    "will also merge the landmarks in addition to adding temporary loop "
    "closure edges.");

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace map_optimization {

VIMapRelaxation::VIMapRelaxation(
    const visualization::ViwlsGraphRvizPlotter* plotter,
    bool signal_handler_enabled)
    : plotter_(plotter), signal_handler_enabled_(signal_handler_enabled) {}

void VIMapRelaxation::visualizePosegraph(const vi_map::VIMap& map) const {
  if (plotter_ == nullptr) {
    VLOG(3) << "You need to initialize with a plotter";
    return;
  }

  plotter_->visualizeMap(map);
}

void VIMapRelaxation::detectLoopclosures(
    vi_map::MissionIdSet mission_ids, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  loop_detector_node::LoopDetectorNode loop_detector;

  for (const vi_map::MissionId& mission_id : mission_ids) {
    loop_detector.addMissionToDatabase(mission_id, *map);
  }

  const bool kMergeLandmarks = FLAGS_lc_relax_merge_landmarks;
  constexpr bool kAddLoopclosureEdges = true;

  pose::Transformation T_G_M2;
  vi_map::LoopClosureConstraintVector inlier_constraints;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    loop_detector.detectLoopClosuresMissionToDatabase(
        mission_id, kMergeLandmarks, kAddLoopclosureEdges, map, &T_G_M2,
        &inlier_constraints);
  }
}

bool VIMapRelaxation::findLoopClosuresAndSolveRelaxation(
    const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  ceres::Solver::Options solver_options =
      map_optimization::initSolverOptionsFromFlags();

  return findLoopClosuresAndSolveRelaxation(
      solver_options, mission_id_list, map);
}

bool VIMapRelaxation::findLoopClosuresAndSolveRelaxation(
    const ceres::Solver::Options& solver_options,
    const vi_map::MissionIdList& mission_id_list, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  vi_map::MissionIdSet mission_ids(
      mission_id_list.begin(), mission_id_list.end());

  detectLoopclosures(mission_ids, map);

  const int num_lc_edges = numLoopclosureEdges(*map);
  if (num_lc_edges == 0) {
    LOG(INFO) << "No loop closure edges found, relaxation cannot proceed.";
    return false;
  }
  LOG(INFO) << num_lc_edges
            << " loopclosure edges found and TEMPORARILY added to the map.";

  return solveRelaxation(solver_options, mission_ids, map);
}

bool VIMapRelaxation::solveRelaxation(
    const ceres::Solver::Options& solver_options,
    const vi_map::MissionIdSet& mission_ids, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

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

  options.add_loop_closure_edges = true;

  map_optimization::OptimizationProblem::UniquePtr optimization_problem(
      map_optimization::constructOptimizationProblem(
          mission_ids, options, map));
  CHECK(optimization_problem);

  std::vector<std::shared_ptr<ceres::IterationCallback>> callbacks;
  if (FLAGS_ba_enable_signal_handler) {
    map_optimization::appendSignalHandlerCallback(&callbacks);
  }

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

  map_optimization::solve(
      solver_options_with_callbacks, optimization_problem.get());

  visualizePosegraph(*map);

  const size_t number_of_loop_closure_edges_removed =
      map->removeLoopClosureEdges();
  LOG(INFO) << "Removed " << number_of_loop_closure_edges_removed
            << " loop closures edges.";

  return true;
}

}  // namespace map_optimization
