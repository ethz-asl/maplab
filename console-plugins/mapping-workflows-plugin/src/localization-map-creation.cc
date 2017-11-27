#include "mapping-workflows-plugin/localization-map-creation.h"

#include <algorithm>

#include <glog/logging.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <loop-closure-plugin/vi-map-merger.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/vi-map-optimizer.h>
#include <map-optimization/vi-map-relaxation.h>
#include <map-sparsification-plugin/keyframe-pruning.h>
#include <vi-map-helpers/vi-map-landmark-quality-evaluation.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/vi-map.h>

DECLARE_uint64(vi_map_landmark_quality_min_observers);

namespace mapping_workflows_plugin {
// Processes a raw map containing landmarks into a localization summary map.
// It runs the following steps:
// 1) landmark intialization,
// 2) retriangulation,
// 3) keyframing,
// 4) landmark quality evaluation,
// 5) visual-inertial batch optimization.
// 6) relaxation,
// 7) loopclosure,
// 8) visual-inertial batch optimization.
int processVIMapToLocalizationMap(
    bool initialize_landmarks,
    const map_sparsification::KeyframingHeuristicsOptions& keyframe_options,
    vi_map::VIMap* map, visualization::ViwlsGraphRvizPlotter* plotter) {
  // plotter is optional.
  CHECK_NOTNULL(map);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  if (mission_ids.size() != 1u) {
    LOG(WARNING) << "Only single mission supported. Aborting.";
    return common::CommandStatus::kStupidUserError;
  }
  const vi_map::MissionId& mission_id = mission_ids.front();

  // Initialize landmarks by triangulation.
  if (initialize_landmarks) {
    vi_map_helpers::VIMapManipulation manipulation(map);
    manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(
        mission_id);
    landmark_triangulation::retriangulateLandmarksOfMission(mission_id, map);
  }
  CHECK_GT(map->numLandmarks(), 0u);

  // Select keyframes along the mission. Unconditionally add the last vertex as
  // a keyframe if it isn't a keyframe already.
  int retval = map_sparsification_plugin::keyframeMapBasedOnHeuristics(
      keyframe_options, mission_id, plotter, map);
  if (retval != common::CommandStatus::kSuccess) {
    LOG(ERROR) << "Keyframing failed! Aborting.";
    return retval;
  }

  // Evaluate the quality of landmarks after keyframing the map.
  FLAGS_vi_map_landmark_quality_min_observers = 2;
  vi_map_helpers::evaluateLandmarkQuality(map);

  // Common options for the subsequent optimizations.
  ceres::Solver::Options solver_options =
      map_optimization::initSolverOptionsFromFlags();
  constexpr bool kEnableSignalHandler = true;
  map_optimization::VIMapOptimizer optimizer(plotter, kEnableSignalHandler);
  map_optimization::ViProblemOptions vi_problem_options =
      map_optimization::ViProblemOptions::initFromGFlags();

  constexpr int kNumInitialOptviIterations = 3;
  solver_options.max_num_iterations = kNumInitialOptviIterations;

  // Initial visual-inertial optimization.
  bool success = optimizer.optimizeVisualInertial(
      vi_problem_options, solver_options, {mission_id}, nullptr, map);
  if (!success) {
    LOG(ERROR) << "Optimization failed! Aborting.";
    return common::CommandStatus::kUnknownError;
  }

  // Overwrite the number of iterations to a reasonable value.
  // TODO(dymczykm) A temporary solution for the optimization not to take too
  // long. Better termination conditions are necessary.
  constexpr int kMaxNumIterations = 10;
  solver_options.max_num_iterations = kMaxNumIterations;

  // Relax the map.
  map_optimization::VIMapRelaxation relaxation(plotter, kEnableSignalHandler);
  success = relaxation.relax(solver_options, {mission_id}, map);
  if (!success) {
    LOG(WARNING) << "Pose-graph relaxation failed, but this might be fine if "
                 << "no loopclosures are present in the dataset.";
  }

  // Loop-close the map.
  loop_closure_plugin::VIMapMerger merger(map, plotter);
  retval = merger.findLoopClosuresBetweenAllMissions();
  if (retval != common::CommandStatus::kSuccess) {
    LOG(ERROR) << "Loop-closure failed! Aborting.";
    return retval;
  }

  // Optimize the map.
  success = optimizer.optimizeVisualInertial(
      vi_problem_options, solver_options, {mission_id}, nullptr, map);
  if (!success) {
    LOG(ERROR) << "Optimization failed! Aborting.";
    return common::CommandStatus::kUnknownError;
  }

  return common::CommandStatus::kSuccess;
}

}  // namespace mapping_workflows_plugin
