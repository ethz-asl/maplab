#include "map-optimization-legacy-plugin/legacy-optimizer-plugin.h"

#include <ceres/ceres.h>
#include <console-common/basic-console-plugin.h>
#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <map-optimization-legacy/ba-optimization-options.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

#include "map-optimization-legacy-plugin/vi-map-optimizer-legacy.h"

DECLARE_string(map_mission);

namespace map_optimization_legacy_plugin {

OptimizerPlugin::OptimizerPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"legacy_optimize_vision_only", "loptv"},
      [this]() -> int { return optimizeVisionOnly(); },
      "Legacy - Vision only BA.", common::Processing::Sync);

  addCommand(
      {"legacy_optimize", "loptvi"},
      [this]() -> int { return optimizeVisualInertial(); },
      "Legacy - Visual inertial BA.", common::Processing::Sync);

  addCommand(
      {"legacy_optimize_single_mission", "loptom"},
      [this]() -> int { return optimizeOneMission(); },
      "Legacy - Optimize one mission while holding other parameters fixed.",
      common::Processing::Sync);

  addCommand(
      {"legacy_relax", "lrealx"}, [this]() -> int { return relax(); },
      "Legacy - Use posegraph relaxation on the entire map. Loop closure edges "
      "need to "
      "be present already.",
      common::Processing::Sync);
}

int OptimizerPlugin::optimizeVisionOnly() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  map_optimization_legacy::BaOptimizationOptions options;
  options.include_visual = true;
  options.include_inertial = false;
  options.include_wheel_odometry = false;
  options.include_gps = false;
  options.position_only_gps = false;

  VIMapOptimizer optimizer(plotter_, kSignalHandlerEnabled);
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  return optimizer.optimize(options, map.get());
}

int OptimizerPlugin::optimizeVisualInertial() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  VIMapOptimizer optimizer(plotter_, kSignalHandlerEnabled);
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  map_optimization_legacy::BaOptimizationOptions options;
  return optimizer.optimize(options, map.get());
}

int OptimizerPlugin::optimizeOneMission() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionId mission_id;
  map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);

  if (!mission_id.isValid()) {
    LOG(ERROR) << "The given mission \"" << FLAGS_map_mission
               << "\" is not valid.";
    return common::kUnknownError;
  }

  VIMapOptimizer optimizer(plotter_, kSignalHandlerEnabled);
  ceres::Solver::Summary summary;
  map_optimization_legacy::BaOptimizationOptions options;
  options.fix_not_selected_missions = true;
  CHECK(options.mission_ids_to_optimize.emplace(mission_id).second);
  return optimizer.optimize(options, map.get(), &summary);
}

int OptimizerPlugin::relax() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  // Check if we have any loop closure edges.
  if (!map.get()->hasEdgesOfType(pose_graph::Edge::EdgeType::kLoopClosure)) {
    LOG(ERROR) << "Map has no loop closure edges. Please run loopclosure first"
               << " and then rerun this command.";
    return common::kStupidUserError;
  }

  VIMapOptimizer optimizer(plotter_, kSignalHandlerEnabled);
  const int status = optimizer.relaxPosegraphBasedOnLoopclosureEdges(map.get());
  LOG(INFO) << "Removing loop closure edges.";
  map.get()->removeLoopClosureEdges();
  return status;
}

}  // namespace map_optimization_legacy_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    map_optimization_legacy_plugin::OptimizerPlugin);
