#include "map-optimization-plugin/optimizer-plugin.h"

#include <ceres/ceres.h>
#include <console-common/basic-console-plugin.h>
#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <map-optimization/outlier-rejection-solver.h>
#include <map-optimization/vi-optimization-builder.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

#include "map-optimization/vi-map-optimizer.h"
#include "map-optimization/vi-map-relaxation.h"

DEFINE_bool(
    ba_use_outlier_rejection_solver, true,
    "Reject outlier landmarks during the solve?");
DECLARE_string(map_mission);
DECLARE_string(map_mission_list);

namespace map_optimization_plugin {
OptimizerPlugin::OptimizerPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"noptimize_visual", "optv"},
      [this]() -> int {
        constexpr bool kVisualOnly = true;
        return optimizeVisualInertial(
            kVisualOnly, FLAGS_ba_use_outlier_rejection_solver);
      },
      "Visual optimization over the selected missions "
      "(per default all).",
      common::Processing::Sync);
  addCommand(
      {"optimize_visual_inertial", "optvi"},
      [this]() -> int {
        constexpr bool kVisualOnly = false;
        return optimizeVisualInertial(
            kVisualOnly, FLAGS_ba_use_outlier_rejection_solver);
      },
      "Visual-inertial optimization over the selected missions "
      "(per default all).",
      common::Processing::Sync);
  addCommand(
      {"relax"}, [this]() -> int { return relaxMap(); }, "nRelax posegraph.",
      common::Processing::Sync);
  addCommand(
      {"relax_missions_independently", "srelax"},
      [this]() -> int { return relaxMapMissionsSeparately(); },
      "Relax missions separately.", common::Processing::Sync);
}

int OptimizerPlugin::optimizeVisualInertial(
    bool visual_only, bool outlier_rejection) {
  // Select map and missions to optimize.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList missions_to_optimize_list;
  if (!FLAGS_map_mission.empty()) {
    if (!FLAGS_map_mission_list.empty()) {
      LOG(ERROR) << "Please provide only one of --map_mission and "
                 << "--map_mission_list.";
      return common::kStupidUserError;
    }
    vi_map::MissionId mission_id;
    if (!map->hexStringToMissionIdIfValid(FLAGS_map_mission, &mission_id)) {
      LOG(ERROR) << "The given mission id \"" << FLAGS_map_mission
                 << "\" is not valid.";
      return common::kStupidUserError;
    }
    missions_to_optimize_list.emplace_back(mission_id);
  } else if (!FLAGS_map_mission_list.empty()) {
    if (!vi_map::csvIdStringToIdList(
            FLAGS_map_mission_list, &missions_to_optimize_list)) {
      LOG(ERROR) << "The provided CSV mission id list is not valid!";
      return common::kStupidUserError;
    }
  } else {
    map->getAllMissionIds(&missions_to_optimize_list);
  }
  vi_map::MissionIdSet missions_to_optimize(
      missions_to_optimize_list.begin(), missions_to_optimize_list.end());

  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();
  if (visual_only) {
    options.add_inertial_constraints = false;
  }

  map_optimization::VIMapOptimizer optimizer(plotter_, kSignalHandlerEnabled);
  bool success;
  if (outlier_rejection) {
    map_optimization::OutlierRejectionSolverOptions outlier_rejection_options =
        map_optimization::OutlierRejectionSolverOptions::initFromFlags();
    success = optimizer.optimizeVisualInertial(
        options, missions_to_optimize, &outlier_rejection_options, map.get());
  } else {
    success = optimizer.optimizeVisualInertial(
        options, missions_to_optimize, nullptr, map.get());
  }
  if (!success) {
    return common::kUnknownError;
  }
  return common::kSuccess;
}

int OptimizerPlugin::relaxMap() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  map_optimization::VIMapRelaxation relaxation(plotter_, kSignalHandlerEnabled);
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_id_list;
  map.get()->getAllMissionIds(&mission_id_list);

  const bool success = relaxation.relax(mission_id_list, map.get());

  if (!success) {
    return common::kUnknownError;
  }
  return common::kSuccess;
}

int OptimizerPlugin::relaxMapMissionsSeparately() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  map_optimization::VIMapRelaxation relaxation(plotter_, kSignalHandlerEnabled);
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_id_list;
  map.get()->getAllMissionIds(&mission_id_list);

  for (const vi_map::MissionId& mission_id : mission_id_list) {
    relaxation.relax({mission_id}, map.get());
  }

  return common::kSuccess;
}

}  // namespace map_optimization_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    map_optimization_plugin::OptimizerPlugin);
