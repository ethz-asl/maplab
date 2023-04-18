#include "loop-closure-plugin/loop-closure-plugin.h"

#include <console-common/console.h>
#include <descriptor-projection/train-projection-matrix.h>
#include <map-manager/map-manager.h>
#include <posegraph/pose-graph.h>
#include <posegraph/unique-id.h>
#include <vi-map/vi-map-serialization.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

#include "loop-closure-plugin/vi-map-merger.h"

DECLARE_string(map_mission);

namespace loop_closure_plugin {

LoopClosurePlugin::LoopClosurePlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  CHECK_NOTNULL(console);

  addCommand(
      {"lc", "loopclosure_all_missions"},
      [this]() -> int { return findLoopClosuresBetweenAllMissions(); },
      "Find loop closures between all missions.", common::Processing::Sync);

  addCommand(
      {"lcom", "loopclosure_one_mission"},
      [this]() -> int { return findLoopClosuresInOneMission(); },
      "Find loop closures in one mission.", common::Processing::Sync);

  addCommand(
      {"dlc", "delete_loopclosure_edges"},
      [this]() -> int { return deleteAllLoopClosureEdges(); },
      "Delete loop closure edges of all missions.", common::Processing::Sync);

  addCommand(
      {"train_projection_matrix"},
      [this]() -> int {
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        descriptor_projection::TrainProjectionMatrix(*map);

        return common::kSuccess;
      },
      "Train a new descriptor projection matrix based on the selected map. Use "
      "--lc_projection_matrix_filename to specify the target file for the "
      "projecton matrix.",
      common::Processing::Sync);
}

bool areQualitiesOfAllLandmarksSet(const vi_map::VIMap& map) {
  vi_map::LandmarkIdList landmark_ids;
  map.getAllLandmarkIds(&landmark_ids);
  // Check all landmarks to ensure that their quality is not unknown.
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (map.getLandmark(landmark_id).getQuality() ==
        vi_map::Landmark::Quality::kUnknown) {
      return false;
    }
  }
  return true;
}

int LoopClosurePlugin::findLoopClosuresBetweenAllMissions() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (!areQualitiesOfAllLandmarksSet(*map)) {
    LOG(ERROR) << "Some landmarks are of unknown quality. Update them with the "
               << "elq command.";
    return common::kStupidUserError;
  }

  VIMapMerger merger(map.get(), getPlotterUnsafe());
  return merger.findLoopClosuresBetweenAllMissions();
}

int LoopClosurePlugin::findLoopClosuresInOneMission() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (!areQualitiesOfAllLandmarksSet(*map)) {
    LOG(ERROR) << "Some landmarks are of unknown quality. Update them with the "
               << "elq command.";
    return common::kStupidUserError;
  }

  if (FLAGS_map_mission.empty()) {
    LOG(ERROR) << "Specify a valid mission with -map_mission.";
    return common::kStupidUserError;
  }

  vi_map::MissionIdList mission_ids;
  vi_map::MissionId mission_id;
  map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
  mission_ids.emplace_back(mission_id);

  VIMapMerger merger(map.get(), getPlotterUnsafe());
  return merger.findLoopClosuresBetweenMissions(mission_ids);
}

int LoopClosurePlugin::deleteAllLoopClosureEdges() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  const size_t number_of_loop_closure_edges_removed =
      map->removeLoopClosureEdges();
  LOG(INFO) << "Removed " << number_of_loop_closure_edges_removed
            << " loop closures edges.";
  return common::kSuccess;
}

}  // namespace loop_closure_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    loop_closure_plugin::LoopClosurePlugin);
