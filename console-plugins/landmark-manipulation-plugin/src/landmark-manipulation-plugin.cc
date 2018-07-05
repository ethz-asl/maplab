#include "landmark-manipulation-plugin/landmark-manipulation-plugin.h"

#include <console-common/basic-console-plugin.h>
#include <console-common/console.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <map-manager/map-manager.h>
#include <vi-map-helpers/vi-map-landmark-quality-evaluation.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

DECLARE_string(map_mission);

namespace landmark_manipulation_plugin {

LandmarkManipulationPlugin::LandmarkManipulationPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"retriangulate_landmarks", "rtl"},
      [this]() -> int { return retriangulateLandmarks(); },
      "Retriangulate all landmarks.", common::Processing::Sync);
  addCommand(
      {"evaluate_landmark_quality", "elq"},
      [this]() -> int { return evaluateLandmarkQuality(); },
      "Evaluates and sets the landmark quality of all landmarks.",
      common::Processing::Sync);
  addCommand(
      {"reset_landmark_quality", "rlq"},
      [this]() -> int { return resetLandmarkQualityToUnknown(); },
      "Reset the landmark quality of all landmarks to unknown.",
      common::Processing::Sync);
  addCommand(
      {"init_track_landmarks", "itl"},
      [this]() -> int { return initTrackLandmarks(); },
      "Initialize all unprocessed but tracked landmarks.",
      common::Processing::Sync);
  addCommand(
      {"remove_bad_landmarks", "rbl"},
      [this]() -> int { return removeBadLandmarks(); },
      "Delete all landmarks that are marked as bad.", common::Processing::Sync);
}

int LandmarkManipulationPlugin::retriangulateLandmarks() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  if (!FLAGS_map_mission.empty()) {
    vi_map::MissionId mission_id;
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      return common::kStupidUserError;
    }
    mission_ids.emplace_back(mission_id);
  } else {
    map->getAllMissionIds(&mission_ids);
  }

  landmark_triangulation::retriangulateLandmarks(mission_ids, map.get());
  return common::kSuccess;
}

int LandmarkManipulationPlugin::evaluateLandmarkQuality() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids_to_process;
  if (!FLAGS_map_mission.empty()) {
    vi_map::MissionId mission_id;
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      return common::kStupidUserError;
    } else {
      mission_ids_to_process.emplace_back(mission_id);
    }
  } else {
    map->getAllMissionIds(&mission_ids_to_process);
  }

  vi_map_helpers::evaluateLandmarkQuality(mission_ids_to_process, map.get());
  return common::kSuccess;
}

int LandmarkManipulationPlugin::resetLandmarkQualityToUnknown() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids_to_process;
  if (!FLAGS_map_mission.empty()) {
    vi_map::MissionId mission_id;
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      return common::kStupidUserError;
    } else {
      mission_ids_to_process.emplace_back(mission_id);
    }
  } else {
    map->getAllMissionIds(&mission_ids_to_process);
  }

  vi_map_helpers::resetLandmarkQualityToUnknown(
      mission_ids_to_process, map.get());
  return common::kSuccess;
}

int LandmarkManipulationPlugin::initTrackLandmarks() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids_to_process;
  if (!FLAGS_map_mission.empty()) {
    vi_map::MissionId mission_id;
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      return common::kStupidUserError;
    } else {
      mission_ids_to_process.emplace_back(mission_id);
    }
  } else {
    map->getAllMissionIds(&mission_ids_to_process);
  }

  size_t mission_idx = 0u;
  vi_map_helpers::VIMapManipulation manipulation(map.get());
  for (const vi_map::MissionId& mission_id : mission_ids_to_process) {
    VLOG(1) << "Processing mission " << mission_idx + 1 << " of "
            << mission_ids_to_process.size();
    const size_t num_new_landmarks =
        manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(
            mission_id);
    landmark_triangulation::retriangulateLandmarksOfMission(
        mission_id, map.get());
    VLOG(1) << "Initialized " << num_new_landmarks << " new landmarks.";
    ++mission_idx;
  }
  return common::kSuccess;
}

int LandmarkManipulationPlugin::removeBadLandmarks() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map_helpers::VIMapManipulation manipulation(map.get());
  const size_t num_removed = manipulation.removeBadLandmarks();
  LOG(INFO) << "Removed " << num_removed << " bad landmark(s).";
  return common::kSuccess;
}

}  // namespace landmark_manipulation_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    landmark_manipulation_plugin::LandmarkManipulationPlugin);
