#include "map-anchoring-plugin/anchoring-plugin.h"

#include <console-common/console.h>
#include <map-anchoring/map-anchoring.h>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>

DECLARE_string(map_mission);

namespace map_anchoring_plugin {

AnchoringPlugin::AnchoringPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(CHECK_NOTNULL(console), plotter) {
  addCommand(
      {"set_mission_baseframe_to_known", "sbk"},
      [this]() -> int {
        constexpr bool kIsKnown = true;
        return setMissionBaseframeKnownState(kIsKnown);
      },
      "Mark mission baseframe of a mission as known.",
      common::Processing::Sync);
  addCommand(
      {"set_mission_baseframe_to_unknown"},
      [this]() -> int {
        constexpr bool kIsKnown = false;
        return setMissionBaseframeKnownState(kIsKnown);
      },
      "Mark mission baseframe of a mission as unknown.",
      common::Processing::Sync);

  addCommand(
      {"set_all_mission_baseframes_to_unknown", "sabu"},
      [this]() -> int {
        constexpr bool kIsKnown = false;
        return setAllMissionBaseframesKnownState(kIsKnown);
      },
      "Mark all mission baseframes as unknown.", common::Processing::Sync);
  addCommand(
      {"set_all_mission_baseframes_to_known"},
      [this]() -> int {
        constexpr bool kIsKnown = true;
        return setAllMissionBaseframesKnownState(kIsKnown);
      },
      "Mark all mission baseframes as known.", common::Processing::Sync);

  addCommand(
      {"anchor_mission", "am"}, [this]() -> int { return anchorMission(); },
      "Try to anchor this mission to another mission with known baseframe.",
      common::Processing::Sync);

  addCommand(
      {"anchor_all_missions", "aam"},
      [this]() -> int { return anchorAllMissions(); },
      "Try to anchor all missions to another mission with known baseframe.",
      common::Processing::Sync);
}

int AnchoringPlugin::setMissionBaseframeKnownState(
    const bool baseframe_known_state) const {
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

  map_anchoring::setMissionBaseframeKnownState(
      mission_id, baseframe_known_state, map.get());

  return common::kSuccess;
}

int AnchoringPlugin::setAllMissionBaseframesKnownState(
    const bool baseframe_known_state) const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  vi_map::MissionIdList all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);

  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    map_anchoring::setMissionBaseframeKnownState(
        mission_id, baseframe_known_state, map.get());
  }

  return common::kSuccess;
}

int AnchoringPlugin::anchorMission() const {
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

  map_anchoring::anchorMission(mission_id, map.get());
  if (plotter_ != nullptr) {
    plotter_->visualizeMap(*map);
  }

  return common::kSuccess;
}

int AnchoringPlugin::anchorAllMissions() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);
  const bool success = map_anchoring::anchorAllMissions(map.get());
  if (plotter_ != nullptr) {
    plotter_->visualizeMap(*map);
  }

  return success ? common::kSuccess : common::kUnknownError;
}

}  // namespace map_anchoring_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    map_anchoring_plugin::AnchoringPlugin);
