#include "map-sparsification-plugin/map-sparsification-plugin.h"

#include <string>
#include <vector>

#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <map-sparsification-plugin/landmark-sparsification.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

#include "map-sparsification-plugin/keyframe-pruning.h"

DECLARE_string(map_mission);
DECLARE_string(map_mission_list);

DEFINE_int32(
    num_landmarks_to_keep, -1,
    "Number of landmarks to keep after sparsification.");

namespace map_sparsification_plugin {

MapSparsificationPlugin::MapSparsificationPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"keyframe_heuristic", "kfh"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        // Select mission.
        vi_map::MissionIdList missions_to_keyframe;
        if (!FLAGS_map_mission.empty()) {
          if (!FLAGS_map_mission_list.empty()) {
            LOG(ERROR) << "Please provide only one of --map_mission and "
                       << "--map_mission_list.";
            return common::kStupidUserError;
          }
          vi_map::MissionId mission_id;
          if (!map->hexStringToMissionIdIfValid(
                  FLAGS_map_mission, &mission_id)) {
            LOG(ERROR) << "The given mission id \"" << FLAGS_map_mission
                       << "\" is not valid.";
            return common::kStupidUserError;
          }
          missions_to_keyframe.emplace_back(mission_id);
        } else if (!FLAGS_map_mission_list.empty()) {
          if (!vi_map::csvIdStringToIdList(
                  FLAGS_map_mission_list, &missions_to_keyframe)) {
            LOG(ERROR) << "The provided CSV mission id list is not valid!";
            return common::kStupidUserError;
          }
        } else {
          map->getAllMissionIds(&missions_to_keyframe);
        }

        if (missions_to_keyframe.empty()) {
          LOG(ERROR) << "No missions found to keyframe.";
          return common::kStupidUserError;
        }

        using map_sparsification::KeyframingHeuristicsOptions;
        KeyframingHeuristicsOptions options =
            KeyframingHeuristicsOptions::initializeFromGFlags();
        for (const vi_map::MissionId& mission_id : missions_to_keyframe) {
          VLOG(1) << "Keyframing mission " << mission_id << '.';
          if (keyframeMapBasedOnHeuristics(
                  options, mission_id, plotter_, map.get()) !=
              common::kSuccess) {
            LOG(ERROR) << "Keyframing of mission " << mission_id << " failed.";
            return common::kUnknownError;
          }
        }
        return common::kSuccess;
      },
      "Keyframe map based on heuristics. Use the flag --map_mission or "
      "--map_mission_list to specify which missions to keyframe. If neither "
      "flag is given, all missions of the selected map will be keyframed.",
      common::Processing::Sync);

  addCommand(
      {"landmark_sparsify", "lsparsify"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (FLAGS_num_landmarks_to_keep < 0) {
          LOG(WARNING) << "Specify the number of landmarks to be kept using "
                       << "the --num_landmarks_to_keep flag.";
          return common::kStupidUserError;
        }
        sparsifyMapLandmarks(FLAGS_num_landmarks_to_keep, map.get());

        return common::kSuccess;
      },
      "Sparsify landmarks using summarization techniques. Use the flag "
      "--num_of_landmarks_to_keep to set the number of desired landmarks.",
      common::Processing::Sync);
}

}  // namespace map_sparsification_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(
    map_sparsification_plugin::MapSparsificationPlugin);
