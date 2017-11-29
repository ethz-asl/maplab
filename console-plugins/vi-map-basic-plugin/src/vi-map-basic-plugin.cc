#include "vi-map-basic-plugin/vi-map-basic-plugin.h"

#include <algorithm>
#include <iostream>  // NOLINT
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/time.h>
#include <console-common/console-plugin-base-with-plotter.h>
#include <console-common/console.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map-manager/map-manager.h>
#include <map-resources/resource-map.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/ui-utility.h>
#include <vi-map-helpers/mission-clustering-coobservation.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/semantics-manager.h>
#include <vi-map/vi-map.h>
#include <visualization/sequential-plotter.h>
#include <visualization/viwls-graph-plotter.h>

DECLARE_string(map_key);
DECLARE_string(map_folder);
DECLARE_string(map_mission);
DECLARE_bool(overwrite);
DECLARE_bool(vis_color_by_mission);
DECLARE_double(vis_scale);

DEFINE_string(target_map_key, "", "Target map key for copy and move commands.");
DEFINE_bool(
    copy_resources_to_map_folder, false,
    "Set this to true to migrate the resources to the map folder (by creating "
    "a copy of the resources) before saving the map. This flag will be reset "
    "after every use.");
DEFINE_bool(
    move_resources_to_map_folder, false,
    "Set this to true to migrate the resources to the map folder (by moving "
    "the resources) before saving the map. This flag will be reset after every "
    "use.");
DEFINE_string(
    copy_resources_to_external_folder, "",
    "Set this to true to migrate the resources to an extneral folder (by "
    "creating a copy of the resources) before saving the map. This flag will"
    " be reset after every use.");
DEFINE_string(
    move_resources_to_external_folder, "",
    "Set this to true to migrate the resources to an extneral folder (by "
    "moving the resources) before saving the map. This flag will be reset "
    "after every use.");
DEFINE_string(
    resource_folder, "", "Specifies the resource folder for certain commands.");
DEFINE_uint64(
    vertices_per_proto_file, 200u,
    "Determines the number of vertices that are stored in one proto file. "
    "NOTE: If this is set too large, the map can't be read anymore.");
DEFINE_string(
    maps_folder, ".",
    "Folder which contains one or more maps on the filesystem.");

DEFINE_double(
    spatially_distribute_missions_meters, 20,
    "Amount to shift missions when distributing them spatially.");
DEFINE_int32(
    spatially_distribute_missions_dimension, 0,
    "Dimension to shift along [x, y, z] = [0, 1, 2].");
DEFINE_bool(
    spatially_distribute_missions_around_circle, false,
    "Should sdm distribute the missions around a circle.");

namespace vi_map {

backend::SaveConfig parseSaveConfigFromGFlags() {
  backend::SaveConfig config;
  config.overwrite_existing_files = FLAGS_overwrite;

  if (FLAGS_copy_resources_to_map_folder ||
      FLAGS_move_resources_to_map_folder) {
    CHECK(
        !FLAGS_copy_resources_to_map_folder ||
        !FLAGS_move_resources_to_map_folder);
    CHECK(
        FLAGS_copy_resources_to_external_folder.empty() &&
        FLAGS_move_resources_to_external_folder.empty());
    config.migrate_resources_settings = backend::SaveConfig::
        MigrateResourcesSettings::kMigrateResourcesToMapFolder;
    config.move_resources_when_migrating = FLAGS_move_resources_to_map_folder;
  } else if (
      !FLAGS_copy_resources_to_external_folder.empty() ||
      !FLAGS_move_resources_to_external_folder.empty()) {
    CHECK(
        FLAGS_copy_resources_to_external_folder.empty() ||
        FLAGS_move_resources_to_external_folder.empty());
    CHECK(
        !FLAGS_copy_resources_to_map_folder &&
        !FLAGS_move_resources_to_map_folder);
    config.migrate_resources_settings = backend::SaveConfig::
        MigrateResourcesSettings::kMigrateResourcesToExternalFolder;
    if (FLAGS_move_resources_to_external_folder.empty()) {
      config.external_folder_for_migration =
          FLAGS_copy_resources_to_external_folder;
      config.move_resources_when_migrating = false;
    } else {
      config.external_folder_for_migration =
          FLAGS_move_resources_to_external_folder;
      config.move_resources_when_migrating = true;
    }
  }

  config.vertices_per_proto_file = FLAGS_vertices_per_proto_file;
  static constexpr size_t kMaxVerticesPerProtoFile = 300u;
  CHECK_LE(config.vertices_per_proto_file, kMaxVerticesPerProtoFile);

  return config;
}

VIMapBasicPlugin::VIMapBasicPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(CHECK_NOTNULL(console), plotter) {
  // General commands.
  addCommand(
      {"select_map", "select"}, [this]() -> int { return selectMap(); },
      "Changes the selected map. Usage: select_map --map_key=<map_key>",
      common::Processing::Sync);
  addCommand(
      {"list", "list_maps", "ls"}, [this]() -> int { return listMaps(); },
      "Lists all the maps in the storage.", common::Processing::Sync);
  addCommand(
      {"list_maps_on_file_system"},
      [this]() -> int { return listMapsOnFileSystem(); },
      "List all maps found on the file system in the given folder."
      "Usage: list_maps_on_file_system [--maps_folder=<path>]. ",
      common::Processing::Sync);

  // Accessors to map storage.
  addCommand(
      {"create_new_map"}, [this]() -> int { return createNewMap(); },
      "Creates a new, empty map. Usage: create_new_map --map_key=<new_map_key>",
      common::Processing::Sync);
  addCommand(
      {"delete_map", "delete"}, [this]() -> int { return deleteMap(); },
      "Deletes a map from the storage.", common::Processing::Sync);
  addCommand(
      {"clear_storage"}, [this]() -> int { return clearStorage(); },
      "Deletes all the maps in the storage.", common::Processing::Sync);
  addCommand(
      {"rename_map", "rename"}, [this]() -> int { return renameMap(); },
      "Renames the selected map. Usage: rename_map --map_key=<new_key_name>",
      common::Processing::Sync);
  addCommand(
      {"copy_map"}, [this]() -> int { return copyMap(); },
      "Copies the selected map. Resources won't be copied over, but instead "
      "are references "
      "via an external resource folder. Usage: copy_map "
      "--target_map_key=<key_of_new_map>",
      common::Processing::Sync);
  addCommand(
      {"merge_map"}, [this]() -> int { return mergeMaps(); },
      "Merges the map given by --map_key with the currently selected map."
      "Usage: merge_map: --map_key=<other_map>",
      common::Processing::Sync);
  addCommand(
      {"join_all_maps"}, [this]() -> int { return joinAllMaps(); },
      "Creates a new empty map and merges all loaded map into the new map. "
      "Then, all the source maps will be deleted from the storage (they will "
      "remain on the file system). Usage: join_all_maps "
      "--target_map_key=<key_of_new_combined_map>",
      common::Processing::Sync);

  // Load/save.
  addCommand(
      {"load"}, [this]() -> int { return loadMap(); },
      "Loads a map from a given path. Usage: load [--map_folder=<map_path>]. "
      "If "
      "--map_folder isn't specified, the current path (\".\") is taken.",
      common::Processing::Sync);
  addCommand(
      {"load_all"}, [this]() -> int { return loadAllMaps(); },
      "Loads all maps from a given path. Usage: load_all "
      "[--maps_folder=<map_path>]. If "
      "--maps_folder isn't specified, the current path (\".\") is taken.",
      common::Processing::Sync);
  addCommand(
      {"save"}, [this]() -> int { return saveMap(); },
      "Saves a map to the given path. Usage: save [--overwrite] "
      "[--map_folder=<path>] "
      "[--copy_resources_to_map_folder=<true/false> / "
      "--move_resources_to_map_folder=<true/false> / "
      "--copy_resources_to_external_folder=<path> / "
      "--move_resources_to_external_folder=<path>]. If --map_folder isn't "
      "specified, the "
      "map will be saved in the map folder (defined by the map metadata).",
      common::Processing::Sync);
  addCommand(
      {"save_all"}, [this]() -> int { return saveAllMaps(); },
      "Saves all maps to the given path. Usage: save_all [--overwrite] "
      "[--maps_folder=<path>] [--copy_resources_to_map_folder=<true/false> /"
      "--move_resources_to_map_folder=<true/false> / "
      "--copy_resources_to_external_folder=<path> / "
      "--move_resources_to_external_folder=<path>]. If --maps_folder isn't "
      "specified, the "
      "map will be saved in the map folder (defined by the map metadata).",
      common::Processing::Sync);

  addCommand(
      {"load_merge_map"}, [this]() -> int { return loadMergeMap(); },
      "Loads the map from the given path and merges the map with the currently "
      "selected map. If no map is selected, a new map is created with the key "
      "name as the name of the folder to load the maps from. Usage: "
      "load_merge_map --map_folder=<map_path>",
      common::Processing::Sync);
  addCommand(
      {"load_merge_all_maps"}, [this]() -> int { return loadMergeAllMaps(); },
      "Loads all the maps under the given path and merges the maps with the "
      "currently selected map. Usage: load_merge_all_maps "
      "--maps_folder=<map_path>",
      common::Processing::Sync);

  // Metadata handling.
  addCommand(
      {"get_map_folder"}, [this]() -> int { return getMapFolder(); },
      "Gets the map folder of the selected map.", common::Processing::Sync);
  addCommand(
      {"set_map_folder"}, [this]() -> int { return setMapFolder(); },
      "Sets the map folder of the current map. Usage: set_map_folder "
      "--map_folder=<folder_path>",
      common::Processing::Sync);
  addCommand(
      {"clean_up_resource_folders"},
      [this]() -> int { return cleanupResourceFolders(); },
      "Clean up of resource folders, by merging identical paths and adapting "
      "the resource references.",
      common::Processing::Sync);

  // Resource handling.
  addCommand(
      {"list_resource_folders"},
      [this]() -> int { return listResourceFolders(); },
      "Prints a list of all external resource folders for the selected map.",
      common::Processing::Sync);
  addCommand(
      {"use_map_resource_folder"},
      [this]() -> int { return useMapResourceFolder(); },
      "Sets the selected map to use the map folder for resource storage. This "
      "doesn't "
      "affect existing resources, only new resources will be stored in this "
      "folder.",
      common::Processing::Sync);
  addCommand(
      {"use_external_resource_folder"},
      [this]() -> int { return useExternalResourceFolder(); },
      "Sets the selected map to use an external folder for resource storage. "
      "This doesn't "
      "affect existing resources, only new resources will be stored in this "
      "folder. Usage: "
      "use_external_folder --resource_folder=<path>",
      common::Processing::Sync);
  addCommand(
      {"resource_statistics", "res_stats"},
      [this]() -> int { return printResourceStatistics(); },
      "Prints the resource statistics for the selected map.",
      common::Processing::Sync);
  addCommand(
      {"resource_cache_statistics", "res_cache_stats"},
      [this]() -> int { return printResourceCacheStatistics(); },
      "Prints resource cache statistics for the selected map.",
      common::Processing::Sync);

  addCommand(
      {"check_map_consistency"},
      [this]() -> int { return checkMapConsistency(); },
      "Checks if the map is consistent.", common::Processing::Sync);

  // Map statistics, visualization.
  addCommand(
      {"ms", "map_stats"}, [this]() -> int { return mapStatistics(); },
      "Print map statistics.", common::Processing::Sync);
  addCommand(
      {"mcs", "mission_coobservability_stats"},
      [this]() -> int { return printMissionCoobservabilityStatistics(); },
      "Print mission co-observability statistics.", common::Processing::Sync);
  addCommand(
      {"print_baseframes"},
      [this]() -> int { return printBaseframeTransformations(); },
      "Print baseframe transfromations for all missions.",
      common::Processing::Sync);
  addCommand(
      {"print_camera_calibrations"},
      [this]() -> int { return printCameraCalibrations(); },
      "Print the camera calibrations of all missions.",
      common::Processing::Sync);
  addCommand(
      {"spatially_distribute_missions"},
      [this]() -> int { return spatiallyDistributeMissions(); },
      "Spatially distributes missions with unknown baseframes.",
      common::Processing::Sync);
  addCommand(
      {"v", "visualize"}, [this]() -> int { return visualizeMap(); },
      "Visualizes the selected map.", common::Processing::Sync);
  addCommand(
      {"vs", "visualize_sequentially"},
      [this]() -> int { return visualizeMapSequentially(); },
      "Visualizes the map sequentially.", common::Processing::Sync);

  // Mission operations.
  addCommand(
      {"rm", "remove_mission"}, [this]() { return removeMission(); },
      "Removes mission including all vertices, edges and landmarks.",
      common::Processing::Sync);
  addCommand(
      {"rmi", "remove_mission_interactive"},
      [this]() { return removeMissionInteractive(); },
      "Cycle through missions and ask which to remove. The current mission "
      "will be visualized in RViz a different color.",
      common::Processing::Sync);

  // Backwards compatibility commands.
  addCommand(
      {"convert_map_to_new_format"},
      [this]() -> int { return convertMapToNewFormat(); },
      "Loads a map using deprecated deserialization and then stores it again "
      "using the latest serialization.",
      common::Processing::Sync);
}

int VIMapBasicPlugin::selectMap() {
  const vi_map::VIMapManager map_manager;
  if (FLAGS_map_key.empty()) {
    std::vector<std::string> all_map_keys;
    map_manager.getAllMapKeys(&all_map_keys);

    if (all_map_keys.empty()) {
      LOG(ERROR) << "No map key specified, please set flag \"map_key\" when "
                    "calling this command.";
      return common::kStupidUserError;
    }

    const std::string& map_key_to_select = all_map_keys.front();
    LOG(WARNING) << "No map key specified, selecting the first map, which is \""
                 << map_key_to_select << "\".";
    console_->setSelectedMapKey(map_key_to_select);
  }

  if (!map_manager.hasMap(FLAGS_map_key)) {
    LOG(ERROR) << "Key \"" << FLAGS_map_key << "\" not found.";
    return common::kUnknownError;
  }
  console_->setSelectedMapKey(FLAGS_map_key);

  return common::kSuccess;
}

int VIMapBasicPlugin::listMaps() {
  const vi_map::VIMapManager map_manager;
  std::vector<std::string> all_map_keys;
  map_manager.getAllMapKeys(&all_map_keys);

  std::stringstream output;
  if (!all_map_keys.empty()) {
    output << "VIMaps saved in the storage (" << all_map_keys.size()
           << " total):";
    std::sort(all_map_keys.begin(), all_map_keys.end());
    const std::string& selected_map_key = console_->getSelectedMapKey();
    for (const std::string& key : all_map_keys) {
      output << "\n  ";
      if (key == selected_map_key) {
        // Mark selected map.
        output << "* "
               << common::colorText(key, common::ForegroundColors::kGreen);
      } else {
        output << "  " << key;
      }
      output << " -> ";
      std::string map_folder;
      vi_map::VIMapManager::MapReadAccess map =
          map_manager.getMapReadAccess(key);
      if (map->hasMapFolder()) {
        map_manager.getMapFolder(key, &map_folder);
        output << map_folder;
      } else {
        output << common::colorText(
            "No map folder defined!", common::ForegroundColors::kRed);
      }
    }
  } else {
    output << "No maps saved in the storage.";
  }
  std::cout << output.str() << std::endl;
  return common::kSuccess;
}

int VIMapBasicPlugin::createNewMap() {
  if (FLAGS_map_key.empty()) {
    LOG(ERROR) << "No map key specified, please set flag \"map_key\" when "
                  "calling this command.";
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  if (map_manager.hasMap(FLAGS_map_key)) {
    LOG(ERROR) << "Key \"" << FLAGS_map_key << "\" already exists.";
    return common::kUnknownError;
  }

  vi_map::VIMap::UniquePtr vi_map = aligned_unique<vi_map::VIMap>();
  map_manager.addMap(FLAGS_map_key, vi_map);
  console_->setSelectedMapKey(FLAGS_map_key);
  console_->addMapKeyToAutoCompletion(FLAGS_map_key);
  return common::kSuccess;
}

int VIMapBasicPlugin::deleteMap() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  if (!map_manager.hasMap(selected_map_key)) {
    LOG(ERROR) << "Key \"" << selected_map_key << "\" not found.";
    return common::kUnknownError;
  }

  if (common::askUserForYesNoOverConsole(
          "Do you really want to delete the map \"" + selected_map_key +
          "\" from the storage?")) {
    map_manager.deleteMap(selected_map_key);
    console_->removeMapKeyFromAutoCompletion(selected_map_key);
    const std::string kNoMapSelected = "";
    console_->setSelectedMapKey(kNoMapSelected);
  }
  return common::kSuccess;
}

int VIMapBasicPlugin::clearStorage() {
  if (common::askUserForYesNoOverConsole(
          "Do you really want to delete all of the maps in the storage?")) {
    vi_map::VIMapManager map_manager;
    std::unordered_set<std::string> all_map_keys;
    map_manager.getAllMapKeys(&all_map_keys);
    for (const std::string& key : all_map_keys) {
      map_manager.deleteMap(key);
      console_->removeMapKeyFromAutoCompletion(key);
    }
    const std::string kNoMapSelected = "";
    console_->setSelectedMapKey(kNoMapSelected);
  }
  return common::kSuccess;
}

int VIMapBasicPlugin::renameMap() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  if (FLAGS_map_key.empty()) {
    LOG(ERROR) << "No map key specified, please set flag \"map_key\" when "
                  "calling this command.";
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  if (!map_manager.hasMap(selected_map_key)) {
    LOG(ERROR) << "Key \"" << selected_map_key << "\" not found.";
    return common::kUnknownError;
  }
  if (map_manager.hasMap(FLAGS_map_key)) {
    LOG(ERROR) << "Key \"" << FLAGS_map_key << "\" already exists.";
    return common::kUnknownError;
  }

  map_manager.renameMap(selected_map_key, FLAGS_map_key);
  console_->removeMapKeyFromAutoCompletion(selected_map_key);
  console_->addMapKeyToAutoCompletion(FLAGS_map_key);
  console_->setSelectedMapKey(FLAGS_map_key);
  return common::kSuccess;
}

int VIMapBasicPlugin::copyMap() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  if (FLAGS_target_map_key.empty()) {
    LOG(ERROR) << "No target map key specified, please set flag "
                  "\"target_map_key\" when calling "
               << "this command.";
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  if (!map_manager.hasMap(selected_map_key)) {
    LOG(ERROR) << "Key \"" << selected_map_key << "\" not found.";
    return common::kUnknownError;
  }
  if (map_manager.hasMap(FLAGS_target_map_key)) {
    LOG(ERROR) << "Key \"" << FLAGS_target_map_key << "\" already exists.";
    return common::kUnknownError;
  }

  map_manager.copyMap(selected_map_key, FLAGS_target_map_key);
  console_->addMapKeyToAutoCompletion(FLAGS_target_map_key);

  return common::kSuccess;
}

int VIMapBasicPlugin::mergeMaps() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  if (FLAGS_map_key.empty()) {
    LOG(ERROR) << "No map key specified, please set flag \"map_key\" when "
                  "calling this command.";
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  if (!map_manager.hasMap(selected_map_key)) {
    LOG(ERROR) << "Key \"" << selected_map_key << "\" not found.";
    return common::kUnknownError;
  }
  if (!map_manager.hasMap(FLAGS_map_key)) {
    LOG(ERROR) << "Key \"" << FLAGS_map_key << "\" not found.";
    return common::kUnknownError;
  }
  if (map_manager.hasMap(FLAGS_target_map_key)) {
    LOG(ERROR) << "Key \"" << FLAGS_target_map_key << "\" already exists.";
    return common::kUnknownError;
  }

  LOG(INFO) << "Merging all missions from map \"" << FLAGS_map_key
            << "\" into map \"" << selected_map_key << "\".";
  map_manager.mergeMaps(selected_map_key, FLAGS_map_key);

  return common::kSuccess;
}

int VIMapBasicPlugin::joinAllMaps() {
  vi_map::VIMapManager map_manager;
  if (!map_manager.isKeyValid(FLAGS_target_map_key)) {
    LOG(ERROR) << "The chosen target map key \"" << FLAGS_target_map_key
               << "\" isn't valid.";
    return common::kStupidUserError;
  }

  std::vector<std::string> all_current_map_keys;
  map_manager.getAllMapKeys(&all_current_map_keys);

  if (all_current_map_keys.size() <= 1u) {
    LOG(ERROR) << "There is no map or only one map loaded in the storage. "
               << "Joining all maps is aborted as there is nothing to do.";
    return common::kStupidUserError;
  }

  // Use the first map as a base.
  const std::string& base_map_key = all_current_map_keys.front();
  VLOG(1) << "Selected \"" << base_map_key << "\" as base map for merging.";

  vi_map::VIMapManager::MapWriteAccess base_map =
      map_manager.getMapWriteAccess(base_map_key);

  // Merge all other maps into base maps and delete them.
  for (std::vector<std::string>::const_iterator it_map_keys =
           all_current_map_keys.cbegin() + 1;
       it_map_keys != all_current_map_keys.cend(); ++it_map_keys) {
    {
      vi_map::VIMapManager::MapReadAccess map_to_be_merged =
          map_manager.getMapReadAccess(*it_map_keys);
      base_map->mergeAllMissionsFromMap(*map_to_be_merged);
    }
    map_manager.deleteMap(*it_map_keys);
    console_->removeMapKeyFromAutoCompletion(*it_map_keys);
    CHECK(!map_manager.hasMap(*it_map_keys));
    LOG(INFO) << "Merging \"" << *it_map_keys << "\" into \"" << base_map_key
              << "\".";
  }

  // Rename current map key to target map key.
  map_manager.renameMap(base_map_key, FLAGS_target_map_key);

  std::vector<std::string> map_keys_after_join;
  map_manager.getAllMapKeys(&map_keys_after_join);
  CHECK_EQ(1u, map_keys_after_join.size());
  CHECK_EQ(FLAGS_target_map_key, map_keys_after_join[0u]);

  console_->setSelectedMapKey(FLAGS_target_map_key);
  console_->removeMapKeyFromAutoCompletion(base_map_key);
  console_->addMapKeyToAutoCompletion(FLAGS_target_map_key);
  LOG(INFO) << "Merged " << all_current_map_keys.size()
            << " maps together into the target map \"" << FLAGS_target_map_key
            << "\".";

  return common::kSuccess;
}

int VIMapBasicPlugin::listMapsOnFileSystem() {
  if (FLAGS_maps_folder.empty()) {
    LOG(ERROR) << "No path specified, please set the flag \"maps_folder\".";
    return common::kStupidUserError;
  }

  std::vector<std::string> map_list;
  const std::string maps_folder = FLAGS_maps_folder;
  vi_map::VIMapManager map_manager;
  map_manager.listAllMapsInFolder(maps_folder, &map_list);
  std::vector<std::string> map_keys;
  map_manager.getDefaultMapKeys(map_list, &map_keys);

  if (map_keys.empty()) {
    LOG(INFO) << "Found zero maps in folder " << maps_folder << '.';
  } else {
    LOG(INFO) << "Found the following (" << map_keys.size()
              << ") map(s) in folder " << maps_folder << ':';
    for (const std::string& map_key : map_keys) {
      LOG(INFO) << "\t - Default key: "
                << common::formatText(map_key, common::FormatOptions::kBold);
    }
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::loadMap() {
  if (FLAGS_map_folder.empty()) {
    LOG(ERROR) << "No path specified, please set the flag \"map_folder\".";
    return common::kStupidUserError;
  }

  std::string map_folder = FLAGS_map_folder;
  if (map_folder.empty()) {
    map_folder = ".";
  }

  vi_map::VIMapManager map_manager;
  const std::string& map_key = FLAGS_map_key;
  if (map_key.empty()) {
    // Automatic key.
    std::string new_key_name;
    if (!map_manager.loadMapFromFolder(map_folder, &new_key_name)) {
      return common::kUnknownError;
    }
    CHECK(!new_key_name.empty());
    console_->setSelectedMapKey(new_key_name);
    console_->addMapKeyToAutoCompletion(new_key_name);
  } else {
    if (!map_manager.loadMapFromFolder(map_folder, map_key)) {
      return common::kUnknownError;
    }
    console_->setSelectedMapKey(map_key);
    console_->addMapKeyToAutoCompletion(map_key);
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::loadMergeMap() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  if (FLAGS_map_folder.empty()) {
    LOG(ERROR) << "No path specified, please set the flag \"map_folder\".";
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  const std::string kKeyToLoadIn =
      std::string("temporary_map_") +
      std::to_string(aslam::time::nanoSecondsSinceEpoch());
  map_manager.loadMapFromFolder(FLAGS_map_folder, kKeyToLoadIn);
  map_manager.mergeMaps(selected_map_key, kKeyToLoadIn);
  map_manager.deleteMap(kKeyToLoadIn);

  return common::kSuccess;
}

int VIMapBasicPlugin::loadMergeAllMaps() {
  std::string selected_map_key = console_->getSelectedMapKey();
  std::string maps_folder = FLAGS_maps_folder;
  if (maps_folder.empty()) {
    LOG(ERROR) << "Invalid maps folder. Specify a valid maps folder with "
               << "--maps_folder";
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;

  // Map key of the merge map is determined by:
  // 1) flag target_map_key
  // 2) selected map key
  // 3) flag maps_folder
  std::string target_map_key = FLAGS_target_map_key;
  if (target_map_key.empty()) {
    if (!selected_map_key.empty()) {
      target_map_key = selected_map_key;
    } else {
      // Use folder name to infer target map key.
      std::string folder_without_key;
      common::splitPathAndFilename(
          common::getRealPath(maps_folder), &folder_without_key,
          &target_map_key);
    }
  }

  if (selected_map_key.empty()) {
    // Ensure chosen target map key is available.
    if (!map_manager.isKeyValid(target_map_key)) {
      LOG(ERROR) << "The target map key \"" << target_map_key
                 << "\" is not a valid key.";
      return common::kStupidUserError;
    }
    if (map_manager.hasMap(target_map_key)) {
      LOG(ERROR)
          << "The target map key \"" << target_map_key
          << "\" cannot be used because a map with this key already exists.";
      return common::kStupidUserError;
    }
  }

  std::unordered_set<std::string> loaded_keys;
  if (!map_manager.loadAllMapsFromFolder(maps_folder, &loaded_keys)) {
    LOG(ERROR) << "Loading maps from \"" << maps_folder << "\" failed.";
    return common::kUnknownError;
  }
  CHECK(!loaded_keys.empty()) << "No map loaded from \"" << maps_folder
                              << "\".";

  if (selected_map_key.empty() || !map_manager.hasMap(selected_map_key)) {
    // No map selected -- use the first newly loaded map as a base.
    selected_map_key = *loaded_keys.begin();
    loaded_keys.erase(selected_map_key);
  } else {
    console_->removeMapKeyFromAutoCompletion(selected_map_key);
  }

  if (!selected_map_key.empty() && selected_map_key != target_map_key) {
    map_manager.renameMap(selected_map_key, target_map_key);
  }
  selected_map_key = target_map_key;
  console_->setSelectedMapKey(selected_map_key);
  console_->addMapKeyToAutoCompletion(selected_map_key);

  VLOG(1) << "Using \"" << selected_map_key
          << "\" as the base for future merge operations.";

  vi_map::VIMapManager::MapWriteAccess base_map =
      map_manager.getMapWriteAccess(selected_map_key);
  for (const std::string& loaded_key : loaded_keys) {
    {
      vi_map::VIMapManager::MapReadAccess map_to_be_merged =
          map_manager.getMapReadAccess(loaded_key);
      base_map->mergeAllMissionsFromMap(*map_to_be_merged);
    }
    map_manager.deleteMap(loaded_key);
    VLOG(1) << "Merging \"" << loaded_key << "\" into \"" << selected_map_key
            << "\".";
  }

  LOG(INFO) << "Merged " << loaded_keys.size() << " maps into \""
            << selected_map_key << "\".";

  return common::kSuccess;
}

int VIMapBasicPlugin::loadAllMaps() {
  std::string maps_folder = FLAGS_maps_folder;
  if (maps_folder.empty()) {
    LOG(ERROR) << "Invalid maps folder. Specify a valid maps folder with "
               << "--maps_folder";
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  std::unordered_set<std::string> new_keys;
  if (!map_manager.loadAllMapsFromFolder(maps_folder, &new_keys)) {
    return common::kUnknownError;
  }

  // Add new keys to auto completion.
  for (const std::string& key : new_keys) {
    console_->addMapKeyToAutoCompletion(key);
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::saveMap() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  std::string map_folder = FLAGS_map_folder;

  if (map_folder.empty()) {
    if (!map_manager.getMapReadAccess(selected_map_key)->hasMapFolder()) {
      LOG(ERROR) << "Selected map doesn't have a map folder. Please use "
                    "\"set_map_folder "
                 << "--map_folder=<path>\" or \"save --map_folder=<path>\" to "
                    "set the map folder.";
      return common::kStupidUserError;
    }
    if (!map_manager.saveMapToMapFolder(
            selected_map_key, parseSaveConfigFromGFlags())) {
      return common::kUnknownError;
    }
  } else {
    if (!map_manager.saveMapToFolder(
            selected_map_key, map_folder, parseSaveConfigFromGFlags())) {
      return common::kUnknownError;
    }
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::saveAllMaps() {
  std::string maps_folder = FLAGS_maps_folder;

  vi_map::VIMapManager map_manager;
  if (maps_folder.empty()) {
    if (!map_manager.saveAllMapsToMapFolder(parseSaveConfigFromGFlags())) {
      return common::kUnknownError;
    }
  } else {
    if (!map_manager.saveAllMapsToFolder(
            maps_folder, parseSaveConfigFromGFlags())) {
      return common::kUnknownError;
    }
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::getMapFolder() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);
  if (!map->hasMapFolder()) {
    LOG(ERROR) << "Selected map doesn't have a map folder. Please use "
                  "\"set_map_folder "
               << "--map_folder=<path>\" to set the map folder.";
    return common::kStupidUserError;
  }
  std::string map_folder;
  map->getMapFolder(&map_folder);
  std::cout << "The map folder for the selected map is "
            << common::formatText(map_folder, common::FormatOptions::kBold)
            << "." << std::endl;
  // This is necessary because getRealPath is not defined for folders that don't
  // exist.
  if (common::pathExists(map_folder)) {
    std::cout << "As an absolute path: " << common::formatText(
                                                common::getRealPath(map_folder),
                                                common::FormatOptions::kBold)
              << "." << std::endl;
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::cleanupResourceFolders() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  std::cout << "Resource folders before: " << std::endl;
  listResourceFolders();

  vi_map::VIMapManager map_manager;
  map_manager.getMapWriteAccess(selected_map_key)->cleanupResourceFolders();

  std::cout << "Resource folders after: " << std::endl;
  listResourceFolders();

  return common::kSuccess;
}

int VIMapBasicPlugin::setMapFolder() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  if (FLAGS_map_folder.empty()) {
    LOG(ERROR) << "No map folder given, please set the --map_folder flag";
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  map_manager.getMapWriteAccess(selected_map_key)
      ->setMapFolder(FLAGS_map_folder);

  std::cout << "Changed the map folder of the selected map to "
            << common::formatText(
                   FLAGS_map_folder, common::FormatOptions::kBold)
            << "." << std::endl;
  return common::kSuccess;
}

int VIMapBasicPlugin::listResourceFolders() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);
  if (!map->hasMapFolder()) {
    LOG(ERROR) << "Selected map doesn't have a map folder. Please use "
                  "\"set_map_folder "
               << "--map_folder=<path>\" to set the map folder.";
    return common::kStupidUserError;
  }

  const backend::ResourceMap::MetaData metadata = map->getMetaDataCopy();
  std::string map_folder;
  map->getMapFolder(&map_folder);

  std::cout << "The map folder is set to:\n  ";
  if (metadata.resource_folder_in_use ==
      backend::ResourceMap::kMapResourceFolder) {
    std::cout << "* "
              << common::colorText(map_folder, common::ForegroundColors::kGreen)
              << " (using map folder for resources)";
  } else {
    std::cout << "  " << map_folder;
  }
  std::cout << "\n" << std::endl;

  if (!metadata.external_resource_folders.empty()) {
    std::cout << "Defined external folders ("
              << metadata.external_resource_folders.size() << " in total):\n";

    backend::ResourceMap::ResourceFolderIndex folder_index = 0;
    for (const std::string& external_folder :
         metadata.external_resource_folders) {
      std::cout << "  ";
      if (folder_index == metadata.resource_folder_in_use) {
        std::cout << "* "
                  << common::colorText(
                         std::string("(") + std::to_string(folder_index) +
                             ") " + external_folder,
                         common::ForegroundColors::kGreen)
                  << "\n";
      } else {
        std::cout << "  (" << folder_index << ") " << external_folder << "\n";
      }
      ++folder_index;
    }
  } else {
    std::cout << "No external resource folders are defined." << std::endl;
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::useMapResourceFolder() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  if (!map_manager.getMapReadAccess(selected_map_key)->hasMapFolder()) {
    LOG(ERROR) << "Selected map doesn't have a map folder. Please use "
                  "\"set_map_folder "
               << "--map_folder=<path>\" to set the map folder.";
    return common::kStupidUserError;
  }
  map_manager.getMapWriteAccess(selected_map_key)->useMapResourceFolder();
  return common::kSuccess;
}

int VIMapBasicPlugin::useExternalResourceFolder() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  if (FLAGS_resource_folder.empty()) {
    LOG(ERROR) << "No folder specified, please set the --resource_folder flag.";
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  map_manager.getMapWriteAccess(selected_map_key)
      ->useExternalResourceFolder(FLAGS_resource_folder);
  return common::kSuccess;
}

int VIMapBasicPlugin::printResourceStatistics() {
  const std::string& selected_map_key = console_->getSelectedMapKey();
  if (selected_map_key.empty()) {
    LOG(ERROR) << "No map is selected, please use \"select_map\" command to "
                  "choose one first.";
    return common::kStupidUserError;
  }

  const vi_map::VIMapManager map_manager;
  std::cout << map_manager.getMapReadAccess(selected_map_key)
                   ->printResourceStatistics()
            << std::endl;

  return common::kSuccess;
}

int VIMapBasicPlugin::printResourceCacheStatistics() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  const vi_map::VIMapManager map_manager;
  std::cout
      << map_manager.getMapReadAccess(selected_map_key)->printCacheStatistics()
      << std::endl;

  return common::kSuccess;
}

int VIMapBasicPlugin::checkMapConsistency() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);
  vi_map::checkMapConsistency(*map);

  return common::kSuccess;
}

int VIMapBasicPlugin::mapStatistics() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);
  std::cout << map->printMapStatistics() << std::endl << std::endl;
  if (map->numMissions() > 1u) {
    std::cout << map->printMapAccumulatedStatistics() << std::endl;
  }

  return common::kSuccess;
}

int VIMapBasicPlugin::printMissionCoobservabilityStatistics() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  vi_map::MissionIdList all_missions;
  map->getAllMissionIds(&all_missions);

  std::vector<vi_map::MissionIdSet> mission_clusters =
      vi_map_helpers::clusterMissionByLandmarkCoobservations(
          *map, vi_map::MissionIdSet(all_missions.begin(), all_missions.end()));

  std::cout << "Cluster(s) of missions with co-observed landmarks: "
            << std::endl;

  size_t idx = 0u;
  for (const vi_map::MissionIdSet& mission_cluster : mission_clusters) {
    std::cout << "cluster " << idx++ << ":" << std::endl;
    std::cout << "  " << printIdContainer(mission_cluster) << std::endl;
  }
  return common::kSuccess;
}

int VIMapBasicPlugin::printCameraCalibrations() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  const vi_map::VIMapManager map_manager;
  const vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  vi_map::MissionIdList all_missions;
  map->getAllMissionIds(&all_missions);
  if (all_missions.empty()) {
    LOG(ERROR) << "The selected map does not contain any missions.";
    return common::kUnknownError;
  }

  std::stringstream output;
  output << "Camera calibrations: \n\n";
  for (const vi_map::MissionId& mission_id : all_missions) {
    const aslam::NCamera& ncam =
        map->getSensorManager().getNCameraForMission(mission_id);

    output << "Mission: " << mission_id << "\n";
    for (const aslam::Camera::Ptr& cam : ncam.getCameraVector()) {
      CHECK(cam);
      cam->printParameters(output, "");

      output << "T_C_B:\n";
      output << ncam.get_T_C_B(cam->getId()) << "\n";
    }
    output << "\n";
  }
  std::cout << output.str();
  return common::kSuccess;
}

int VIMapBasicPlugin::printBaseframeTransformations() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  const vi_map::VIMapManager map_manager;
  const vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  vi_map::MissionIdList all_missions;
  map->getAllMissionIds(&all_missions);
  if (all_missions.empty()) {
    LOG(ERROR) << "The selected map does not contain any missions.";
    return common::kUnknownError;
  }

  // Add a fixed number of spaces to the beginning of each line of a
  // multi-line string.
  auto indent_string = [](
      const std::string& input, size_t indent) -> std::string {
    if (input.empty()) {
      return input;
    }
    const std::string intend_string(indent, ' ');

    std::istringstream input_ss(input);
    std::string line, output;
    while (std::getline(input_ss, line)) {
      output += (intend_string + line + '\n');
    }
    return output;
  };

  std::stringstream stats_text;
  stats_text << "Baseframe transformations: " << '\n';
  size_t mission_number = 0u;
  for (const vi_map::MissionId& mission_id : all_missions) {
    stats_text << "Mission " << mission_number << " (" << mission_id.hexString()
               << "):" << '\n';
    const vi_map::MissionBaseFrame& baseframe =
        map->getMissionBaseFrame(map->getMission(mission_id).getBaseFrameId());
    stats_text << "  status: "
               << (baseframe.is_T_G_M_known() ? "known" : "unknown")
               << std::endl;
    stats_text << "  T_G_M: " << std::endl;

    std::stringstream T_G_M;
    T_G_M << baseframe.get_T_G_M();
    constexpr size_t kNumSpacesForIndentation = 4u;
    stats_text << indent_string(T_G_M.str(), kNumSpacesForIndentation) << '\n';
    ++mission_number;
  }

  std::cout << stats_text.str();
  return common::kSuccess;
}

int VIMapBasicPlugin::removeMission() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  // Select mission.
  vi_map::MissionId mission_id;
  if (!map->hexStringToMissionIdIfValid(FLAGS_map_mission, &mission_id)) {
    LOG(ERROR) << "The given mission id \"" << FLAGS_map_mission
               << "\" is not valid.";
    return common::kStupidUserError;
  }

  const bool kRemoveBaseframe = true;
  map->removeMission(mission_id, kRemoveBaseframe);
  return common::kSuccess;
}

int VIMapBasicPlugin::removeMissionInteractive() {
  if (plotter_ == nullptr) {
    LOG(ERROR) << "Please enable the visualization.";
    return common::kStupidUserError;
  }

  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  // Cycle through mission, visualize the current one in a different color and
  // ask the user to remove it.
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  size_t mission_idx = 1u;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    // Visualize only this mission in a different color.
    constexpr bool kPlotBaseframes = true;
    constexpr bool kPlotVertices = true;
    constexpr bool kPlotEdges = true;
    constexpr bool kPlotLandmarks = true;
    FLAGS_vis_color_by_mission = false;
    plotter_->visualizeMap(
        *map, kPlotBaseframes, kPlotVertices, kPlotEdges, kPlotLandmarks);

    const double initial_vis_scale = FLAGS_vis_scale;
    FLAGS_vis_color_by_mission = true;
    FLAGS_vis_scale = 3.0 * initial_vis_scale;
    plotter_->visualizeMissions(
        *map, {mission_id}, kPlotBaseframes, kPlotVertices, kPlotEdges,
        kPlotLandmarks);

    // Restore the initial visualization scale for the rest of the missions.
    FLAGS_vis_scale = initial_vis_scale;

    FLAGS_vis_color_by_mission = false;
    plotter_->visualizeMap(*map, false, false, false, kPlotLandmarks);

    bool got_meaningful_answer = false;
    do {
      // Ask the user what to do with this mission.
      std::cout << "(" << mission_idx << "/" << mission_ids.size() << ")"
                << " Would you like to delete the mission " << mission_id
                << "? [y/n/q]: ";

      const char answer = getchar();
      std::cout << std::endl;
      if (answer == 'y' || answer == 'Y') {
        const bool kRemoveBaseframe = true;
        map->removeMission(mission_id, kRemoveBaseframe);
        got_meaningful_answer = true;
      } else if (answer == 'n' || answer == 'N') {
        got_meaningful_answer = true;
      } else if (answer == 'q' || answer == 'Q') {
        return common::kSuccess;
      }
    } while (!got_meaningful_answer);
    ++mission_idx;
  }
  return common::kSuccess;
}

int VIMapBasicPlugin::spatiallyDistributeMissions() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (map->numMissions() == 0u) {
    LOG(ERROR) << "No missions in database.";
    return common::kUnknownError;
  }
  typedef std::pair<double, vi_map::MissionId> LengthAndMission;
  std::vector<LengthAndMission> lengths_and_missions;

  vi_map::MissionIdList all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);

  vi_map::MissionIdList missions_with_unknown_baseframe;
  vi_map::MissionIdList missions_with_known_baseframe;
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    const vi_map::VIMission& mission = map->getMission(mission_id);
    vi_map::MissionBaseFrame& baseframe =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    if (baseframe.is_T_G_M_known()) {
      missions_with_known_baseframe.emplace_back(mission_id);
    } else {
      missions_with_unknown_baseframe.emplace_back(mission_id);
    }
  }

  if (missions_with_unknown_baseframe.empty()) {
    LOG(WARNING) << "All mission baseframes of the map are known. No missions "
                 << "to spatially distribute.";
    return common::kUnknownError;
  }

  // Compute the average vertex position of the vertices from missions with
  // known base-frames.
  Eigen::Vector3d mean_vertex_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_baseframe_position = Eigen::Vector3d::Zero();
  size_t num_samples = 0u;
  for (const vi_map::MissionId& mission_id : missions_with_known_baseframe) {
    pose_graph::VertexIdList mission_vertices;
    map->getAllVertexIdsInMission(mission_id, &mission_vertices);
    for (const pose_graph::VertexId& vertex_id : mission_vertices) {
      mean_vertex_position += map->getVertex_G_p_I(vertex_id);
      ++num_samples;
    }
    mean_baseframe_position +=
        map->getMissionBaseFrameForMission(mission_id).get_p_G_M();
  }
  if (num_samples > 0u) {
    mean_vertex_position /= num_samples;
    mean_baseframe_position /= num_samples;
  }

  if (FLAGS_spatially_distribute_missions_around_circle) {
    VLOG(1) << "Will distribute the missions around a circle.";
    VLOG(2) << "Mean position of known mission vertices "
            << mean_vertex_position.transpose();
  } else {
    VLOG(1) << "Will distribute missions along one dimension.";
    VLOG(2) << "Mean position of known mission baseframes "
            << mean_baseframe_position.transpose();
  }

  CHECK_GE(FLAGS_spatially_distribute_missions_dimension, 0);
  CHECK_LE(FLAGS_spatially_distribute_missions_dimension, 2);

  const double radians_per_mission =
      2. * M_PI / missions_with_unknown_baseframe.size();
  for (size_t i = 0u; i < missions_with_unknown_baseframe.size(); ++i) {
    const vi_map::MissionId& mission_id = missions_with_unknown_baseframe[i];
    const vi_map::VIMission& mission = map->getMission(mission_id);
    vi_map::MissionBaseFrame& baseframe =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    Eigen::Vector3d p_GM = baseframe.get_p_G_M();

    if (FLAGS_spatially_distribute_missions_around_circle) {
      p_GM(0) = mean_vertex_position(0) +
                FLAGS_spatially_distribute_missions_meters *
                    cos(i * radians_per_mission);
      p_GM(1) = mean_vertex_position(1) +
                FLAGS_spatially_distribute_missions_meters *
                    sin(i * radians_per_mission);
    } else {
      p_GM(FLAGS_spatially_distribute_missions_dimension) =
          mean_baseframe_position(
              FLAGS_spatially_distribute_missions_dimension) +
          FLAGS_spatially_distribute_missions_meters * (i + 1);
    }
    baseframe.set_p_G_M(p_GM);
  }
  return common::kSuccess;
}

int VIMapBasicPlugin::visualizeMap() {
  if (!plotter_) {
    LOG(ERROR) << "The plotter is not initialized. Visualization is not "
               << "possible in a ros-free environment.";
    return common::kStupidUserError;
  }

  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);
  plotter_->visualizeMap(*map);

  return common::kSuccess;
}

int VIMapBasicPlugin::visualizeMapSequentially() {
  if (!plotter_) {
    LOG(ERROR) << "The plotter is not initialized. Visualization is not "
                  "possible in a ros-free "
               << "environment.";
    return common::kStupidUserError;
  }

  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  const vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  // Select missions.
  std::unordered_set<vi_map::MissionId> mission_ids;
  vi_map::MissionId mission_id;
  if (map->hexStringToMissionIdIfValid(FLAGS_map_mission, &mission_id)) {
    mission_ids.emplace(mission_id);
  } else {
    map->getAllMissionIds(&mission_ids);
  }

  visualization::SequentialPlotter seq_plotter(plotter_);
  seq_plotter.publishMissionsSequentially(*map, mission_ids);
  return common::kSuccess;
}

int VIMapBasicPlugin::convertMapToNewFormat() {
  if (FLAGS_map_folder.empty()) {
    LOG(ERROR) << "No path specified, please set the flag \"map_folder\".";
    return common::kStupidUserError;
  }

  vi_map::VIMap::UniquePtr map = aligned_unique<vi_map::VIMap>();
  if (!map->loadFromFolderDeprecated(FLAGS_map_folder)) {
    LOG(ERROR) << "Unable to load the map using deprecated deserialization.";
    return common::kUnknownError;
  }

  if (!(FLAGS_copy_resources_to_map_folder ||
        FLAGS_move_resources_to_map_folder)) {
    LOG(WARNING)
        << "\n"
        << "############################################################\n"
        << "WARNING: The resources of the converted map will be linked \n"
        << "to the map resource folder of the old map format unless you \n"
        << "use --copy_resources_to_map_folder or \n"
        << "--move_resources_to_map_folder to copy/move all the map \n"
        << "resources to the map folder of the new map.\n"
        << "############################################################\n";
  }
  const std::string map_folder_out = FLAGS_map_folder + "_converted";
  return map->saveToFolder(map_folder_out, parseSaveConfigFromGFlags());
}

}  // namespace vi_map

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(vi_map::VIMapBasicPlugin);
