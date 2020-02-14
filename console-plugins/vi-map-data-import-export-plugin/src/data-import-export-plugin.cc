#include "vi-map-data-import-export-plugin/data-import-export-plugin.h"

#include <console-common/console.h>
#include <csv-export/csv-export.h>
#include <map-manager/map-manager.h>
#include <maplab-common/file-logger.h>
#include <vi-map-data-import-export/export-ncamera-calibration.h>
#include <vi-map-data-import-export/export-vertex-data.h>

DECLARE_string(map_mission);
DECLARE_bool(csv_export_imu_data);
DECLARE_bool(csv_export_tracks_and_keypoints);
DECLARE_bool(csv_export_descriptors);
DECLARE_bool(csv_export_landmarks);
DECLARE_bool(csv_export_observations);

DEFINE_string(csv_export_path, "", "Path to save the map in CSV format into.");
DEFINE_string(
    mission_info_export_path, "", "Export path of the mission info yaml.");
DEFINE_string(
    ncamera_calibration_export_folder, "",
    "Folder to export the ncamera calibration into.");
DEFINE_string(pose_export_file, "", "File to export poses to.");
DEFINE_string(
    pose_export_reference_sensor_id, "",
    "Sensor id defining in what coordinate frame to express the vertex poses.");

namespace data_import_export {

DataImportExportPlugin::DataImportExportPlugin(common::Console* console)
    : common::ConsolePluginBase(console) {
  auto csv_export = [this]() -> int {
    std::string selected_map_key;
    if (!getSelectedMapKeyIfSet(&selected_map_key)) {
      return common::kStupidUserError;
    }

    vi_map::VIMapManager map_manager;
    vi_map::VIMapManager::MapReadAccess map =
        map_manager.getMapReadAccess(selected_map_key);
    const std::string& save_path = FLAGS_csv_export_path;
    if (save_path.empty()) {
      LOG(ERROR) << "No path to export the CSV files into has been specified. "
                 << "Please specify using the --csv_export_path flag.";
      return common::kStupidUserError;
    }

    csv_export::exportMapToCsv(*map, save_path);
    return common::kSuccess;
  };
  addCommand(
      {"csv_export"}, csv_export,
      "Exports keyframe, keypoint and track, landmark and IMU data to CSV "
      "files in a folder specified by --csv_export_path. Check the "
      "documentation for information on the CSV format.",
      common::Processing::Sync);
  addCommand(
      {"csv_export_vertices_only"},
      [csv_export]() -> int {
        FLAGS_csv_export_imu_data = false;
        FLAGS_csv_export_tracks_and_keypoints = false;
        FLAGS_csv_export_descriptors = false;
        FLAGS_csv_export_landmarks = false;
        FLAGS_csv_export_observations = false;
        return csv_export();
      },
      "Exports only vertices in a CSV file in a folder specified by "
      "--csv_export_path.",
      common::Processing::Sync);

  addCommand(
      {"export_mission_info"}, [this]() -> int { return exportMissionInfo(); },
      "Exports a yaml that lists map key to mission id associations of all "
      "loaded maps so that data from the CSV exporter can more easily be "
      "linked to a specific mission.",
      common::Processing::Sync);

  addCommand(
      {"export_trajectory_to_csv", "ettc"},
      [this]() -> int { return exportPosesVelocitiesAndBiasesToCsv("asl"); },
      "Export poses, velocities and biases to a CSV file specified with "
      "--pose_export_file.",
      common::Processing::Sync);

  addCommand(
      {"export_trajectory_to_csv_in_rpg_format", "ettc_rpg"},
      [this]() -> int { return exportPosesVelocitiesAndBiasesToCsv("rpg"); },
      "Export timestamped posesto a CSV file in RPG trajectory evaluation "
      "format, specified with --pose_export_file.",
      common::Processing::Sync);

  addCommand(
      {"export_ncamera_calibration", "encc"},
      [this]() -> int { return exportNCameraCalibration(); },
      "Exports the ncamera calibration to the folder specified with "
      "--ncamera_calibration_export_folder.",
      common::Processing::Sync);
}

int DataImportExportPlugin::exportMissionInfo() const {
  const vi_map::VIMapManager map_manager;
  std::unordered_set<std::string> list_of_all_map_keys;
  map_manager.getAllMapKeys(&list_of_all_map_keys);
  if (list_of_all_map_keys.empty()) {
    LOG(ERROR) << "No maps are loaded.";
    return common::kStupidUserError;
  }
  if (FLAGS_mission_info_export_path.empty()) {
    LOG(ERROR) << "No export path has been specified, use "
               << "--mission_info_export_path to specify one.";
    return common::kStupidUserError;
  }

  if (!common::createPathToFile(FLAGS_mission_info_export_path)) {
    LOG(ERROR) << "Couldn't create path to \"" << FLAGS_mission_info_export_path
               << "\".";
    return common::kUnknownError;
  }

  common::FileLogger output_file(FLAGS_mission_info_export_path);
  for (const std::string& map_key : list_of_all_map_keys) {
    const vi_map::VIMapManager::MapReadAccess map =
        map_manager.getMapReadAccess(map_key);
    vi_map::MissionIdList list_of_all_mission_ids_in_map;
    map->getAllMissionIds(&list_of_all_mission_ids_in_map);
    if (!list_of_all_mission_ids_in_map.empty()) {
      output_file << map_key << ":\n";
      for (size_t idx = 0u; idx < list_of_all_mission_ids_in_map.size();
           ++idx) {
        const vi_map::MissionId& mission_id =
            list_of_all_mission_ids_in_map[idx];
        output_file << "  - index: " << idx << '\n';
        output_file << "    id: " << mission_id.hexString() << '\n';
      }
    }
  }

  return common::kSuccess;
}

int DataImportExportPlugin::exportPosesVelocitiesAndBiasesToCsv(
    const std::string& format /* = "asl" */) const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  if (FLAGS_map_mission.empty()) {
    map->getAllMissionIds(&mission_ids);
    if (mission_ids.empty()) {
      LOG(ERROR)
          << "There are no missions available in the loaded map. Aborting.";
      return common::kUnknownError;
    }
  } else {
    vi_map::MissionId mission_id;
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      LOG(ERROR) << "Mission ID invalid. Specify a valid mission id with "
                    "--map_mission.";
      return common::kUnknownError;
    }
    mission_ids.emplace_back(mission_id);
  }
  CHECK(!mission_ids.empty());

  const std::string kFilename = "vertex_poses_velocities_biases.csv";
  const std::string filepath =
      FLAGS_pose_export_file.empty()
          ? common::concatenateFolderAndFileName(map->getMapFolder(), kFilename)
          : FLAGS_pose_export_file;
  CHECK(!filepath.empty());

  aslam::SensorId reference_sensor_id;
  if (FLAGS_pose_export_reference_sensor_id.empty()) {
    // Pick first valid IMU from mission
    for (const vi_map::MissionId& mission_id : mission_ids) {
      const vi_map::VIMission& mission = map->getMission(mission_id);
      if (mission.hasImu()) {
        reference_sensor_id = mission.getImuId();
        break;
      }
    }

    if (!reference_sensor_id.isValid()) {
      LOG(ERROR)
          << "Could not find a mission with a valid IMU, reference sensor id"
          << "must be specified manually.";
      return common::kStupidUserError;
    }
  } else {
    reference_sensor_id.fromHexString(FLAGS_pose_export_reference_sensor_id);

    if (!reference_sensor_id.isValid()) {
      LOG(ERROR) << "Invalid sensor id.";
      return common::kStupidUserError;
    }
  }

  if (!map->getSensorManager().hasSensor(reference_sensor_id)) {
    LOG(ERROR) << "Sensor does not exist";
    return common::kStupidUserError;
  }
  data_import_export::exportPosesVelocitiesAndBiasesToCsv(
      *map, mission_ids, reference_sensor_id, filepath, format);
  return common::kSuccess;
}

int DataImportExportPlugin::exportNCameraCalibration() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  if (FLAGS_ncamera_calibration_export_folder.empty()) {
    LOG(ERROR) << "Specify a valid export folder with "
               << "--ncamera_calibration_export_folder.";
    return common::kStupidUserError;
  }

  data_import_export::exportNCameraCalibration(
      *map, FLAGS_ncamera_calibration_export_folder);
  return common::kSuccess;
}

}  // namespace data_import_export

MAPLAB_CREATE_CONSOLE_PLUGIN(data_import_export::DataImportExportPlugin);
