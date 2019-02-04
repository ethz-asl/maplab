#include "vi-map/vi-map-serialization-deprecated.h"

#include <map-resources/resource-map-serialization.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/proto-serialization-helper.h>

#include "vi-map/vi-map-serialization.h"
#include "vi-map/vi-map.h"

namespace vi_map {
namespace serialization {
namespace deprecated {

bool getListOfExistingMapFiles(
    const std::string& map_folder, std::string* sensors_yaml_filepath,
    std::vector<std::string>* list_of_resource_filenames,
    std::vector<std::string>* list_of_map_proto_filepaths) {
  CHECK_NOTNULL(sensors_yaml_filepath)->clear();
  CHECK_NOTNULL(list_of_resource_filenames)->clear();
  CHECK_NOTNULL(list_of_map_proto_filepaths)->clear();
  if (!common::pathExists(map_folder)) {
    return false;
  }

  bool minimum_required_files_exist =
      backend::resource_map_serialization::getListOfExistingMapFiles(
          map_folder, list_of_resource_filenames);

  std::string path_to_vi_map_folder;
  common::concatenateFolderAndFileName(
      map_folder, static_cast<std::string>(internal::kFolderName),
      &path_to_vi_map_folder);

  std::string vi_map_file;
  std::string path_to_vi_map_file;
  for (const std::string& map_file : internal::kMinimumVIMapProtoFiles) {
    common::concatenateFolderAndFileName(
        path_to_vi_map_folder, map_file, &path_to_vi_map_file);
    if (common::fileExists(path_to_vi_map_file)) {
      common::concatenateFolderAndFileName(
          internal::kFolderName, map_file, &vi_map_file);
      list_of_map_proto_filepaths->emplace_back(vi_map_file);
    }
  }

  size_t number_of_vertex_files = 0u;
  std::string vertex_proto_file_name =
      internal::kFileNameVertices + std::to_string(number_of_vertex_files);

  common::concatenateFolderAndFileName(
      path_to_vi_map_folder, vertex_proto_file_name, &path_to_vi_map_file);
  while (common::fileExists(path_to_vi_map_file)) {
    common::concatenateFolderAndFileName(
        internal::kFolderName, vertex_proto_file_name, &vi_map_file);
    list_of_map_proto_filepaths->emplace_back(vi_map_file);
    ++number_of_vertex_files;
    vertex_proto_file_name =
        internal::kFileNameVertices + std::to_string(number_of_vertex_files);
    common::concatenateFolderAndFileName(
        path_to_vi_map_folder, vertex_proto_file_name, &path_to_vi_map_file);
  }

  common::concatenateFolderAndFileName(
      path_to_vi_map_folder,
      static_cast<std::string>(internal::kYamlSensorsFilename),
      sensors_yaml_filepath);

  minimum_required_files_exist &= common::fileExists(*sensors_yaml_filepath);

  return minimum_required_files_exist;
}

bool loadMapFromFolder(const std::string& folder_path, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  std::string sensors_yaml_filepath;
  std::vector<std::string> list_of_resource_filepaths;
  std::vector<std::string> list_of_map_proto_filepaths;
  if (!common::pathExists(folder_path) ||
      !getListOfExistingMapFiles(
          folder_path, &sensors_yaml_filepath, &list_of_resource_filepaths,
          &list_of_map_proto_filepaths)) {
    LOG(ERROR) << "Map under \"" << folder_path << "\" does not exist!";
    return false;
  }

  // Load metadata first.
  CHECK(
      backend::resource_map_serialization::loadMetaDataFromFolder(
          folder_path, map));

  // Load the sensors.
  map->getSensorManager().deserializeFromFile(sensors_yaml_filepath);

  // Since we load this map from what obviously is the map folder, we can
  // replace the previous map folder (and resource folder) with the path of
  // the loaded map. This is important, because someone might have move the
  // map on the file system and those paths might be wrong now.
  map->replaceMapFolder(folder_path);

  const size_t number_of_protos = list_of_map_proto_filepaths.size();
  CHECK_GE(number_of_protos, internal::kMinNumProtos);

  common::MultiThreadedProgressBar progress_bar;
  std::string path_to_map_file;
  common::concatenateFolderAndFileName(
      folder_path, internal::kFolderName, &path_to_map_file);

  std::mutex map_mutex;

  std::function<void(const std::vector<size_t>)> load_function =
      [&map, list_of_map_proto_filepaths, path_to_map_file, &progress_bar,
       &map_mutex](const std::vector<size_t> range) {
        progress_bar.setNumElements(range.size());
        size_t num_processed_tasks = 0u;

        for (const size_t& task_idx : range) {
          CHECK_LT(task_idx, list_of_map_proto_filepaths.size());
          const std::string file_name =
              list_of_map_proto_filepaths[task_idx].substr(
                  std::strlen(internal::kFolderName) + 1u);
          proto::VIMap proto;

          // If the map was saved in an old map format, the file may not
          // exist. This is ok to do here since we already check for all
          // essential files above with hasMapOnFileSystem.
          std::string complete_path_to_file;
          common::concatenateFolderAndFileName(
              path_to_map_file, file_name, &complete_path_to_file);

          if (common::fileExists(complete_path_to_file)) {
            CHECK(!file_name.empty());
            CHECK(
                common::proto_serialization_helper::parseProtoFromFile(
                    path_to_map_file, file_name, &proto));

            switch (task_idx) {
              case internal::kProtoListMissionsIndex: {
                std::lock_guard<std::mutex> guard(map_mutex);
                deserializeMissionsAndBaseframes(proto, map);
              } break;
              case internal::kProtoListEdgesIndex: {
                std::lock_guard<std::mutex> guard(map_mutex);
                deserializeEdges(proto, map);
              } break;
              case internal::kProtoListLandmarkIndexIndex: {
                std::lock_guard<std::mutex> guard(map_mutex);
                deserializeLandmarkIndex(proto, map);
              } break;
              case internal::kProtoListOptionalSensorData: {
                std::lock_guard<std::mutex> guard(map_mutex);
                deserializeOptionalSensorData(proto, map);
              } break;
              default: {
                CHECK_GE(task_idx, internal::kProtoListVerticesStartIndex);
                std::lock_guard<std::mutex> guard(map_mutex);
                deserializeVertices(proto, map);
              } break;
            }
          } else {
            LOG(FATAL) << "Trying to read a proto file that does not exist!: "
                       << file_name;
          }
          progress_bar.update(++num_processed_tasks);
        }
      };

  VLOG(1) << "Reading missions...";
  load_function({internal::kProtoListMissionsIndex});

  // Deserialize vertices first after missions. This is because other fields
  // might depend on edges and can't be inserted into the map without the
  // corresponding vertices being present.
  constexpr bool kAlwaysParallelize = true;
  constexpr size_t kMaxNumberOfThreads = 8;
  const size_t num_threads =
      std::min(common::getNumHardwareThreads(), kMaxNumberOfThreads);

  const size_t end_index = number_of_protos;
  const size_t start_index = internal::kProtoListVerticesStartIndex;
  if (end_index > start_index) {
    VLOG(1) << "Reading vertices...";
    common::ParallelProcess(
        start_index, end_index, load_function, kAlwaysParallelize, num_threads);
  } else {
    VLOG(1) << "No vertex data found.";
  }

  VLOG(1) << "Reading edges...";
  load_function({internal::kProtoListEdgesIndex});

  VLOG(1) << "Reading landmarks...";
  load_function({internal::kProtoListLandmarkIndexIndex});

  VLOG(1) << "Reading optional sensor data...";
  load_function({internal::kProtoListOptionalSensorData});

  CHECK(
      backend::resource_map_serialization::loadMapFromFolder(folder_path, map));

  LOG(INFO) << "Loaded VIMap from \"" << folder_path << "\".";
  return true;
}

std::string getFileNameFromIndex(const size_t index) {
  std::string file_name;
  switch (index) {
    case internal::kProtoListMissionsIndex:
      file_name = internal::kFileNameMissions;
      break;
    case internal::kProtoListEdgesIndex:
      file_name = internal::kFileNameEdges;
      break;
    case internal::kProtoListLandmarkIndexIndex:
      file_name = internal::kFileNameLandmarkIndex;
      break;
    case internal::kProtoListOptionalSensorData:
      file_name = internal::kFileNameOptionalSensorData;
      break;
    default: {
      // Vertex files.
      CHECK_GE(index, internal::kProtoListVerticesStartIndex);
      file_name =
          internal::kFileNameVertices +
          std::to_string(index - internal::kProtoListVerticesStartIndex);
      break;
    }
  }
  return file_name;
}

}  // namespace deprecated
}  // namespace serialization
}  // namespace vi_map
