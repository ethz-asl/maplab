#include "vi-map/vi-map-serialization.h"

#include <functional>
#include <mutex>  // NOLINT
#include <string>
#include <thread>  // NOLINT

#include <aslam/common/timer.h>
#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <map-resources/resource-map-serialization.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/proto-serialization-helper.h>

#include "vi-map/vi-map.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {
namespace serialization {

void serializeVertices(const vi_map::VIMap& map, vi_map::proto::VIMap* proto) {
  constexpr size_t kStartIndex = 0u;
  // Serialize all vertices.
  const size_t number_of_vertices = map.numVertices();
  serializeVertices(map, kStartIndex, number_of_vertices, proto);
}

size_t serializeVertices(
    const vi_map::VIMap& map, const size_t start_index,
    const size_t vertices_per_proto, vi_map::proto::VIMap* proto) {
  CHECK_NOTNULL(proto);
  pose_graph::VertexIdList vertex_ids;
  map.getAllVertexIds(&vertex_ids);
  const size_t end_index =
      std::min(vertex_ids.size(), start_index + vertices_per_proto);
  size_t counter = start_index;
  proto->mutable_vertex_ids()->Reserve(end_index - counter);
  proto->mutable_vertices()->Reserve(end_index - counter);
  for (; counter < end_index; ++counter) {
    const pose_graph::VertexId& id = vertex_ids[counter];
    id.serialize(proto->add_vertex_ids());
    map.getVertex(id).serialize(proto->add_vertices());
  }

  return counter;
}

void serializeEdges(const vi_map::VIMap& map, vi_map::proto::VIMap* proto) {
  CHECK_NOTNULL(proto);
  pose_graph::EdgeIdList edge_ids;
  map.getAllEdgeIds(&edge_ids);
  proto->mutable_edge_ids()->Reserve(edge_ids.size());
  proto->mutable_edges()->Reserve(edge_ids.size());
  for (const pose_graph::EdgeId& id : edge_ids) {
    id.serialize(proto->add_edge_ids());
    map.getEdgeAs<vi_map::Edge>(id).serialize(proto->add_edges());
  }
}

void serializeMissionsAndBaseframes(
    const vi_map::VIMap& map, vi_map::proto::VIMap* proto) {
  CHECK_NOTNULL(proto);
  MissionIdList mission_ids;
  map.getAllMissionIds(&mission_ids);
  for (const MissionId& id : mission_ids) {
    id.serialize(proto->add_mission_ids());
    map.getMission(id).serialize(proto->add_missions());
  }

  MissionBaseFrameIdList mission_baseframe_ids;
  map.getAllMissionBaseFrameIds(&mission_baseframe_ids);
  for (const MissionBaseFrameId& id : mission_baseframe_ids) {
    id.serialize(proto->add_mission_base_frame_ids());
    map.getMissionBaseFrame(id).serialize(proto->add_mission_base_frames());
  }
}

void serializeLandmarkIndex(
    const vi_map::VIMap& map, vi_map::proto::VIMap* proto) {
  CHECK_NOTNULL(proto);
  LandmarkIdList landmark_ids;
  map.getAllLandmarkIds(&landmark_ids);
  const size_t num_landmarks = landmark_ids.size();
  proto->mutable_landmark_index()->Reserve(num_landmarks);
  for (const LandmarkId& id : landmark_ids) {
    vi_map::proto::LandmarkToVertexReference* reference =
        proto->add_landmark_index();
    CHECK_NOTNULL(reference);
    id.serialize(reference->mutable_landmark_id());
    map.getLandmarkStoreVertexId(id).serialize(reference->mutable_vertex_id());
  }
  CHECK_EQ(static_cast<int>(num_landmarks), proto->landmark_index_size());
}

void serializeOptionalSensorData(const VIMap& map, proto::VIMap* proto) {
  CHECK_NOTNULL(proto);
  MissionIdList mission_ids;
  map.getAllMissionIds(&mission_ids);
  for (const MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());
    if (map.hasOptionalSensorData(mission_id)) {
      const OptionalSensorData& optional_sensor_data =
          map.getOptionalSensorData(mission_id);
      vi_map::proto::OptionalSensorDataMissionPair*
          proto_optional_sensor_data_mission_pair =
              CHECK_NOTNULL(proto->add_optional_sensor_data_mission_id_pair());
      mission_id.serialize(
          proto_optional_sensor_data_mission_pair->mutable_mission_id());
      optional_sensor_data.serialize(
          proto_optional_sensor_data_mission_pair
              ->mutable_optional_sensor_data());
    }
  }
}

void deserializeVertices(
    const vi_map::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_EQ(proto.vertex_ids_size(), proto.vertices_size());
  for (int i = 0; i < proto.vertex_ids_size(); ++i) {
    pose_graph::VertexId id;
    id.deserialize(proto.vertex_ids(i));
    vi_map::Vertex* vertex(new vi_map::Vertex);
    vertex->deserialize(id, proto.vertices(i));

    const vi_map::MissionId& mission_id = vertex->getMissionId();
    CHECK(map->hasMission(mission_id));
    aslam::NCamera::Ptr ncamera =
        map->getSensorManager().getNCameraSharedForMission(mission_id);
    CHECK(ncamera);
    vertex->setNCameras(ncamera);

    map->addVertex(vi_map::Vertex::UniquePtr(vertex));
  }
}

void deserializeEdges(const vi_map::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_EQ(proto.edge_ids_size(), proto.edges_size());
  for (int i = 0; i < proto.edge_ids_size(); ++i) {
    pose_graph::EdgeId id;
    id.deserialize(proto.edge_ids(i));
    map->addEdge(vi_map::Edge::deserialize(id, proto.edges(i)));
  }
}

void deserializeMissionsAndBaseframes(
    const vi_map::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_EQ(proto.mission_ids_size(), proto.missions_size());
  CHECK_EQ(
      proto.mission_base_frame_ids_size(), proto.mission_base_frames_size());

  // The baseframes are not necessarily stored in corresponding order to
  // missions. So we first load the baseframes and then assign the correct
  // mission. Each mission has its own baseframe though.
  CHECK_EQ(proto.missions_size(), proto.mission_base_frames_size());

  std::vector<MissionBaseFrame> baseframes(proto.mission_base_frames_size());
  std::unordered_map<MissionBaseFrameId, size_t> baseframeid_idx;

  for (int i = 0; i < proto.mission_base_frames_size(); ++i) {
    MissionBaseFrameId mission_frame_id;
    mission_frame_id.deserialize(proto.mission_base_frame_ids(i));
    CHECK(mission_frame_id.isValid());

    baseframes[i].deserialize(mission_frame_id, proto.mission_base_frames(i));
    CHECK(baseframeid_idx.emplace(mission_frame_id, i).second);
  }

  for (int i = 0; i < proto.missions_size(); ++i) {
    MissionId mission_id;
    mission_id.deserialize(proto.mission_ids(i));
    CHECK(mission_id.isValid());

    VIMission* mission(new VIMission);
    mission->deserialize(mission_id, proto.missions(i));

    const MissionBaseFrameId& mission_frame_id = mission->getBaseFrameId();
    CHECK(mission_frame_id.isValid());
    const size_t baseframe_idx =
        common::getChecked(baseframeid_idx, mission_frame_id);

    map->addNewMissionWithBaseframe(
        VIMission::UniquePtr(mission), baseframes[baseframe_idx]);
  }
}

void deserializeLandmarkIndex(
    const vi_map::proto::VIMap& proto, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  for (int i = 0; i < proto.landmark_index_size(); ++i) {
    LandmarkId landmark_id;
    landmark_id.deserialize(proto.landmark_index(i).landmark_id());
    pose_graph::VertexId storing_vertex_id;
    storing_vertex_id.deserialize(proto.landmark_index(i).vertex_id());

    if (map->hasLandmark(landmark_id)) {
      CHECK_EQ(storing_vertex_id, map->getLandmarkStoreVertexId(landmark_id));
    } else {
      map->addLandmarkIndexReference(landmark_id, storing_vertex_id);
    }
  }

  if (proto.landmark_index_ids_size() > 0) {
    LOG(WARNING) << "Legacy map with store/global landmark ids found. "
                 << "Cleaning up the ids.";
    CHECK_EQ(proto.landmark_index_size(), proto.landmark_index_ids_size());

    // Go over all the landmarks and change the ids in the visualframes
    // to the actual landmark id (instead of the old global landmark id).
    vi_map::LandmarkIdList landmark_ids;
    map->getAllLandmarkIds(&landmark_ids);
    for (const vi_map::LandmarkId& actual_landmark_id : landmark_ids) {
      const vi_map::Landmark& landmark = map->getLandmark(actual_landmark_id);
      landmark.forEachObservation(
          [&](const vi_map::KeypointIdentifier& keypoint_id) {
            vi_map::Vertex& vertex =
                map->getVertex(keypoint_id.frame_id.vertex_id);
            vertex.setObservedLandmarkId(keypoint_id, actual_landmark_id);
          });
    }
  }
}

void deserializeOptionalSensorData(const proto::VIMap& proto, VIMap* map) {
  CHECK_NOTNULL(map);
  for (const proto::OptionalSensorDataMissionPair&
           proto_mission_id_with_sensor_data :
       proto.optional_sensor_data_mission_id_pair()) {
    MissionId mission_id;
    mission_id.deserialize(proto_mission_id_with_sensor_data.mission_id());
    CHECK(mission_id.isValid());
    map->emplaceOptionalSensorData(
        mission_id, proto_mission_id_with_sensor_data.optional_sensor_data());
  }
}

namespace internal {

size_t numberOfProtos(
    const VIMap& map, const backend::SaveConfig& save_config) {
  const size_t num_vertices = map.numVertices();
  const size_t num_vertices_protos =
      (save_config.vertices_per_proto_file - 1u + num_vertices) /
      save_config.vertices_per_proto_file;
  return internal::kMinNumProtos + num_vertices_protos;
}

}  // namespace internal

void serializeToListOfProtos(
    const vi_map::VIMap& map, const backend::SaveConfig& save_config,
    std::vector<proto::VIMap>* list_of_protos) {
  CHECK_NOTNULL(list_of_protos)->clear();

  const size_t num_protos = internal::numberOfProtos(map, save_config);
  list_of_protos->insert(list_of_protos->end(), num_protos, proto::VIMap());

  serializeToFunction(
      map, save_config,
      [&list_of_protos](
          const size_t task_idx, const proto::VIMap& proto) -> bool {
        CHECK_LT(task_idx, list_of_protos->size());
        (*list_of_protos)[task_idx] = proto;
        return true;
      });
}

size_t serializeToFunction(
    const vi_map::VIMap& map, const backend::SaveConfig& save_config,
    const std::function<bool(const size_t, const proto::VIMap&)>&  // NOLINT
    function) {
  CHECK(function);
  const size_t num_protos = internal::numberOfProtos(map, save_config);

  common::MultiThreadedProgressBar progress_bar;
  std::function<void(const std::vector<size_t>)> serialize_function =
      [&](const std::vector<size_t>& range) {
        size_t num_processed_tasks = 0u;
        progress_bar.setNumElements(2u * range.size());
        proto::VIMap proto;
        for (size_t task_idx : range) {
          proto.Clear();
          switch (task_idx) {
            case internal::kProtoListMissionsIndex:
              serializeMissionsAndBaseframes(map, &proto);
              break;
            case internal::kProtoListEdgesIndex:
              serializeEdges(map, &proto);
              break;
            case internal::kProtoListLandmarkIndexIndex:
              serializeLandmarkIndex(map, &proto);
              break;
            case internal::kProtoListOptionalSensorData:
              serializeOptionalSensorData(map, &proto);
              break;
            default: {
              // Case vertices.
              serializeVertices(
                  map, (task_idx - internal::kProtoListVerticesStartIndex) *
                           save_config.vertices_per_proto_file,
                  save_config.vertices_per_proto_file, &proto);
              break;
            }
          }
          progress_bar.update(++num_processed_tasks);

          CHECK(function(task_idx, proto));
          progress_bar.update(++num_processed_tasks);
        }
      };

  constexpr bool kAlwaysParallelize = true;
  constexpr size_t kMaxNumberOfThreads = 8;
  const size_t num_threads =
      std::min(common::getNumHardwareThreads(), kMaxNumberOfThreads);
  common::ParallelProcess(
      num_protos, serialize_function, kAlwaysParallelize, num_threads);
  return num_protos;
}

void deserializeFromListOfProtos(
    const std::vector<vi_map::proto::VIMap>& list_of_protos,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK_GE(list_of_protos.size(), internal::kMinNumProtos);

  deserializeMissionsAndBaseframes(
      list_of_protos[internal::kProtoListMissionsIndex], map);

  // Deserialize vertices first after missions. This is because other fields
  // might depend on edges and can't be inserted into the map without the
  // corresponding vertices being present.
  for (size_t vertices_index = internal::kProtoListVerticesStartIndex;
       vertices_index < list_of_protos.size(); ++vertices_index) {
    deserializeVertices(list_of_protos[vertices_index], map);
  }

  deserializeEdges(list_of_protos[internal::kProtoListEdgesIndex], map);
  deserializeLandmarkIndex(
      list_of_protos[internal::kProtoListLandmarkIndexIndex], map);
  deserializeOptionalSensorData(
      list_of_protos[internal::kProtoListOptionalSensorData], map);
}

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

  std::string vi_map_file;
  std::string path_to_vi_map_folder;
  common::concatenateFolderAndFileName(
      map_folder, static_cast<std::string>(internal::kFolderName),
      &path_to_vi_map_folder);

  std::string path_to_vi_map_file;
  bool only_optional_sensor_data_proto_missing = true;
  for (const std::string& map_file : internal::kMinimumVIMapProtoFiles) {
    common::concatenateFolderAndFileName(
        path_to_vi_map_folder, map_file, &path_to_vi_map_file);
    if (common::fileExists(path_to_vi_map_file)) {
      common::concatenateFolderAndFileName(
          internal::kFolderName, map_file, &vi_map_file);
      list_of_map_proto_filepaths->emplace_back(vi_map_file);
    } else {
      if (map_file != internal::kFileNameOptionalSensorData) {
        only_optional_sensor_data_proto_missing = false;
      }
      minimum_required_files_exist = false;
    }
  }

  LOG_IF(
      ERROR,
      !minimum_required_files_exist && only_optional_sensor_data_proto_missing)
      << "Unable to find the optional sensor data proto file "
      << internal::kFileNameOptionalSensorData
      << ", but all the other required VI-Map proto files are present. "
      << "This may indicate that you are trying "
      << "to load a map with a deprecated map format. You can try "
      << "to convert the map to the new format using the command "
      << "'convert_map_to_new_format --map_folder <path_to_deprecated_map>'.";

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

bool hasMapOnFileSystem(const std::string& folder_path) {
  std::string sensors_yaml_filepath;
  std::vector<std::string> list_of_resource_filepaths;
  std::vector<std::string> list_of_map_proto_filepaths;
  return getListOfExistingMapFiles(
      folder_path, &sensors_yaml_filepath, &list_of_resource_filepaths,
      &list_of_map_proto_filepaths);
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

bool saveMapToFolder(
    const std::string& folder_path, const backend::SaveConfig& config,
    vi_map::VIMap* map) {
  std::string complete_folder_path;
  common::concatenateFolderAndFileName(
      folder_path, getSubFolderName(), &complete_folder_path);

  // Check if path is the name of an already existing directory or file.
  if (common::fileExists(complete_folder_path) ||
      (!config.overwrite_existing_files &&
       common::pathExists(complete_folder_path))) {
    LOG(ERROR) << "Cannot save map because file already exists.";
    return false;
  }

  if (!common::createPath(complete_folder_path)) {
    LOG(ERROR) << "Could not create path to file!";
    return false;
  }

  map->setMapFolder(folder_path);

  // Serialize the sensors.
  std::string sensors_yaml_filepath;
  common::concatenateFolderAndFileName(
      complete_folder_path, internal::kYamlSensorsFilename,
      &sensors_yaml_filepath);
  map->getSensorManager().serializeToFile(sensors_yaml_filepath);

  const size_t num_files = serializeToFunction(
      *map, config,
      [&complete_folder_path](
          const size_t task_idx, const proto::VIMap& proto) -> bool {
        const std::string file_name = getFileNameFromIndex(task_idx);
        CHECK(!file_name.empty());
        return common::proto_serialization_helper::serializeProtoToFile(
            complete_folder_path, file_name, proto);
      });

  // Delete leftover vertex files from previous maps.
  size_t vertex_file_index_to_delete = num_files - internal::kMinNumProtos;
  std::string vertex_file_to_delete =
      internal::kFileNameVertices + std::to_string(vertex_file_index_to_delete);
  std::string path_to_vertex_file = common::concatenateFolderAndFileName(
      complete_folder_path, vertex_file_to_delete);
  while (common::fileExists(path_to_vertex_file)) {
    CHECK(common::deleteFile(path_to_vertex_file));

    vertex_file_to_delete = internal::kFileNameVertices +
                            std::to_string(vertex_file_index_to_delete);
    path_to_vertex_file = common::concatenateFolderAndFileName(
        complete_folder_path, vertex_file_to_delete);
  }

  // Serialize resource map.
  backend::resource_map_serialization::saveMapToFolder(
      folder_path, config, map);

  LOG(INFO) << "Saved map in \"" << folder_path << "\".";
  return true;
}

void serializeSensorManagerToArray(
    const vi_map::VIMap& map, network::RawMessageData* raw_data) {
  CHECK_NOTNULL(raw_data);
  YAML::Node sensors_yaml_node;
  map.getSensorManager().serialize(&sensors_yaml_node);
  std::ostringstream sensors_yaml_ostream;
  sensors_yaml_ostream << sensors_yaml_node;
  const size_t num_characters = sensors_yaml_ostream.str().size();
  CHECK_GT(num_characters, 0u);
  const size_t num_bytes = num_characters + 1u;
  raw_data->first = new uint8_t[num_bytes];
  std::memcpy(raw_data->first, sensors_yaml_ostream.str().c_str(), num_bytes);
  raw_data->second = num_bytes;
}

void serializeToRawArray(
    const vi_map::VIMap& map, network::RawMessageDataList* raw_data) {
  CHECK_NOTNULL(raw_data);

  // Use default number of vertices per proto.
  const backend::SaveConfig save_config;

  // Example layout: num_protos = 5, kNumSensorYamlFiles = 1
  // Index
  // [ 0 ]
  // [ 1 ]
  // [ 2 ] <- begin_index, sensor manager
  // [ 3 ] <- proto 1
  // [ 4 ] <- proto 2
  // [ 5 ] <- proto 3
  // [ 6 ] <- proto 4
  // [ 7 ] <- proto 5

  const size_t begin_index = raw_data->size();
  const size_t num_protos = internal::numberOfProtos(map, save_config);

  raw_data->insert(
      raw_data->end(), num_protos + internal::kNumSensorYamlFiles,
      network::RawMessageData());

  const size_t sensor_manager_index = begin_index;

  CHECK_LT(sensor_manager_index, raw_data->size());
  network::RawMessageData& raw_data_element = (*raw_data)[sensor_manager_index];
  serializeSensorManagerToArray(map, &raw_data_element);

  const size_t proto_begin_index = begin_index + internal::kNumSensorYamlFiles;
  CHECK_EQ(raw_data->size(), proto_begin_index + num_protos);
  serializeToFunction(
      map, save_config,
      [&raw_data, &proto_begin_index](
          const size_t task_idx, const proto::VIMap& proto) -> bool {
        const size_t index = proto_begin_index + task_idx;
        network::RawMessageData& raw_data_element = (*raw_data)[index];
        LOG(INFO) << "Serializing proto at index " << index << '.';
        common::proto_serialization_helper::serializeToArray(
            proto, &raw_data_element.first, &raw_data_element.second);
        return true;
      });
}

void deserializeSensorManagerFromArray(
    const network::RawMessageData& raw_data, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  const size_t num_bytes = raw_data.second;
  CHECK_GT(num_bytes, 0u);

  static_assert(
      sizeof(char) == sizeof(uint8_t),
      "Expected char to be of size 1 byte, "
      "but this does not seems to be the case.");

  YAML::Node sensors_yaml_node;
  try {
    sensors_yaml_node =
        YAML::Load(static_cast<const char* const>(raw_data.first));
  } catch (const YAML::ParserException& parser_exception) {
    LOG(FATAL) << "Failed to load sensor YAML file from raw data buffer with "
               << "exception: " << parser_exception.what();
  }
  CHECK(sensors_yaml_node.IsDefined());
  CHECK(sensors_yaml_node.IsMap());
  map->getSensorManager().deserialize(sensors_yaml_node);
}

void deserializeFromRawArray(
    const network::RawMessageDataList& raw_data, const size_t start_index,
    vi_map::VIMap* map) {
  // Example layout: kMinNumProtos = 3, kNumSensorYamlFiles = 1
  // Index
  // [ 0 ]
  // [ 1 ]
  // [ 2 ] <- start_index, sensor manager
  // [ 3 ] <- proto 1
  // [ 4 ] <- proto 2
  // [ 5 ] <- proto 3
  // [ 6 ] <- proto 4
  // [ 7 ] <- proto 5
  CHECK_NOTNULL(map);
  CHECK_GE(
      raw_data.size(),
      start_index + internal::kMinNumProtos + internal::kNumSensorYamlFiles);

  const size_t sensor_manager_index = start_index;
  deserializeSensorManagerFromArray(raw_data[sensor_manager_index], map);

  const size_t num_protos =
      raw_data.size() - start_index - internal::kNumSensorYamlFiles;
  std::vector<proto::VIMap> proto_list(num_protos);
  constexpr bool kAlwaysParallelize = true;
  const size_t num_threads = common::getNumHardwareThreads();
  const size_t proto_start_index = start_index + internal::kNumSensorYamlFiles;
  common::ParallelProcess(
      num_protos,
      [&raw_data, &proto_list,
       &proto_start_index](const std::vector<size_t>& range) {
        for (size_t task_idx : range) {
          const size_t raw_data_idx = proto_start_index + task_idx;
          common::proto_serialization_helper::deserializeFromArray(
              raw_data[raw_data_idx].first, raw_data[raw_data_idx].second,
              &proto_list[task_idx]);
        }
      },
      kAlwaysParallelize, num_threads);

  deserializeFromListOfProtos(proto_list, map);
}

}  // namespace serialization

}  // namespace vi_map
