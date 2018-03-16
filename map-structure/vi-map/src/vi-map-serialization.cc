#include "vi-map/vi-map-serialization.h"

#include <string>

#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <map-resources/resource-map-serialization.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/proto-serialization-helper.h>

#include "vi-map/vi-map-metadata.h"
#include "vi-map/vi-map-serialization-deprecated.h"
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
  // Serialize all edges.
  const size_t num_edges = map.numEdges();
  constexpr size_t kStartIndex = 0u;
  serializeEdges(map, kStartIndex, num_edges, proto);
}

size_t serializeEdges(
    const vi_map::VIMap& map, const size_t start_index,
    const size_t edges_per_proto, vi_map::proto::VIMap* proto) {
  CHECK_NOTNULL(proto);
  pose_graph::EdgeIdList edge_ids;
  map.getAllEdgeIds(&edge_ids);
  const size_t end_index =
      std::min(edge_ids.size(), start_index + edges_per_proto);
  size_t counter = start_index;
  proto->mutable_edge_ids()->Reserve(end_index - counter);
  proto->mutable_edges()->Reserve(end_index - counter);
  for (; counter < end_index; ++counter) {
    const pose_graph::EdgeId& id = edge_ids[counter];
    id.serialize(proto->add_edge_ids());
    map.getEdgeAs<vi_map::Edge>(id).serialize(proto->add_edges());
  }

  return counter;
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

namespace {

VIMapMetadata createMetadataForVIMap(const vi_map::VIMap& map) {
  const size_t num_vertices_in_map = map.numVertices();
  const size_t num_vertices_files =
      (backend::SaveConfig::kVerticesPerProtoFile - 1u + num_vertices_in_map) /
      backend::SaveConfig::kVerticesPerProtoFile;

  const size_t num_edges_in_map = map.numEdges();
  const size_t num_edges_files =
      (backend::SaveConfig::kEdgesPerProtoFile - 1u + num_edges_in_map) /
      backend::SaveConfig::kEdgesPerProtoFile;

  VIMapMetadata metadata;

  auto fill_function = [&metadata](
      const VIMapFileType& file_type, const std::string& base_file_name,
      const size_t num_files) {
    for (size_t i = 0u; i < num_files; ++i) {
      const std::string file_name = base_file_name + std::to_string(i);
      metadata.emplace(file_type, file_name);
    }
  };
  metadata.emplace(VIMapFileType::kMissions, internal::kFileNameMissions);
  fill_function(
      VIMapFileType::kVertices, internal::kFileNameVertices,
      num_vertices_files);
  fill_function(
      VIMapFileType::kEdges, internal::kFileNameEdges, num_edges_files);
  metadata.emplace(
      VIMapFileType::kLandmarkIndex, internal::kFileNameLandmarkIndex);
  metadata.emplace(
      VIMapFileType::kOptionalSensorData,
      internal::kFileNameOptionalSensorData);

  return metadata;
}

}  // namespace

bool getListOfExistingMapFiles(
    const std::string& map_folder, VIMapMetadata* metadata,
    std::vector<std::string>* list_of_resource_filenames) {
  CHECK_NOTNULL(metadata)->clear();
  CHECK_NOTNULL(list_of_resource_filenames)->clear();
  if (!common::pathExists(map_folder)) {
    return false;
  }

  bool all_map_files_exist =
      backend::resource_map_serialization::getListOfExistingMapFiles(
          map_folder, list_of_resource_filenames);

  std::string path_to_vi_map_folder;
  common::concatenateFolderAndFileName(
      map_folder, static_cast<std::string>(internal::kFolderName),
      &path_to_vi_map_folder);
  const std::string metadata_filepath = common::concatenateFolderAndFileName(
      path_to_vi_map_folder, internal::kFileNameMetadata);

  if (!common::fileExists(metadata_filepath)) {
    // Deprecated map format.
    std::string sensors_yaml_filepath;
    std::vector<std::string> list_of_map_proto_filepaths;
    return deprecated::getListOfExistingMapFiles(
        map_folder, &sensors_yaml_filepath, list_of_resource_filenames,
        &list_of_map_proto_filepaths);
  }

  // There needs to be exactly one sensors file.
  if (!common::fileExists(
          common::concatenateFolderAndFileName(
              path_to_vi_map_folder, internal::kYamlSensorsFilename))) {
    LOG(ERROR) << "The sensor manager file (" << internal::kYamlSensorsFilename
               << ") doesn't exist in the map folder.";
    return false;
  }

  vi_map::proto::VIMapMetadata metadata_proto;
  constexpr bool kIsTextFormat = true;
  if (!common::proto_serialization_helper::parseProtoFromFile(
          path_to_vi_map_folder, internal::kFileNameMetadata, &metadata_proto,
          kIsTextFormat)) {
    LOG(ERROR) << "Couldn't read the map metadata file.";
    return false;
  }

  deserializeMetadata(metadata_proto, metadata);
  if (metadata->empty()) {
    LOG(ERROR) << "The map doesn't seem to contain any files.";
    return false;
  }
  for (const VIMapMetadata::value_type& value : *metadata) {
    const std::string complete_path = common::concatenateFolderAndFileName(
        path_to_vi_map_folder, value.second);
    if (!common::fileExists(complete_path)) {
      LOG(ERROR) << "File \"" << complete_path
                 << "\" doesn't exist, but it should.";
      all_map_files_exist = false;
    }
  }

  return all_map_files_exist;
}

bool hasMapOnFileSystem(const std::string& folder_path) {
  std::string metadata_filepath;
  VIMapMetadata metadata;
  std::vector<std::string> list_of_resource_filepaths;
  return getListOfExistingMapFiles(
      folder_path, &metadata, &list_of_resource_filepaths);
}

bool loadMapFromFolder(const std::string& folder_path, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  VIMapMetadata metadata;
  std::vector<std::string> list_of_resource_filepaths;
  if (!common::pathExists(folder_path) ||
      !getListOfExistingMapFiles(
          folder_path, &metadata, &list_of_resource_filepaths)) {
    LOG(ERROR) << "Map under \"" << folder_path
               << "\" does not exist or is incomplete!";
    return false;
  }

  if (metadata.empty()) {
    // Try loading with the deprecated style.
    LOG(WARNING) << "!!! WARNING !!!\nThis map is in a deprecated format. To "
                 << "convert the map to the new format, please resave the map. "
                 << "Please convert all your maps as soon as possible to the "
                 << "new format by loading and resaving them.";
    return deprecated::loadMapFromFolder(folder_path, map);
  }

  // Load map resources metadata first.
  CHECK(
      backend::resource_map_serialization::loadMetaDataFromFolder(
          folder_path, map));

  // Since we load this map from what obviously is the map folder, we can
  // replace the previous map folder (and resource folder) with the path of
  // the loaded map. This is important, because someone might have move the
  // map on the file system and those paths might be wrong now.
  map->replaceMapFolder(folder_path);

  std::string path_to_vi_map_folder;
  common::concatenateFolderAndFileName(
      folder_path, internal::kFolderName, &path_to_vi_map_folder);

  // Load the sensors.
  map->getSensorManager().deserializeFromFile(
      common::concatenateFolderAndFileName(
          path_to_vi_map_folder, internal::kYamlSensorsFilename));

  deserializeFromProtoFromFunction(
      metadata,
      [&](const std::string& file_name, proto::VIMap* proto) -> bool {
        CHECK_NOTNULL(proto);
        return common::proto_serialization_helper::parseProtoFromFile(
            path_to_vi_map_folder, file_name, proto);
      },
      map);

  CHECK(
      backend::resource_map_serialization::loadMapFromFolder(folder_path, map));

  LOG(INFO) << "Loaded VIMap from \"" << folder_path << "\".";
  return true;
}

bool saveMapToFolder(
    const std::string& folder_path, const backend::SaveConfig& config,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

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

  VIMapMetadata metadata = createMetadataForVIMap(*map);
  // Metadata should at least contain one reference to the metadata file and
  // one reference to the sensor yaml.
  CHECK_GE(metadata.size(), 2u);
  if (!config.overwrite_existing_files) {
    // Check that no file that should be written already exists.
    for (const VIMapMetadata::value_type& value : metadata) {
      const std::string path_to_map_file = common::concatenateFolderAndFileName(
          complete_folder_path, value.second);
      if (common::pathExists(path_to_map_file) ||
          common::fileExists(path_to_map_file)) {
        LOG(ERROR) << "Map can't be saved because a file would be overwritten.";
        return false;
      }
    }
  }

  map->setMapFolder(folder_path);

  // Serialize the sensors.
  std::string sensors_yaml_filepath;
  common::concatenateFolderAndFileName(
      complete_folder_path, internal::kYamlSensorsFilename,
      &sensors_yaml_filepath);
  map->getSensorManager().serializeToFile(sensors_yaml_filepath);

  serializeToProtoAndCallFunction(
      *map, metadata,
      [&](const std::string& file_name, const proto::VIMap& proto) -> bool {
        CHECK(!file_name.empty());
        return common::proto_serialization_helper::serializeProtoToFile(
            complete_folder_path, file_name, proto);
      });
  // Save metadata.
  constexpr bool kIsTextFormat = true;
  proto::VIMapMetadata metadata_proto;
  serializeMetadata(metadata, &metadata_proto);
  common::proto_serialization_helper::serializeProtoToFile(
      complete_folder_path, internal::kFileNameMetadata, metadata_proto,
      kIsTextFormat);

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

namespace {

typedef std::unordered_map<std::string, size_t> MetadataNameToIndexMap;

MetadataNameToIndexMap createMetadataNameToIndexInverseMap(
    const VIMapMetadata& metadata) {
  MetadataNameToIndexMap name_to_index_map;
  {
    // Start at 2 to skip metadata and sensor manager files.
    size_t counter = internal::kRegularMapFilesOffset;
    for (const VIMapMetadata::value_type& entry : metadata) {
      CHECK(name_to_index_map.emplace(entry.second, counter).second);
      ++counter;
    }
    CHECK_EQ(counter, metadata.size() + internal::kRegularMapFilesOffset);
  }

  return name_to_index_map;
}

}  // namespace

void serializeToRawArray(
    const vi_map::VIMap& map, network::RawMessageDataList* raw_data) {
  CHECK_NOTNULL(raw_data);

  // Use default number of vertices per proto.
  const backend::SaveConfig save_config;

  // Example layout:
  // Index
  // [ 0 ]
  // [ 1 ]
  // [ 2 ] <- begin_index, metadata
  // [ 3 ] <- sensor manager
  // [ 4 ] <- begin_index + kRegularMapFilesOffset, proto 1 (e.g. missions)
  // [ 5 ] <- proto 2 (e.g. vertices)
  // [ 6 ] <- proto 3 (e.g. edges)
  // [ 7 ] <- proto 4 (e.g. landmark index)

  const size_t begin_index = raw_data->size();
  const VIMapMetadata metadata = createMetadataForVIMap(map);
  const size_t num_files = metadata.size() + internal::kRegularMapFilesOffset;

  raw_data->insert(raw_data->end(), num_files, network::RawMessageData());
  CHECK_EQ(begin_index + num_files, raw_data->size());

  // Build an inverse map of name -> index.
  MetadataNameToIndexMap name_to_index_map =
      createMetadataNameToIndexInverseMap(metadata);

  // Insert metadata and sensor manager.
  {
    // Metadata always comes first.
    network::RawMessageData& raw_data_element =
        (*raw_data)[begin_index + internal::kMetadataIndexOffset];

    proto::VIMapMetadata metadata_proto;
    serializeMetadata(metadata, &metadata_proto);
    common::proto_serialization_helper::serializeToArray(
        metadata_proto, &raw_data_element.first, &raw_data_element.second);
  }
  {
    network::RawMessageData& raw_data_element =
        (*raw_data)[begin_index + internal::kSensorsYamlIndexOffset];
    serializeSensorManagerToArray(map, &raw_data_element);
  }

  serializeToProtoAndCallFunction(
      map, metadata,
      [&raw_data, &name_to_index_map, &begin_index](
          const std::string& name, const proto::VIMap& proto) -> bool {
        const MetadataNameToIndexMap::const_iterator name_to_index_iterator =
            name_to_index_map.find(name);
        if (name_to_index_iterator == name_to_index_map.cend()) {
          return false;
        }
        const size_t index = begin_index + name_to_index_iterator->second;
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
  // Example layout:
  // Index
  // [ 0 ]
  // [ 1 ]
  // [ 2 ] <- start_index, metadata
  // [ 3 ] <- sensor manager
  // [ 4 ] <- begin_index + kRegularMapFilesOffset, proto 1 (e.g. missions)
  // [ 5 ] <- proto 2 (e.g. vertices)
  // [ 6 ] <- proto 3 (e.g. edges)
  // [ 7 ] <- proto 4 (e.g. landmark index)
  CHECK_NOTNULL(map);

  // Check that there is a metadata file.
  const size_t metadata_index = start_index + internal::kMetadataIndexOffset;
  CHECK_GT(raw_data.size(), metadata_index);
  proto::VIMapMetadata metadata_proto;
  const network::RawMessageData& metadata_raw_data_pair =
      raw_data[metadata_index];
  common::proto_serialization_helper::deserializeFromArray(
      metadata_raw_data_pair.first, metadata_raw_data_pair.second,
      &metadata_proto);
  VIMapMetadata metadata;
  deserializeMetadata(metadata_proto, &metadata);

  CHECK_GE(
      raw_data.size(),
      start_index + internal::kRegularMapFilesOffset + metadata.size());

  // Deserialize sensor manager.
  deserializeSensorManagerFromArray(
      raw_data[start_index + internal::kSensorsYamlIndexOffset], map);

  // Build an inverse map of name -> index.
  MetadataNameToIndexMap name_to_index_map =
      createMetadataNameToIndexInverseMap(metadata);

  deserializeFromProtoFromFunction(
      metadata,
      [&](const std::string& name, proto::VIMap* proto) -> bool {
        CHECK_NOTNULL(proto);
        const size_t index = common::getChecked(name_to_index_map, name);
        const network::RawMessageData& raw_data_element =
            raw_data[start_index + index];
        common::proto_serialization_helper::deserializeFromArray(
            raw_data_element.first, raw_data_element.second, proto);
        return true;
      },
      map);
}

}  // namespace serialization
}  // namespace vi_map
