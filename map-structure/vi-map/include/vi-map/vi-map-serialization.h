#ifndef VI_MAP_VI_MAP_SERIALIZATION_H_
#define VI_MAP_VI_MAP_SERIALIZATION_H_

#include <string>
#include <vector>

#include <maplab-common/map-manager-config.h>
#include <maplab-common/network-common.h>

#include "vi-map/vi_map.pb.h"

namespace vi_map {

class VIMap;

namespace serialization {

namespace internal {

// Minimum number of protos when deserializing from network/file system. The
// minimum number of protos includes edges, missions, landmark_index and
// other_fields. (A vertices proto is only generated when there are vertices.)
constexpr size_t kMinNumProtos = 4u;
constexpr size_t kNumSensorYamlFiles = 1u;

constexpr size_t kProtoListMissionsIndex = 0u;
constexpr size_t kProtoListEdgesIndex = 1u;
constexpr size_t kProtoListLandmarkIndexIndex = 2u;
constexpr size_t kProtoListOptionalSensorData = 3u;
constexpr size_t kProtoListVerticesStartIndex =
    kMinNumProtos;  // Vertices appear after all other elements and may not
                    // exist.

constexpr char kFolderName[] = "vi_map";
constexpr char kFileNameVertices[] = "vertices";
constexpr char kFileNameEdges[] = "edges";
constexpr char kFileNameMissions[] = "missions";
constexpr char kFileNameLandmarkIndex[] = "landmark_index";
constexpr char kFileNameOptionalSensorData[] = "optional_sensor_data";

constexpr char kYamlSensorsFilename[] = "sensors.yaml";

const std::vector<std::string> kMinimumVIMapProtoFiles = {
    kFileNameMissions, kFileNameEdges, kFileNameLandmarkIndex,
    kFileNameOptionalSensorData};

size_t numberOfProtos(const VIMap& map, const backend::SaveConfig& save_config);

}  // namespace internal

void serializeVertices(const vi_map::VIMap& map, vi_map::proto::VIMap* proto);
// Only serializes vertices starting from the index given by start_index until
// vertices_per_proto vertices are in the proto or there are no more vertices in
// the map. Returns the index of the vertex one past the last serialized one.
size_t serializeVertices(
    const vi_map::VIMap& map, const size_t start_index,
    const size_t vertices_per_proto, vi_map::proto::VIMap* proto);
void serializeEdges(const vi_map::VIMap& map, vi_map::proto::VIMap* proto);
void serializeMissionsAndBaseframes(
    const vi_map::VIMap& map, vi_map::proto::VIMap* proto);
void serializeLandmarkIndex(
    const vi_map::VIMap& map, vi_map::proto::VIMap* proto);
void serializeOptionalSensorData(
    const VIMap& map, vi_map::proto::VIMap* proto);

void serializeSensorManagerToArray(
    const vi_map::VIMap& map, network::RawMessageData* raw_data);

// Note: Missions have to be deserialized before vertices.
void deserializeVertices(const vi_map::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeEdges(const vi_map::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeMissionsAndBaseframes(
    const vi_map::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeLandmarkIndex(
    const vi_map::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeOptionalSensorData(
    const vi_map::proto::VIMap& proto, vi_map::VIMap* map);

void deserializeSensorManagerFromArray(
    const network::RawMessageData& raw_data, vi_map::VIMap* map);

// The order of the protos for these functions is:
// missions, edges, landmark_index, other_fields, (vertices0, ...)
void serializeToListOfProtos(
    const vi_map::VIMap& map, const backend::SaveConfig& save_config,
    std::vector<vi_map::proto::VIMap>* list_of_protos);
void deserializeFromListOfProtos(
    const std::vector<vi_map::proto::VIMap>& list_of_protos,
    vi_map::VIMap* map);

// Serializes the map into individual protos, then calls the given function for
// further processing (e.g. save to disk or serialize to array).
// Returns the number of protos that have been serialized.
size_t serializeToFunction(
    const vi_map::VIMap& map, const backend::SaveConfig& save_config,
    const std::function<bool(const size_t, const proto::VIMap&)>&  // NOLINT
    function);

// ============================
// INTERACTION WITH FILE SYSTEM
// ============================
inline std::string getSubFolderName() {
  return internal::kFolderName;
}
bool getListOfExistingMapFiles(
    const std::string& map_folder, std::string* sensors_yaml_filepath,
    std::vector<std::string>* list_of_resource_filenames,
    std::vector<std::string>* list_of_map_proto_filepaths);
bool hasMapOnFileSystem(const std::string& folder_path);
bool loadMapFromFolder(const std::string& map_folder, vi_map::VIMap* map);
bool saveMapToFolder(
    const std::string& folder_path, const backend::SaveConfig& config,
    vi_map::VIMap* map);

// ==========
// NETWORKING
// ==========
void serializeToRawArray(
    const vi_map::VIMap& map, network::RawMessageDataList* raw_data);
void deserializeFromRawArray(
    const network::RawMessageDataList& raw_data, const size_t start_index,
    vi_map::VIMap* map);

}  // namespace serialization

}  // namespace vi_map

#endif  // VI_MAP_VI_MAP_SERIALIZATION_H_
