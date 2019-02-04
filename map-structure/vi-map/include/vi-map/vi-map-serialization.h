#ifndef VI_MAP_VI_MAP_SERIALIZATION_H_
#define VI_MAP_VI_MAP_SERIALIZATION_H_

#include <string>
#include <vector>

#include <maplab-common/map-manager-config.h>
#include <maplab-common/network-common.h>

#include "vi-map/vi-map-metadata.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class VIMap;

namespace serialization {

namespace internal {

// These are only used by the save command. Load reads the metadata file to
// get the file names.
constexpr char kFolderName[] = "vi_map";
constexpr char kFileNameMetadata[] = "vi_map_metadata";
constexpr char kFileNameVertices[] = "vertices";
constexpr char kFileNameEdges[] = "edges";
constexpr char kFileNameMissions[] = "missions";
constexpr char kFileNameLandmarkIndex[] = "landmark_index";
constexpr char kFileNameOptionalSensorData[] = "optional_sensor_data";

constexpr char kYamlSensorsFilename[] = "sensors.yaml";

constexpr size_t kMetadataIndexOffset = 0u;
constexpr size_t kSensorsYamlIndexOffset = 1u;

// This describes the index offset from the start of the VIMap to the first
// proto that describes a VIMap proto (i.e. a missions, vertices, edges, etc.
// proto). Files belonging to the VIMap, but are not a VIMap are the VIMap
// metadata and the sensors yaml. This is mainly relevant for the serialization
// to a list of byte arrays.
// Example layout of a VIMap serialized to a list of arrays:
// List index
// [ 0 ] <- metadata
// [ 1 ] <- sensor manager
// [ 2 ] <- begin_index + kRegularMapFilesOffset, proto 1 (e.g. missions)
// [ 3 ] <- proto 2 (e.g. vertices)
// [ 4 ] <- proto 3 (e.g. edges)
// [ 5 ] <- proto 4 (e.g. landmark index)
constexpr size_t kRegularMapFilesOffset = kSensorsYamlIndexOffset + 1u;

}  // namespace internal

void serializeVertices(const vi_map::VIMap& map, vi_map::proto::VIMap* proto);
// Only serializes vertices starting from the index given by start_index until
// vertices_per_proto vertices are in the proto or there are no more vertices in
// the map. Returns the index of the vertex one past the last serialized one.
size_t serializeVertices(
    const vi_map::VIMap& map, const size_t start_index,
    const size_t vertices_per_proto, vi_map::proto::VIMap* proto);
void serializeEdges(const vi_map::VIMap& map, vi_map::proto::VIMap* proto);
size_t serializeEdges(
    const vi_map::VIMap& map, const size_t start_index,
    const size_t edges_per_proto, vi_map::proto::VIMap* proto);
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

// Serializes the proto part of the map (everything without the sensor system)
// into individual protos, then calls the given function for further processing
// (e.g. save to disk or serialize to array).
// ProtoProcessFunction should be bool(const std::string&, const proto::VIMap*).
template <typename ProtoProcessFunction>
void serializeToProtoAndCallFunction(
    const vi_map::VIMap& map, const VIMapMetadata& metadata,
    const ProtoProcessFunction& function);

// ProtoSourceFunction should be bool(const std::string&, proto::VIMap*).
template <typename ProtoSourceFunction>
void deserializeFromProtoFromFunction(
    const VIMapMetadata& metadata, const ProtoSourceFunction& function,
    VIMap* map);

// ============================
// INTERACTION WITH FILE SYSTEM
// ============================
inline std::string getSubFolderName() {
  return internal::kFolderName;
}
bool getListOfExistingMapFiles(
    const std::string& map_folder, VIMapMetadata* metadata,
    std::vector<std::string>* list_of_resource_filenames);
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

#include "vi-map/vi-map-serialization-inl.h"

#endif  // VI_MAP_VI_MAP_SERIALIZATION_H_
