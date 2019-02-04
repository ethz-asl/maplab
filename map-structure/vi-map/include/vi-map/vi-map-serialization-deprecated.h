#ifndef VI_MAP_VI_MAP_SERIALIZATION_DEPRECATED_H_
#define VI_MAP_VI_MAP_SERIALIZATION_DEPRECATED_H_

#include <string>
#include <vector>

#include <maplab-common/map-manager-config.h>

namespace vi_map {

class VIMap;

namespace serialization {
namespace deprecated {

namespace internal {

// Minimum number of protos when deserializing from network/file system. The
// minimum number of protos includes edges, missions, landmark_index and
// other_fields. (A vertices proto is only generated when there are vertices.)
constexpr size_t kMinNumProtos = 4u;

constexpr size_t kProtoListMissionsIndex = 0u;
constexpr size_t kProtoListEdgesIndex = 1u;
constexpr size_t kProtoListLandmarkIndexIndex = 2u;
constexpr size_t kProtoListOptionalSensorData = 3u;
constexpr size_t kProtoListVerticesStartIndex =
    kMinNumProtos;  // Vertices appear after all other elements and may not
                    // exist.

// These are only used by the save command. Load reads the metadata file to
// get the file names.
constexpr char kFolderName[] = "vi_map";
constexpr char kFileNameMetadata[] = "metadata";
constexpr char kFileNameVertices[] = "vertices";
constexpr char kFileNameEdges[] = "edges";
constexpr char kFileNameMissions[] = "missions";
constexpr char kFileNameLandmarkIndex[] = "landmark_index";
constexpr char kFileNameOptionalSensorData[] = "optional_sensor_data";

constexpr char kYamlSensorsFilename[] = "sensors.yaml";

const std::vector<std::string> kMinimumVIMapProtoFiles = {
    kFileNameMissions, kFileNameEdges, kFileNameLandmarkIndex,
    kFileNameOptionalSensorData};

}  // namespace internal

bool getListOfExistingMapFiles(
    const std::string& map_folder, std::string* sensors_yaml_filepath,
    std::vector<std::string>* list_of_resource_filenames,
    std::vector<std::string>* list_of_map_proto_filepaths);

bool loadMapFromFolder(const std::string& folder_path, vi_map::VIMap* map);

}  // namespace deprecated
}  // namespace serialization
}  // namespace vi_map

#endif  // VI_MAP_VI_MAP_SERIALIZATION_DEPRECATED_H_
