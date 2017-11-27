#ifndef VI_MAP_DEPRECATED_VI_MAP_SERIALIZATION_DEPRECATED_H_
#define VI_MAP_DEPRECATED_VI_MAP_SERIALIZATION_DEPRECATED_H_

#include <string>
#include <vector>

#include <maplab-common/map-manager-config.h>
#include <maplab-common/network-common.h>
#include <sensors/imu.h>

#include "vi-map/vi_map_deprecated.pb.h"

namespace vi_map {

class VIMap;

namespace serialization_deprecated {

namespace internal {

// Minimum number of protos when deserializing from network/file system. The
// minimum number of protos includes edges, missions, landmark_index and
// other_fields. (A vertices proto is only generated when there are vertices.)
constexpr size_t kMinNumProtos = 4u;

constexpr size_t kProtoListMissionsIndex = 0u;
constexpr size_t kProtoListEdgesIndex = 1u;
constexpr size_t kProtoListLandmarkIndexIndex = 2u;
constexpr size_t kProtoListOtherFieldsIndex = 3u;
constexpr size_t kProtoListVerticesStartIndex =
    kMinNumProtos;  // Vertices appear after all other elements and may not
                    // exist.

constexpr char kFolderName[] = "vi_map";
constexpr char kFileNameVertices[] = "vertices";
constexpr char kFileNameEdges[] = "edges";
constexpr char kFileNameMissions[] = "missions";
constexpr char kFileNameLandmarkIndex[] = "landmark_index";
constexpr char kFileNameOtherFields[] = "other_fields";
const std::vector<std::string> kMinimumVIMapFiles = {
    kFileNameMissions, kFileNameEdges, kFileNameLandmarkIndex,
    kFileNameOtherFields};

size_t numberOfProtos(const VIMap& map, const backend::SaveConfig& save_config);

}  // namespace internal

void serializeImuSigmas(
    const vi_map::ImuSigmas& imu_sigmas,
    vi_map_deprecated::proto::ImuSigmas* proto);
void deserializeImuSigmas(
    const vi_map_deprecated::proto::ImuSigmas& proto,
    vi_map::ImuSigmas* imu_sigmas);

void deserializeCompleteMap(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeVertices(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeEdges(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeMissionsAndBaseFrames(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeLandmarkIndex(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map);
void deserializeOtherFields(
    const vi_map_deprecated::proto::VIMap& proto, vi_map::VIMap* map);

// The order of the protos for these functions is:
// missions, edges, landmark_index, other_fields, (vertices0, ...)
void deserializeFromListOfProtos(
    const std::vector<vi_map_deprecated::proto::VIMap>& list_of_protos,
    vi_map::VIMap* map);

// ============================
// INTERACTION WITH FILE SYSTEM
// ============================
inline std::string getSubFolderName() {
  return internal::kFolderName;
}
bool getListOfExistingMapFiles(
    const std::string& map_folder, std::vector<std::string>* list_of_map_files);
// bool hasMapOnFileSystem(const std::string& folder_path);
bool loadMapFromFolder(const std::string& map_folder, vi_map::VIMap* map);

}  // namespace serialization_deprecated

}  // namespace vi_map

#endif  // VI_MAP_DEPRECATED_VI_MAP_SERIALIZATION_DEPRECATED_H_
