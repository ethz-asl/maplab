#ifndef MAP_RESOURCES_RESOURCE_MAP_SERIALIZATION_H_
#define MAP_RESOURCES_RESOURCE_MAP_SERIALIZATION_H_

#include <string>
#include <vector>

#include <maplab-common/map-manager-config.h>

#include "map-resources/resource-map.h"

namespace backend {

namespace resource_map_serialization {

namespace internal {

const std::string kFileNameResourceInfo = "resource_info";
const std::string kFileNameMapMetadata = "metadata";
const std::vector<std::string> kResourceFiles = {kFileNameMapMetadata,
                                                 kFileNameResourceInfo};

}  // namespace internal

bool getListOfExistingMapFiles(
    const std::string& map_folder, std::vector<std::string>* list_of_map_files);
bool hasMapOnFileSystem(const std::string& folder_path);

bool loadMapFromFolder(const std::string& map_folder, ResourceMap* map);
bool loadMetaDataFromFolder(
    const std::string& folder_path, backend::ResourceMap* map);
bool saveMapToFolder(
    const std::string& folder_path, const SaveConfig& config,
    backend::ResourceMap* map);

}  // namespace resource_map_serialization

}  // namespace backend

#endif  // MAP_RESOURCES_RESOURCE_MAP_SERIALIZATION_H_
