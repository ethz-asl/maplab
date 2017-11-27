#include "map-resources/resource-map-serialization.h"

#include <string>

#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/proto-serialization-helper.h>

#include "map-resources/resource_info_map.pb.h"
#include "map-resources/resource_metadata.pb.h"

namespace backend {

namespace resource_map_serialization {

bool getListOfExistingMapFiles(
    const std::string& map_folder,
    std::vector<std::string>* list_of_map_files) {
  CHECK_NOTNULL(list_of_map_files);
  if (!common::pathExists(map_folder)) {
    return false;
  }
  bool minimum_required_files_exist = true;
  std::string path_to_resource_map_file;
  for (const std::string& map_file : internal::kResourceFiles) {
    common::concatenateFolderAndFileName(
        map_folder, map_file, &path_to_resource_map_file);
    if (common::fileExists(path_to_resource_map_file)) {
      list_of_map_files->emplace_back(map_file);
    } else {
      minimum_required_files_exist = false;
    }
  }
  return minimum_required_files_exist;
}

bool hasMapOnFileSystem(const std::string& folder_path) {
  if (!common::pathExists(folder_path)) {
    return false;
  }
  std::vector<std::string> list_of_map_files;
  return getListOfExistingMapFiles(folder_path, &list_of_map_files);
}

bool loadMapFromFolder(
    const std::string& folder_path, backend::ResourceMap* map) {
  CHECK_NOTNULL(map);

  resource_info::proto::ResourceInfoMap resource_info_proto;
  if (!common::proto_serialization_helper::parseProtoFromFile(
          folder_path, internal::kFileNameResourceInfo, &resource_info_proto)) {
    return false;
  }
  map->deserializeResourceInfo(resource_info_proto);

  map->cleanupResourceFolders();

  return true;
}

bool loadMetaDataFromFolder(
    const std::string& folder_path, backend::ResourceMap* map) {
  CHECK_NOTNULL(map);

  metadata::proto::MetaData metadata_proto;
  constexpr bool kParseAsTextFormat = true;
  if (!common::proto_serialization_helper::parseProtoFromFile(
          folder_path, internal::kFileNameMapMetadata, &metadata_proto,
          kParseAsTextFormat)) {
    // No metadata available.
    return false;
  }

  map->setMetaDataFromProto(metadata_proto);
  return true;
}

bool saveMapToFolder(
    const std::string& folder_path, const SaveConfig& config,
    backend::ResourceMap* map) {
  map->cleanupResourceFolders();

  // Migrate resources if it is desired.
  switch (config.migrate_resources_settings) {
    case SaveConfig::MigrateResourcesSettings::kMigrateResourcesToMapFolder:
      map->migrateAllResourcesToMapResourceFolder(
          config.move_resources_when_migrating);
      break;
    case SaveConfig::MigrateResourcesSettings::
        kMigrateResourcesToExternalFolder:
      map->migrateAllResourcesToFolder(
          config.external_folder_for_migration,
          config.move_resources_when_migrating);
      break;
    case SaveConfig::MigrateResourcesSettings::kDontMigrateResourceFolder:
    default:
      // No action required.
      break;
  }

  metadata::proto::MetaData metadata_proto;
  map->serializeMetaData(&metadata_proto);
  constexpr bool kSerializeAsTextFormat = true;
  if (!common::proto_serialization_helper::serializeProtoToFile(
          folder_path, internal::kFileNameMapMetadata, metadata_proto,
          kSerializeAsTextFormat)) {
    return false;
  }

  resource_info::proto::ResourceInfoMap resource_info_proto;
  map->serializeResourceInfo(&resource_info_proto);
  if (!common::proto_serialization_helper::serializeProtoToFile(
          folder_path, internal::kFileNameResourceInfo, resource_info_proto)) {
    return false;
  }

  return true;
}

}  // namespace resource_map_serialization

}  // namespace backend
