#ifndef MAPLAB_COMMON_MAP_MANAGER_CONFIG_H_
#define MAPLAB_COMMON_MAP_MANAGER_CONFIG_H_

#include <string>

namespace backend {

struct SaveConfig {
  SaveConfig() {}

  enum class MigrateResourcesSettings {
    /// Leaves the resources in the current location.
    kDontMigrateResourceFolder,

    /// Migrates all resources to the map folder.
    kMigrateResourcesToMapFolder,

    /// Migrates all resources to a specified external folder.
    kMigrateResourcesToExternalFolder
  };

  /// If true, any pre-existing files on the disk will be overwritten.
  bool overwrite_existing_files = false;

  /// Determines if the resources will be migrating before the map is saved.
  MigrateResourcesSettings migrate_resources_settings =
      MigrateResourcesSettings::kDontMigrateResourceFolder;

  /// The external folder where all resources should be migrated to.
  std::string external_folder_for_migration = "";

  /// If set to true, the resource migration will move the resources and delete
  /// the resource files in the old location. If set to false, the resources
  /// are copied and will still exist in the old location in the file
  /// system.
  bool move_resources_when_migrating = false;

  /// Determines the number of vertices/edges that should be in a single proto.
  ///
  /// NOTE: If this value is set to high, the map cannot be read anymore because
  /// there is a maximum file size above which protobuf doesn't read the data
  /// anymore.
  static constexpr size_t kVerticesPerProtoFile = 200u;
  static constexpr size_t kEdgesPerProtoFile = 30000u;
};

static_assert(SaveConfig::kVerticesPerProtoFile > 0u, "");
static_assert(SaveConfig::kEdgesPerProtoFile > 0u, "");

}  // namespace backend

#endif  // MAPLAB_COMMON_MAP_MANAGER_CONFIG_H_
