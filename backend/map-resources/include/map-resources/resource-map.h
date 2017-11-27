#ifndef MAP_RESOURCES_RESOURCE_MAP_H_
#define MAP_RESOURCES_RESOURCE_MAP_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <aslam/common/reader-writer-lock.h>

#include "map-resources/resource-common.h"
#include "map-resources/resource-loader.h"

#include "map-resources/resource_metadata.pb.h"

namespace resource_info {
namespace proto {
class ResourceInfoMap;
}  // proto
}  // resource_info

namespace backend {

class ResourceMap {
  friend class ResourceMapTest;

 public:
  ResourceMap();

  // The map folder will be stored in common::MapBase and is also used to
  // initialize the map resource folder in this class.
  explicit ResourceMap(const std::string& map_folder);

  explicit ResourceMap(const metadata::proto::MetaData& metadata_proto);

  void deepCopyFrom(const ResourceMap& other);

  void mergeFromMap(const ResourceMap& source_map);

  // Overwrites the meta data with the content of the meta data proto. If the
  // map already has resources associated with it, this can lead to a
  // inconsistent state of the resource system.
  void setMetaDataFromProto(const metadata::proto::MetaData& metadata_proto);

  // Uniquely identifies a resource folder, either by corresponding to one of
  // the special indices below or to an element of the external resource folder
  // vector.
  typedef int ResourceFolderIndex;
  // Corresponds to the resource folder that is part of the map folder.
  static constexpr ResourceFolderIndex kMapResourceFolder = -1;
  // Corresponds to an unknown folder.
  static constexpr ResourceFolderIndex kUnknownResourceFolder = -2;

  struct MetaData {
    MetaData& operator=(const MetaData& other_meta_data);

    explicit MetaData(const std::string& map_folder);

    explicit MetaData(const metadata::proto::MetaData& metadata_proto);

    void serialize(metadata::proto::MetaData* metadata_proto) const;

    std::string map_folder;
    std::string map_description;

    // Determines which folder is used for new resources. Default is set to the
    // map resource folder.
    ResourceFolderIndex resource_folder_in_use;

    // Subfolder of the map folder for resources.
    const std::string kResourceFolderName = "/resources/";

    // Points to the resource folder that is part of the map folder.
    std::string map_resource_folder;
    // List of external resource folders.
    std::vector<std::string> external_resource_folders;
  };

  void migrateAllResourcesToFolder(
      const std::string& resource_folder, const bool move_resources);
  void migrateAllResourcesToMapResourceFolder(const bool move_resources);

  // Use this to change the map folder, e.g. when saving a map to a new folder
  // or when you simply want to change to a different map folder. Set the map
  // folder and with it the resource map folder.
  // * If this is called on a map that already has a map folder and
  //   adapt_resource_references is set to true, we search for all resources
  //   that are associated with the old map resource folder and turn the old
  //   folder into an external resource folder, such that we don't lose any
  //   resources. However, it does not check if the map resource folder is
  //   already part of the external folders and therefore it will keep both
  //   folder references. Use this when you want to change the map folder while
  //   the resources remain in the old location on the file system.
  // * If adapt_resource_references is set to false, the resource references
  //   will not be updated, but will still be located in the old map folder on
  //   the file system. Use this to update a map folder when the map has been
  //   moved together with the resource folder.
  void setMapFolder(const std::string& map_folder);
  void setMapFolder(
      const std::string& map_folder, const bool adapt_resource_references);

  // Use this only when loading a map from the file system, this will simply
  // replace the map folder (and resource folder) inside the meta data struct.
  // Since we are loading from this folder, we assume it already exists, hence
  // this function will check-fail if it doesn't. It will not modify any of the
  // resource references, they will now just simply point to the resource folder
  // of the map folder where you loaded the map from.
  void replaceMapFolder(const std::string& new_map_folder);

  void getMapFolder(std::string* map_folder) const;
  const std::string& getMapFolder() const;

  bool hasMapFolder() const;

  MetaData getMetaDataCopy() const;

  // Returns the resource folder associated with the current map folder of this
  // map. Will crash if the map folder is not set in MapBase.
  void getMapResourceFolder(std::string* default_folder) const;

  // Returns the list of external resource folders that are registered for this
  // map.
  void getExternalResourceFolders(
      std::vector<std::string>* external_folders) const;

  // Use the provided resource folder as default folder to store new resources.
  // The folder will be compared against the other already registered external
  // folders and will only be registered if it is not known yet.
  void useExternalResourceFolder(const std::string& resource_folder);

  // Use the map resource folder as default folder to store new resources.
  void useMapResourceFolder();

  // Returns the path to the resource folder that is currently selected to be in
  // use.
  void getResourceFolderInUse(std::string* default_folder) const;

  // Get a copy of the current cache statistic state.
  CacheStatistic getResourceCacheStatisticCopy() const;

  size_t getNumResourceCacheMiss(const ResourceType& type) const;
  size_t getNumResourceCacheHits(const ResourceType& type) const;

  void serializeResourceInfo(
      resource_info::proto::ResourceInfoMap* resource_info_map_proto) const;
  void serializeResourceInfo(
      resource_info::proto::ResourceInfoMap* resource_info_map_proto,
      const size_t start_idx,
      const size_t num_resource_infos_to_serialize) const;
  void deserializeResourceInfo(
      const resource_info::proto::ResourceInfoMap& resource_info_map_proto);

  void serializeMetaData(metadata::proto::MetaData* metadata) const;

  size_t numResources() const;
  size_t numResourcesOfType(const ResourceType& type) const;

  std::string printCacheStatistics() const;
  void printCacheStatisticsToLog(int verbosity) const;

  std::string printResourceStatistics() const;
  void printResourceStatisticsToLog(int verbosity) const;

  // Clean up the resource folder system by removing duplicate entries and
  // merging them. After cleanup it will perform a check on all resources.
  void cleanupResourceFolders();

  // Check if all resource files are present. Does not check the content of the
  // resource files.
  bool checkResourceFileSystem() const;

 protected:
  // Check if the resource file is present and attempt to load it to verify its
  // content.
  template <typename DataType>
  bool checkResource(const ResourceId& id, const ResourceType& type) const;

  // Store a new resource to the current default folder and get a new resource
  // id.
  template <typename DataType>
  void addResource(
      const ResourceType& type, const DataType& resource, ResourceId* id);

  // Store a new resource to a specified folder and get a new resource id.
  template <typename DataType>
  void addResource(
      const ResourceType& type, const DataType& resource,
      const std::string& folder, ResourceId* id);

  // Store a new resource to the current default folder with a specific resource
  // id.
  template <typename DataType>
  void addResource(
      const ResourceType& type, const DataType& resource, const ResourceId& id);

  template <typename DataType>
  bool getResource(
      const ResourceId& id, const ResourceType& type, DataType* resource) const;

  // Returns true if the resource was successfully deleted, false if it didn't
  // exist in the first place. By default it also deletes the file on the
  // file-system.
  template <typename DataType>
  bool deleteResource(const ResourceId& id, const ResourceType& type);
  template <typename DataType>
  bool deleteResource(
      const ResourceId& id, const ResourceType& type,
      const bool keep_resource_file);

  // Delete old resource and add a new one. Doesn't fail if the old resource
  // doesn't exist, but returns false.
  template <typename DataType>
  bool replaceResource(
      const ResourceId& id, const ResourceType& type, const DataType& resource);

 private:
  struct ResourceInfo {
    ResourceInfo() {}
    ResourceFolderIndex folder_idx = kUnknownResourceFolder;
  };

  // Store a new resource to the provided resource folder. If an empty folder is
  // provided it will use the current resource folder.
  template <typename DataType>
  void addResourceImpl(
      const ResourceType& type, const DataType& resource,
      const std::string& resource_folder, const ResourceId& id);

  // Provided with a folder index it returns the corresponding folder.
  void getFolderFromIndex(
      const ResourceMap::ResourceFolderIndex& index, std::string* folder) const;
  ResourceMap::ResourceFolderIndex getIndexFromFolder(
      const std::string& folder) const;
  ResourceMap::ResourceFolderIndex registerNewExternalFolder(
      const std::string& folder);

  bool resourceFileExists(const ResourceId& id, const ResourceType& type) const;

  MetaData meta_data_;

  typedef std::unordered_map<ResourceId, ResourceInfo> ResourceInfoMap;

  std::vector<ResourceInfoMap> resource_info_map_;

  ResourceLoader resource_loader_;

  mutable aslam::ReaderWriterMutex resource_mutex_;
};

}  // namespace backend

#include "map-resources/resource-map-inl.h"

#endif  // MAP_RESOURCES_RESOURCE_MAP_H_
