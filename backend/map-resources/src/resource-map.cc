#include "map-resources/resource-map.h"

#include <aslam/common/reader-writer-lock.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>

#include "map-resources/resource_info_map.pb.h"
#include "map-resources/resource_metadata.pb.h"

namespace backend {

ResourceMap::ResourceMap()
    : meta_data_(""), resource_info_map_(kNumResourceTypes) {}

ResourceMap::ResourceMap(const std::string& map_folder)
    : meta_data_(map_folder), resource_info_map_(kNumResourceTypes) {
  // Create map and resource folder.
  if (!common::pathExists(meta_data_.map_resource_folder)) {
    common::createPath(meta_data_.map_resource_folder);
  }
}

ResourceMap::ResourceMap(const metadata::proto::MetaData& metadata_proto)
    : meta_data_(metadata_proto), resource_info_map_(kNumResourceTypes) {
  // Create map and resource folder.
  if (!common::pathExists(meta_data_.map_resource_folder)) {
    common::createPath(meta_data_.map_resource_folder);
  }
}

void ResourceMap::setMetaDataFromProto(
    const metadata::proto::MetaData& metadata_proto) {
  meta_data_ = ResourceMap::MetaData(metadata_proto);
}

void ResourceMap::deepCopyFrom(const ResourceMap& other) {
  aslam::ScopedWriteLock lock(&resource_mutex_);
  meta_data_.map_folder = other.meta_data_.map_folder;
  meta_data_.map_description = other.meta_data_.map_description;
  meta_data_.resource_folder_in_use = other.meta_data_.resource_folder_in_use;
  meta_data_.map_resource_folder = other.meta_data_.map_resource_folder;
  meta_data_.external_resource_folders =
      other.meta_data_.external_resource_folders;
  resource_info_map_ = other.resource_info_map_;
}

void ResourceMap::mergeFromMap(const ResourceMap& source_map) {
  aslam::ScopedWriteLock lock(&resource_mutex_);
  const ResourceFolderIndex num_external_folders_before =
      static_cast<ResourceFolderIndex>(
          meta_data_.external_resource_folders.size());

  // Add map folder of source map to external folders and copy other external
  // folders.
  meta_data_.external_resource_folders.push_back(
      source_map.meta_data_.map_resource_folder);
  meta_data_.external_resource_folders.insert(
      meta_data_.external_resource_folders.end(),
      source_map.meta_data_.external_resource_folders.begin(),
      source_map.meta_data_.external_resource_folders.end());

  // Loop through resource info and adjust folder index.
  for (size_t resource_type = 0u; resource_type < kNumResourceTypes;
       ++resource_type) {
    const size_t size_before = resource_info_map_[resource_type].size();
    for (const ResourceInfoMap::value_type& resource_info_map_value :
         source_map.resource_info_map_[resource_type]) {
      ResourceInfo resource_info = resource_info_map_value.second;
      // Resource folders from source map are in the back of the list.
      // - map folder        was:  -1     is now: num_external_folders_before
      // - external folder   was: >=0     is now: num_external_folders_before +
      // 1 + old value
      resource_info.folder_idx +=
          num_external_folders_before + static_cast<ResourceFolderIndex>(1);

      CHECK(
          resource_info_map_[resource_type]
              .emplace(resource_info_map_value.first, resource_info)
              .second);
    }
    CHECK_EQ(
        resource_info_map_[resource_type].size(),
        size_before + source_map.resource_info_map_[resource_type].size());
  }
}

ResourceMap::MetaData& ResourceMap::MetaData::operator=(
    const ResourceMap::MetaData& other_meta_data) {
  map_folder = other_meta_data.map_folder;
  map_description = other_meta_data.map_description;
  resource_folder_in_use = other_meta_data.resource_folder_in_use;
  map_resource_folder = other_meta_data.map_resource_folder;
  external_resource_folders = other_meta_data.external_resource_folders;
  return *this;
}

ResourceMap::MetaData::MetaData(const std::string& map_folder)
    : map_folder(map_folder),
      resource_folder_in_use(kMapResourceFolder),
      map_resource_folder(map_folder + kResourceFolderName) {}

ResourceMap::MetaData::MetaData(
    const metadata::proto::MetaData& metadata_proto) {
  CHECK(metadata_proto.has_map_folder());
  map_folder = metadata_proto.map_folder();
  CHECK(!map_folder.empty())
      << "metadata::proto::MetaData did not provide a map folder!";

  if (metadata_proto.has_map_description()) {
    map_description = metadata_proto.map_description();
  }

  CHECK(metadata_proto.has_map_resource_folder());
  CHECK(metadata_proto.has_resource_folder_in_use());

  map_resource_folder = metadata_proto.map_resource_folder();
  resource_folder_in_use =
      static_cast<ResourceFolderIndex>(metadata_proto.resource_folder_in_use());

  const size_t num_external_folders =
      metadata_proto.external_resource_folders_size();
  external_resource_folders.resize(num_external_folders);
  for (size_t i = 0u; i < num_external_folders; ++i) {
    external_resource_folders[i] = metadata_proto.external_resource_folders(i);
  }
}

void ResourceMap::MetaData::serialize(
    metadata::proto::MetaData* metadata_proto) const {
  CHECK_NOTNULL(metadata_proto);
  CHECK(!map_folder.empty());
  metadata_proto->set_map_folder(map_folder);
  if (!map_description.empty()) {
    metadata_proto->set_map_description(map_description);
  }

  metadata_proto->set_map_resource_folder(map_resource_folder);
  metadata_proto->set_resource_folder_in_use(
      static_cast<int>(resource_folder_in_use));
  const size_t num_external_folders = external_resource_folders.size();
  for (size_t i = 0u; i < num_external_folders; ++i) {
    metadata_proto->add_external_resource_folders(external_resource_folders[i]);
  }
}

void ResourceMap::replaceMapFolder(const std::string& new_map_folder) {
  CHECK(!new_map_folder.empty());

  // Create path. This is necessary because a map should always be associated
  // with a real folder at any time and because getRealPath does not work
  // otherwise.
  CHECK(common::createPath(new_map_folder));
  const std::string real_new_folder_path = common::getRealPath(new_map_folder);
  meta_data_.map_folder = real_new_folder_path;
  meta_data_.map_resource_folder =
      real_new_folder_path + meta_data_.kResourceFolderName;

  if (!common::pathExists(meta_data_.map_resource_folder)) {
    CHECK(common::createPath(meta_data_.map_resource_folder))
        << "Unable to create resource folder for map: " << real_new_folder_path;
  }
}

void ResourceMap::setMapFolder(const std::string& map_folder) {
  constexpr bool kAdaptResourceReferences = true;
  setMapFolder(map_folder, kAdaptResourceReferences);
}

void ResourceMap::setMapFolder(
    const std::string& map_folder, const bool adapt_resource_references) {
  aslam::ScopedWriteLock lock(&resource_mutex_);
  CHECK(!map_folder.empty());

  // Create path. This is necessary because a map should always be associated
  // with a real folder at any time and because getRealPath does not work
  // otherwise.
  CHECK(common::createPath(map_folder));

  // Get the full path to this folder, this is required because if we save this
  // map to another folder, the current resource folder becomes an external
  // resource folder. This means that if we run maplab from another root folder
  // and load map B, it would fail to find the correct path.
  // Example:
  // load --map_folder test/map_A
  //      => results in: map folder: test/map_A)
  // save --map_folder test/new_map_B
  //      => results in map_folder: test/new_map_B, external folders: test/map_A
  // <close and open maplab inside test/ folder>
  // load --map_folder new_map_B
  //      => results in map_folder: new_map_B, external folders: test/map_A
  //      => external folder cannot be reached anymore.
  const std::string real_folder_path = common::getRealPath(map_folder);

  // If there was no map folder set before, just simply replace the map folder.
  if (meta_data_.map_folder.empty()) {
    replaceMapFolder(real_folder_path);
    return;
  }

  const std::string old_map_resource_folder = meta_data_.map_resource_folder;

  meta_data_.map_folder = real_folder_path;
  meta_data_.map_resource_folder =
      real_folder_path + meta_data_.kResourceFolderName;

  VLOG(2) << "Creating map (resource) folder: "
          << meta_data_.map_resource_folder;
  CHECK(common::createPath(meta_data_.map_resource_folder));

  VLOG(2) << "Setting the map resource folder from: " << old_map_resource_folder
          << " to: " << meta_data_.map_resource_folder;

  if (adapt_resource_references && !old_map_resource_folder.empty()) {
    if (!common::pathExists(old_map_resource_folder)) {
      LOG(FATAL) << "Cannot adapt the resource references to the new map "
                 << "folder, because the previous map resource "
                 << "folder doesn't exist (anymore)! New resource folder: "
                 << meta_data_.map_resource_folder
                 << ", previous resource folder: " << old_map_resource_folder;
    }

    // Adapt the resources so that the old folder is still referenced.
    if (common::isSameRealPath(
            old_map_resource_folder, meta_data_.map_resource_folder)) {
      // If old and new folder are the same, we don't need to change anything.
      VLOG(2) << "New and old map folder are the same, new :"
              << meta_data_.map_resource_folder
              << " old: " << old_map_resource_folder;
      return;
    }

    VLOG(2) << "Old map resource folder (" << old_map_resource_folder
            << ") is turned into an external resource "
            << "folder and all resource references are adapted...";

    const ResourceFolderIndex new_index =
        registerNewExternalFolder(old_map_resource_folder);

    // Loop over all resources in the resource maps and fix the folder index.
    size_t total_num_external_resources = 0u;
    size_t num_new_external_resources = 0u;
    for (size_t type_idx = 0u; type_idx < kNumResourceTypes; ++type_idx) {
      ResourceInfoMap& info_map = resource_info_map_.at(type_idx);
      ResourceInfoMap::iterator it = info_map.begin();
      for (; it != info_map.end(); ++it) {
        ResourceFolderIndex& folder_idx = it->second.folder_idx;
        if (folder_idx == kMapResourceFolder) {
          folder_idx = new_index;
          ++num_new_external_resources;
        }

        if (folder_idx != kMapResourceFolder) {
          ++total_num_external_resources;
        }
      }
    }

    LOG_IF(WARNING, num_new_external_resources > 0u)
        << num_new_external_resources
        << " additional maps resources have been linked to an external "
        << "resource folder! "
        << "Use --copy_resources_to_map_folder and "
        << "--move_resources_to_map_folder to copy/move all the map "
        << "resources to the map folder of the new map.";

    LOG_IF(WARNING, total_num_external_resources > 0u)
        << "There are a total of " << total_num_external_resources
        << " external resources attached to this map. Make sure these "
        << "external resource folder paths remain valid, otherwise you need "
        << "to modify the metadata file accordingly before loading the map.";
  }
}

ResourceMap::ResourceFolderIndex ResourceMap::getIndexFromFolder(
    const std::string& folder) const {
  CHECK(!folder.empty());

  if (!meta_data_.map_folder.empty() &&
      common::isSameRealPath(folder, meta_data_.map_resource_folder)) {
    return kMapResourceFolder;
  } else {
    ResourceFolderIndex folder_idx = 0u;
    for (const std::string& known_folder :
         meta_data_.external_resource_folders) {
      if (common::isSameRealPath(known_folder, folder)) {
        return folder_idx;
      }
      ++folder_idx;
    }
  }
  VLOG(2) << "Return unknown for " << folder;
  return kUnknownResourceFolder;
}

void ResourceMap::getFolderFromIndex(
    const ResourceMap::ResourceFolderIndex& index, std::string* folder) const {
  CHECK_NOTNULL(folder)->clear();
  CHECK_NE(index, kUnknownResourceFolder);

  if (index == kMapResourceFolder) {
    CHECK(!meta_data_.map_folder.empty())
        << "Cannot determine default resource folder because no map folder was "
        << "specified!";
    *folder = meta_data_.map_resource_folder;
  } else if (
      index >= 0 &&
      index < static_cast<ResourceFolderIndex>(
                  meta_data_.external_resource_folders.size())) {
    *folder = meta_data_.external_resource_folders[index];
  } else {
    LOG(FATAL) << "Resource folder index " << index
               << " does not correspond to any resource folder.";
  }
  CHECK(!folder->empty());
}

void ResourceMap::useExternalResourceFolder(
    const std::string& resource_folder) {
  CHECK(!resource_folder.empty());
  aslam::ScopedWriteLock lock(&resource_mutex_);

  VLOG(5) << "Setting external folder as default resource folder: "
          << resource_folder;

  CHECK(common::createPath(resource_folder))
      << "Unable to create resource folder at " << resource_folder;

  // If we already know this folder, we don't need to register it again.
  ResourceFolderIndex existing_folder_idx = getIndexFromFolder(resource_folder);
  if (existing_folder_idx == kUnknownResourceFolder) {
    meta_data_.resource_folder_in_use =
        registerNewExternalFolder(resource_folder);
  } else {
    meta_data_.resource_folder_in_use = existing_folder_idx;
  }
}

void ResourceMap::useMapResourceFolder() {
  aslam::ScopedWriteLock lock(&resource_mutex_);
  CHECK(!meta_data_.map_folder.empty());
  CHECK(!meta_data_.map_resource_folder.empty());
  meta_data_.resource_folder_in_use = kMapResourceFolder;
}

void ResourceMap::migrateAllResourcesToFolder(
    const std::string& target_resource_folder, const bool move_resources) {
  CHECK(!target_resource_folder.empty());
  aslam::ScopedWriteLock lock(&resource_mutex_);
  CHECK(!meta_data_.map_folder.empty());
  CHECK(!meta_data_.map_resource_folder.empty());
  CHECK(common::createPath(target_resource_folder))
      << "Unable to create resource folder at " << target_resource_folder;

  VLOG(1) << "Migrating all resources to folder: " << target_resource_folder;

  // Check if we already know this folder.
  bool is_known_folder = false;
  bool is_map_folder = false;
  ResourceFolderIndex target_folder_idx =
      getIndexFromFolder(target_resource_folder);
  if (target_folder_idx != kUnknownResourceFolder) {
    is_known_folder = true;
    if (target_folder_idx == kMapResourceFolder) {
      is_map_folder = true;
    }
  }

  // Loop over all resources in the resource maps and move them to the new
  // folder if necessary.
  for (size_t type_idx = 0u; type_idx < kNumResourceTypes; ++type_idx) {
    ResourceInfoMap& info_map = resource_info_map_.at(type_idx);

    ResourceInfoMap::iterator it = info_map.begin();
    for (; it != info_map.end(); ++it) {
      ResourceFolderIndex& folder_idx = it->second.folder_idx;
      const bool resource_already_in_target_folder =
          is_known_folder && folder_idx == target_folder_idx;
      if (!resource_already_in_target_folder) {
        // Find previous folder for this resource.
        std::string old_folder;
        getFolderFromIndex(folder_idx, &old_folder);
        CHECK(common::pathExists(old_folder))
            << "Cannot migrate resources, previous resource folder doesn't "
            << "exist (anmore)! Folder: " << old_folder;

        // Move/copy the resource and update the folder index.
        resource_loader_.migrateResource(
            it->first, static_cast<ResourceType>(type_idx), old_folder,
            target_resource_folder, move_resources);
      }

      // If the new folder is the default folder, we need to set the appropriate
      // folder idx.
      folder_idx = is_map_folder ? kMapResourceFolder : 0u;
    }
  }

  // Delete all previous external folder references.
  meta_data_.external_resource_folders.clear();

  // Update resource folder settings.
  meta_data_.resource_folder_in_use = is_map_folder ? kMapResourceFolder : 0u;
  if (!is_map_folder) {
    meta_data_.external_resource_folders.push_back(target_resource_folder);
  }
}

void ResourceMap::migrateAllResourcesToMapResourceFolder(
    const bool move_resources) {
  CHECK(!meta_data_.map_folder.empty())
      << "Cannot determine default resource folder because no map folder was "
      << "specified!";
  migrateAllResourcesToFolder(meta_data_.map_resource_folder, move_resources);
}

ResourceMap::ResourceFolderIndex ResourceMap::registerNewExternalFolder(
    const std::string& external_folder) {
  CHECK(!external_folder.empty());
  CHECK_EQ(getIndexFromFolder(external_folder), kUnknownResourceFolder);
  CHECK(!meta_data_.map_folder.empty());
  CHECK(common::pathExists(external_folder))
      << "New external resource folder does not exist! Folder: "
      << external_folder;
  CHECK(common::pathExists(meta_data_.map_resource_folder))
      << "Map resource folder does not exist! Folder: "
      << meta_data_.map_resource_folder;
  CHECK(
      !common::isSameRealPath(external_folder, meta_data_.map_resource_folder));

  for (const std::string& other_external_folders :
       meta_data_.external_resource_folders) {
    CHECK(!common::isSameRealPath(external_folder, other_external_folders));
  }

  meta_data_.external_resource_folders.push_back(external_folder);
  return meta_data_.external_resource_folders.size() - 1u;
}

void ResourceMap::getMapFolder(std::string* map_folder) const {
  CHECK_NOTNULL(map_folder);
  aslam::ScopedReadLock lock(&resource_mutex_);
  CHECK(!meta_data_.map_folder.empty()) << "No map folder is set, use "
                                        << "hasMapFolder() to check before "
                                        << "calling this function!";
  *map_folder = meta_data_.map_folder;
}

const std::string& ResourceMap::getMapFolder() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  CHECK(!meta_data_.map_folder.empty()) << "No map folder is set, use "
                                        << "hasMapFolder() to check before "
                                        << "calling this function!";
  return meta_data_.map_folder;
}

bool ResourceMap::hasMapFolder() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  return !meta_data_.map_folder.empty();
}

void ResourceMap::getMapResourceFolder(
    std::string* map_resource_folder_ptr) const {
  CHECK_NOTNULL(map_resource_folder_ptr);
  aslam::ScopedReadLock lock(&resource_mutex_);
  *map_resource_folder_ptr = meta_data_.map_resource_folder;
}

ResourceMap::MetaData ResourceMap::getMetaDataCopy() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  return meta_data_;
}

void ResourceMap::getResourceFolderInUse(std::string* default_folder) const {
  CHECK_NOTNULL(default_folder);
  aslam::ScopedReadLock lock(&resource_mutex_);
  getFolderFromIndex(meta_data_.resource_folder_in_use, default_folder);
}

void ResourceMap::getExternalResourceFolders(
    std::vector<std::string>* external_folders) const {
  CHECK_NOTNULL(external_folders)->clear();
  aslam::ScopedReadLock lock(&resource_mutex_);
  *external_folders =
      std::vector<std::string>(meta_data_.external_resource_folders);
}

CacheStatistic ResourceMap::getResourceCacheStatisticCopy() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  return resource_loader_.getCacheStatistic();
}

size_t ResourceMap::getNumResourceCacheMiss(const ResourceType& type) const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  return resource_loader_.getCacheStatistic().getNumMiss(type);
}

size_t ResourceMap::getNumResourceCacheHits(const ResourceType& type) const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  return resource_loader_.getCacheStatistic().getNumHits(type);
}

size_t ResourceMap::numResources() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  size_t counter = 0u;
  for (const ResourceInfoMap& info_map : resource_info_map_) {
    counter += info_map.size();
  }
  return counter;
}

size_t ResourceMap::numResourcesOfType(const ResourceType& type) const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  return resource_info_map_[static_cast<size_t>(type)].size();
}

void ResourceMap::serializeResourceInfo(
    resource_info::proto::ResourceInfoMap* resource_info_map_proto) const {
  CHECK_NOTNULL(resource_info_map_proto);
  const size_t num_resources = numResources();
  constexpr size_t kStartIdx = 0u;
  serializeResourceInfo(resource_info_map_proto, kStartIdx, num_resources);
}

void ResourceMap::serializeResourceInfo(
    resource_info::proto::ResourceInfoMap* resource_info_map_proto,
    const size_t start_idx,
    const size_t num_resource_infos_to_serialize) const {
  CHECK_NOTNULL(resource_info_map_proto);
  if (num_resource_infos_to_serialize == 0u) {
    return;
  }
  aslam::ScopedReadLock lock(&resource_mutex_);

  size_t global_idx = 0u;
  size_t global_end_idx =
      std::min(start_idx + num_resource_infos_to_serialize, numResources());

  for (size_t resource_type = 0u; resource_type < resource_info_map_.size();
       ++resource_type) {
    const ResourceInfoMap& info_map = resource_info_map_[resource_type];
    const size_t num_type_resources = info_map.size();

    // Check if we need to start serializing in this resoure info map.
    if (start_idx < (global_idx + num_type_resources)) {
      for (const ResourceInfoMap::value_type& entry : info_map) {
        const bool reached_end_idx = global_idx >= global_end_idx;
        if (!reached_end_idx) {
          const bool start_serialization = global_idx >= start_idx;
          if (start_serialization) {
            resource_info_map_proto->add_resource_type(resource_type);
            entry.first.serialize(resource_info_map_proto->add_resource_id());
            resource_info::proto::ResourceInfo* resource_info_proto =
                resource_info_map_proto->add_resource_info();
            resource_info_proto->set_folder_idx(entry.second.folder_idx);
          }
          ++global_idx;
        } else {
          CHECK_LE(
              static_cast<size_t>(
                  resource_info_map_proto->resource_info_size()),
              num_resource_infos_to_serialize);
          return;
        }
      }
    } else {
      // Advance to next resource type.
      global_idx += num_type_resources;
    }
  }
  CHECK_LE(
      static_cast<size_t>(resource_info_map_proto->resource_info_size()),
      num_resource_infos_to_serialize);
}

void ResourceMap::deserializeResourceInfo(
    const resource_info::proto::ResourceInfoMap& resource_info_map_proto) {
  CHECK_GE(kNumResourceTypes, resource_info_map_.size());
  aslam::ScopedWriteLock lock(&resource_mutex_);

  const int num_resource_entries = resource_info_map_proto.resource_info_size();
  CHECK_EQ(num_resource_entries, resource_info_map_proto.resource_id_size());
  CHECK_EQ(num_resource_entries, resource_info_map_proto.resource_type_size());

  for (int info_idx = 0; info_idx < num_resource_entries; ++info_idx) {
    const size_t resource_type =
        resource_info_map_proto.resource_type(info_idx);
    CHECK_LT(resource_type, kNumResourceTypes)
        << "ResourceMap contains unknown resource type: " << resource_type;

    ResourceInfoMap& info_type_map = resource_info_map_[resource_type];

    const resource_info::proto::ResourceInfo& resource_info_proto =
        resource_info_map_proto.resource_info(info_idx);
    ResourceInfo info;
    info.folder_idx = resource_info_proto.folder_idx();

    // Verify folder idx.
    std::string folder;
    getFolderFromIndex(info.folder_idx, &folder);
    CHECK(!folder.empty());

    ResourceId resource_id;
    resource_id.deserialize(resource_info_map_proto.resource_id(info_idx));
    CHECK(info_type_map.emplace(resource_id, info).second);
  }
}

void ResourceMap::serializeMetaData(metadata::proto::MetaData* metadata) const {
  CHECK_NOTNULL(metadata);
  aslam::ScopedReadLock lock(&resource_mutex_);
  meta_data_.serialize(metadata);
}

std::string ResourceMap::printCacheStatistics() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  return resource_loader_.getCacheStatistic().print();
}

std::string ResourceMap::printResourceStatistics() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  const std::string separator = "--------------+---------------------------";
  const std::string column_separator = "  |  ";
  const std::string title = " # resources  |  type";

  std::stringstream ss;
  ss << "Resource statistics:\n";

  ss << separator << std::endl << title << std::endl << separator << std::endl;
  for (size_t type_idx = 0u; type_idx < kNumResourceTypes; ++type_idx) {
    ss << std::setfill(' ') << std::setw(12)
       << resource_info_map_.at(type_idx).size() << column_separator
       << ResourceTypeNames[type_idx] << std::endl;
  }
  ss << separator << std::endl;
  return ss.str();
}

void ResourceMap::printCacheStatisticsToLog(int verbosity) const {
  getResourceCacheStatisticCopy().printToLog(verbosity);
}

void ResourceMap::printResourceStatisticsToLog(int verbosity) const {
  VLOG(verbosity) << printResourceStatistics();
}

bool ResourceMap::resourceFileExists(
    const ResourceId& id, const ResourceType& type) const {
  const size_t type_idx = static_cast<size_t>(type);
  const ResourceInfoMap& info_map = resource_info_map_.at(type_idx);
  const ResourceInfoMap::const_iterator it = info_map.find(id);
  if (it == info_map.end()) {
    LOG(ERROR) << "Resource of type " << ResourceTypeNames[type_idx]
               << " and id " << id.hexString()
               << " is not part of the ResourceMap!";
    return false;
  } else {
    std::string folder;
    getFolderFromIndex(it->second.folder_idx, &folder);
    return resource_loader_.resourceFileExists(id, type, folder);
  }
}

bool ResourceMap::checkResourceFileSystem() const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  bool all_files_exist = true;
  for (size_t type_idx = 0u; type_idx < kNumResourceTypes; ++type_idx) {
    const ResourceType type = static_cast<ResourceType>(type_idx);
    for (const ResourceInfoMap::value_type& info_entry :
         resource_info_map_.at(type_idx)) {
      if (!resourceFileExists(info_entry.first, type)) {
        LOG(ERROR) << "Could not find resource " << info_entry.first.hexString()
                   << " of type " << ResourceTypeNames[type_idx];
        all_files_exist = false;
      }
    }
  }
  return all_files_exist;
}

void ResourceMap::cleanupResourceFolders() {
  // Scoped lock is not covering the entire function to make sure we can do a
  // resource file system check afterwards.
  {
    aslam::ScopedWriteLock lock(&resource_mutex_);

    //. Check if there are folders that are not used by any resources.
    std::unordered_set<size_t> folders_in_use;
    for (size_t resource_type = 0u; resource_type < kNumResourceTypes;
         ++resource_type) {
      for (const ResourceInfoMap::value_type& resource_info_map_value :
           resource_info_map_[resource_type]) {
        const ResourceInfo& resource_info = resource_info_map_value.second;
        folders_in_use.insert(resource_info.folder_idx);
      }
    }

    // Store the new, cleaner version of the resource folder tracking variables.
    // Also keep a remapping from old folder indices to the new one to later
    // adapt all the resource entries.
    std::unordered_map<ResourceFolderIndex, ResourceFolderIndex>
        folder_index_remapping;
    std::vector<std::string> clean_external_resource_folders;
    std::string new_map_resource_folder = meta_data_.map_resource_folder;

    // Keeps track of resource folders that we've already merged.
    std::unordered_set<size_t> merged_folders;

    // // The map resource folder will map to the same index.
    const ResourceFolderIndex map_resource_folder_index = kMapResourceFolder;
    CHECK(
        folder_index_remapping
            .emplace(map_resource_folder_index, map_resource_folder_index)
            .second);

    const size_t num_ext_folders = meta_data_.external_resource_folders.size();
    for (size_t i = 0u; i < num_ext_folders; ++i) {
      // Discard folder if it was already merged or if it was never used.
      if (merged_folders.count(i) > 0u) {
        continue;
      }
      if (folders_in_use.count(i) == 0u) {
        continue;
      }

      std::string ext_folder_A = meta_data_.external_resource_folders[i];
      CHECK(common::pathExists(ext_folder_A))
          << "External resource folder doesn't exist (anymore)! Folder: "
          << ext_folder_A;

      if (common::isSameRealPath(
              ext_folder_A, meta_data_.map_resource_folder)) {
        folder_index_remapping[i] = kMapResourceFolder;

        VLOG(1)
            << "Found an external resource folders at index " << i
            << " that is identical to the map resource folder. It will be "
            << "deleted and the resource references adapted to point to the "
            << "map resource folder.";

        // Keep the shorter of the two paths.
        if (ext_folder_A.length() < new_map_resource_folder.length()) {
          new_map_resource_folder = ext_folder_A;
        }

        continue;
      }

      for (size_t j = i + 1u; j < num_ext_folders; ++j) {
        const std::string& ext_folder_B =
            meta_data_.external_resource_folders[j];
        CHECK(common::pathExists(ext_folder_B))
            << "External resource folder doesn't exist (anymore)! Folder: "
            << ext_folder_B;
        if (common::isSameRealPath(ext_folder_B, ext_folder_A)) {
          CHECK(
              folder_index_remapping
                  .emplace(j, clean_external_resource_folders.size())
                  .second);

          VLOG(1) << "Found identical external resource folders with indices "
                  << i << " and " << j << ". Merging them to index "
                  << clean_external_resource_folders.size();

          // Keep the shorter of the two paths.
          if (ext_folder_B.length() < ext_folder_A.length()) {
            ext_folder_A = ext_folder_B;
          }
          // Add this folder to the merged ones, to make sure we don't iterate
          // over it again.
          merged_folders.insert(j);
        }
      }

      // Store shortest version of ext_folder_A to the new set of external
      // resource folders.
      CHECK(
          folder_index_remapping
              .emplace(i, clean_external_resource_folders.size())
              .second);
      VLOG(1) << "External resource folders at index " << i
              << " will be mapped to index "
              << clean_external_resource_folders.size() << ".";

      clean_external_resource_folders.push_back(ext_folder_A);
    }

    // Assign clean resource folder variables.
    meta_data_.map_resource_folder = new_map_resource_folder;
    meta_data_.external_resource_folders = clean_external_resource_folders;

    // Adapt resource references.
    // Loop through resource info and adjust folder index.
    for (size_t resource_type = 0u; resource_type < kNumResourceTypes;
         ++resource_type) {
      for (ResourceInfoMap::value_type& resource_info_map_value :
           resource_info_map_[resource_type]) {
        ResourceInfo& resource_info = resource_info_map_value.second;

        CHECK_GT(folder_index_remapping.count(resource_info.folder_idx), 0u);

        const ResourceFolderIndex new_index =
            folder_index_remapping[resource_info.folder_idx];

        VLOG(3) << "Adapting resource from folder index "
                << resource_info.folder_idx << " to index " << new_index;

        resource_info.folder_idx = new_index;
      }
    }
  }

  // Make sure everything went well.
  checkResourceFileSystem();
}

}  // namespace backend
