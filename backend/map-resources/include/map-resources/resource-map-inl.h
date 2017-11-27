#ifndef MAP_RESOURCES_RESOURCE_MAP_INL_H_
#define MAP_RESOURCES_RESOURCE_MAP_INL_H_

#include <string>

#include <aslam/common/reader-writer-lock.h>

#include "map-resources/resource-common.h"
#include "map-resources/resource-map.h"

namespace backend {

template <typename DataType>
bool ResourceMap::getResource(
    const ResourceId& id, const ResourceType& type, DataType* resource) const {
  CHECK_NOTNULL(resource);
  aslam::ScopedWriteLock lock(&resource_mutex_);
  const ResourceInfoMap& info_map =
      resource_info_map_[static_cast<size_t>(type)];
  const ResourceInfoMap::const_iterator it = info_map.find(id);
  if (it == info_map.cend()) {
    return false;
  } else {
    std::string folder;
    getFolderFromIndex(it->second.folder_idx, &folder);

    resource_loader_.getResource<DataType>(id, type, folder, resource);
    return true;
  }
}

template <typename DataType>
void ResourceMap::addResource(
    const ResourceType& type, const DataType& resource, ResourceId* id) {
  CHECK_NOTNULL(id);
  aslam::ScopedWriteLock lock(&resource_mutex_);
  static const std::string kUseDefaultFolder = "";
  common::generateId(id);
  addResourceImpl(type, resource, kUseDefaultFolder, *id);
}

template <typename DataType>
void ResourceMap::addResource(
    const ResourceType& type, const DataType& resource,
    const std::string& resource_folder, ResourceId* id) {
  CHECK_NOTNULL(id);
  aslam::ScopedWriteLock lock(&resource_mutex_);
  common::generateId(id);
  addResourceImpl(type, resource, resource_folder, *id);
}

template <typename DataType>
void ResourceMap::addResource(
    const ResourceType& type, const DataType& resource, const ResourceId& id) {
  aslam::ScopedWriteLock lock(&resource_mutex_);
  static const std::string kUseDefaultFolder = "";
  addResourceImpl(type, resource, kUseDefaultFolder, id);
}

template <typename DataType>
void ResourceMap::addResourceImpl(
    const ResourceType& type, const DataType& resource,
    const std::string& resource_folder, const ResourceId& id) {
  CHECK(id.isValid());
  ResourceInfoMap& info_map = resource_info_map_[static_cast<size_t>(type)];
  auto it = info_map.emplace(id, ResourceInfo());
  CHECK(it.second) << "ResourceId collision!";
  ResourceInfo& info = it.first->second;

  // Use default resource folder if no folder was passed in.
  std::string folder_to_use;
  if (resource_folder.empty()) {
    info.folder_idx = meta_data_.resource_folder_in_use;
    getFolderFromIndex(info.folder_idx, &folder_to_use);
  } else {
    if (!common::pathExists(resource_folder)) {
      CHECK(common::createPath(resource_folder))
          << "Unable to create resource folder! Folder: " << resource_folder;
    }

    info.folder_idx = getIndexFromFolder(resource_folder);

    // If the folder is unknown so far, register it.
    if (info.folder_idx == kUnknownResourceFolder) {
      info.folder_idx = registerNewExternalFolder(resource_folder);
    }
    folder_to_use = resource_folder;
  }
  CHECK_NE(info.folder_idx, kUnknownResourceFolder);
  CHECK(!folder_to_use.empty());

  resource_loader_.addResource<DataType>(id, type, folder_to_use, resource);
}

template <typename DataType>
bool ResourceMap::deleteResource(
    const ResourceId& id, const ResourceType& type) {
  constexpr bool kKeepResourceFile = false;
  return deleteResource<DataType>(id, type, kKeepResourceFile);
}

template <typename DataType>
bool ResourceMap::deleteResource(
    const ResourceId& id, const ResourceType& type,
    const bool keep_resource_file) {
  aslam::ScopedWriteLock lock(&resource_mutex_);
  ResourceInfoMap& info_map = resource_info_map_[static_cast<size_t>(type)];
  const ResourceInfoMap::const_iterator it = info_map.find(id);
  if (it == info_map.cend()) {
    return false;
  }

  if (!keep_resource_file) {
    std::string folder;
    getFolderFromIndex(it->second.folder_idx, &folder);
    resource_loader_.deleteResource<DataType>(id, type, folder);
  }
  info_map.erase(it);
  return true;
}

template <typename DataType>
bool ResourceMap::replaceResource(
    const ResourceId& id, const ResourceType& type, const DataType& resource) {
  aslam::ScopedWriteLock lock(&resource_mutex_);
  const ResourceInfoMap& info_map =
      resource_info_map_[static_cast<size_t>(type)];
  const ResourceInfoMap::const_iterator it = info_map.find(id);

  // Check if resource exists, if not we just add it.
  if (it != info_map.cend()) {
    std::string folder;
    getFolderFromIndex(it->second.folder_idx, &folder);
    resource_loader_.replaceResource<DataType>(id, type, folder, resource);
    return true;
  } else {
    static const std::string kUseDefaultFolder = "";
    addResourceImpl(type, resource, kUseDefaultFolder, id);
    return false;
  }
}

template <typename DataType>
bool ResourceMap::checkResource(
    const ResourceId& id, const ResourceType& type) const {
  aslam::ScopedReadLock lock(&resource_mutex_);
  const ResourceInfoMap& info_map =
      resource_info_map_[static_cast<size_t>(type)];
  const ResourceInfoMap::const_iterator it = info_map.find(id);
  if (it == info_map.cend()) {
    LOG(ERROR) << "There is no entry in the resource system for id " << id;
    return false;
  } else {
    std::string folder;
    getFolderFromIndex(it->second.folder_idx, &folder);
    const bool resource_available =
        resource_loader_.checkResourceFile<DataType>(id, type, folder);
    if (!resource_available) {
      LOG(ERROR) << "Could not find resource " << id << " in folder " << folder;
    }
    return resource_available;
  }
}

}  // namespace backend

#endif  // MAP_RESOURCES_RESOURCE_MAP_INL_H_
