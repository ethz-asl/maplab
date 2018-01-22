#ifndef MAP_MANAGER_MAP_MANAGER_INL_H_
#define MAP_MANAGER_MAP_MANAGER_INL_H_

#include <chrono>    // NOLINT
#include <iostream>  // NOLINT
#include <memory>
#include <sstream>
#include <string>
#include <thread>  // NOLINT
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <aslam/common/reader-writer-lock.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/map-traits.h>
#include <maplab-common/proto-serialization-helper.h>
#include <maplab-common/text-formatting.h>

#include "map-manager/map-manager.h"
#include "map-manager/map-storage.h"

namespace backend {

template <typename MapType>
MapManager<MapType>::MapManager()
    : map_storage_(CHECK_NOTNULL(MapStorage<MapType>::getInstance())) {}

template <typename MapType>
MapType* MapManager<MapType>::getMapMutable(const std::string& key) {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  return map_storage_->getMapMutable(key);
}

template <typename MapType>
const MapType& MapManager<MapType>::getMap(const std::string& key) const {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  return map_storage_->getMap(key);
}

template <typename MapType>
typename MapManager<MapType>::MapWriteAccess
MapManager<MapType>::getMapWriteAccess(const std::string& key) {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  return map_storage_->getMapWriteAccess(key);
}

template <typename MapType>
typename MapManager<MapType>::MapReadAccess
MapManager<MapType>::getMapReadAccess(const std::string& key) const {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  return map_storage_->getMapReadAccess(key);
}

template <typename MapType>
bool MapManager<MapType>::hasMap(const std::string& key) const {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  return map_storage_->hasMap(key);
}

template <typename MapType>
bool MapManager<MapType>::isKeyValid(const std::string& key) const {
  return map_storage_->isKeyValid(key);
}

template <typename MapType>
std::string MapManager<MapType>::removeProhibitedCharactersFromKey(
    const std::string& key) const {
  return map_storage_->removeProhibitedCharactersFromKey(key);
}

template <typename MapType>
void MapManager<MapType>::getAllMapKeys(
    std::unordered_set<std::string>* all_map_keys_list) const {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  map_storage_->getAllMapKeys(all_map_keys_list);
}

template <typename MapType>
void MapManager<MapType>::getAllMapKeys(
    std::vector<std::string>* all_map_keys_list) const {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  map_storage_->getAllMapKeys(all_map_keys_list);
}

template <typename MapType>
size_t MapManager<MapType>::numberOfMaps() const {
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  return map_storage_->numberOfMaps();
}

template <typename MapType>
void MapManager<MapType>::addMap(
    const std::string& key, AlignedUniquePtr<MapType>& map) {  // NOLINT
  aslam::ScopedWriteLock lock(map_storage_->getContainerMutex());
  map_storage_->addMap(key, map);
}

template <typename MapType>
AlignedUniquePtr<MapType> MapManager<MapType>::releaseMap(
    const std::string& key) {
  std::unique_ptr<typename MapStorage<MapType>::MapAndMutex> map_and_mutex;
  {
    aslam::ScopedWriteLock lock(map_storage_->getContainerMutex());
    map_and_mutex = map_storage_->releaseMapAndMutex(key);
  }

  // Wait until map mutex is no longer in use anywhere.
  while (map_and_mutex->map_mutex.isInUse()) {
    constexpr size_t kSleepTimeMs = 50u;
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
  }
  return std::move(map_and_mutex->map);
}

template <typename MapType>
void MapManager<MapType>::deleteMap(const std::string& key) {
  releaseMap(key);
}

template <typename MapType>
void MapManager<MapType>::renameMap(
    const std::string& old_key, const std::string& new_key) {
  aslam::ScopedWriteLock lock(map_storage_->getContainerMutex());
  map_storage_->renameMap(old_key, new_key);
}

template <typename MapType>
void MapManager<MapType>::copyMap(
    const std::string& source_key, const std::string& target_key) {
  map_storage_->getContainerMutex()->acquireReadLock();
  CHECK(map_storage_->hasMap(source_key)) << "Source map key \"" << source_key
                                          << "\" doesn't exist.";
  MapReadAccess source_map = map_storage_->getMapReadAccess(source_key);
  map_storage_->getContainerMutex()->releaseReadLock();

  AlignedUniquePtr<MapType> target_map = aligned_unique<MapType>();
  traits<MapType>::deepCopy(*source_map, target_map.get());

  // Add copied map to storage.
  aslam::ScopedWriteLock lock(map_storage_->getContainerMutex());
  CHECK(!map_storage_->hasMap(target_key)) << "Target map key \"" << target_key
                                           << "\" already exists.";
  map_storage_->addMap(target_key, target_map);
}

template <typename MapType>
void MapManager<MapType>::mergeMaps(
    const std::string& source_key_merge_base,
    const std::string& source_key_merge_from) {
  CHECK(source_key_merge_base != source_key_merge_from)
      << "Cannot merge maps because the two map keys are identical (\""
      << source_key_merge_base << "\").";
  map_storage_->getContainerMutex()->acquireReadLock();
  for (const std::string& source_key :
       {std::ref(source_key_merge_base), std::ref(source_key_merge_from)}) {
    CHECK(map_storage_->hasMap(source_key)) << "Source map key \"" << source_key
                                            << "\" doesn't exist.";
  }
  MapWriteAccess source_map_merge_base =
      map_storage_->getMapWriteAccess(source_key_merge_base);
  MapReadAccess source_map_merge_from =
      map_storage_->getMapReadAccess(source_key_merge_from);
  map_storage_->getContainerMutex()->releaseReadLock();

  traits<MapType>::mergeTwoMaps(
      *source_map_merge_from, source_map_merge_base.get());
}

template <typename MapType>
void MapManager<MapType>::getMapFolder(
    const std::string& map_key, std::string* map_folder) const {
  CHECK_NOTNULL(map_folder)->clear();
  traits<MapType>::getMapFolder(*getMapReadAccess(map_key), map_folder);
}

template <typename MapType>
void MapManager<MapType>::setMapFolder(
    const std::string& map_key, const std::string& map_folder) {
  traits<MapType>::setMapFolder(map_folder, getMapWriteAccess(map_key).get());
}

template <typename MapType>
bool MapManager<MapType>::loadMapFromFolder(
    const std::string& folder_path, const std::string& key_in) {
  CHECK(!folder_path.empty());
  CHECK(!key_in.empty());
  if (!map_storage_->isKeyValid(key_in)) {
    LOG(ERROR) << "The key \"" << key_in << "\" is not a valid key.";
    return false;
  }
  AlignedUniquePtr<MapType> map = aligned_unique<MapType>();
  if (!traits<MapType>::loadFromFolder(folder_path, map.get())) {
    LOG(ERROR) << "Loading the map failed.";
    return false;
  }

  aslam::ScopedWriteLock lock(map_storage_->getContainerMutex());
  if (map_storage_->hasMap(key_in)) {
    LOG(ERROR) << "Map with the key \"" << key_in
               << "\" already exists in the storage!";
    return false;
  }
  map_storage_->addMap(key_in, map);

  return true;
}

template <typename MapType>
bool MapManager<MapType>::loadMapFromFolder(
    const std::string& folder_path, std::string* key_out) {
  CHECK(!folder_path.empty());

  // Get filename to use as key.
  std::string path_without_trailing_slash(folder_path);
  if (path_without_trailing_slash.back() == '/') {
    // Remove trailing slash as that might cause problems with
    // splitPathAndFilename().
    path_without_trailing_slash.erase(
        path_without_trailing_slash.end() - 1,
        path_without_trailing_slash.end());
  }
  std::string folder_path_without_key, filename;
  common::splitPathAndFilename(
      path_without_trailing_slash, &folder_path_without_key, &filename);
  CHECK(!filename.empty());
  filename = map_storage_->removeProhibitedCharactersFromKey(filename);

  if (key_out != nullptr) {
    *key_out = filename;
  }

  return loadMapFromFolder(folder_path, filename);
}

template <typename MapType>
bool MapManager<MapType>::loadMapFromFolder(const std::string& folder_path) {
  constexpr std::nullptr_t kDontWantGeneratedKey = nullptr;
  return loadMapFromFolder(folder_path, kDontWantGeneratedKey);
}

template <typename MapType>
bool MapManager<MapType>::loadAllMapsFromFolder(
    const std::string& folder_path) {
  constexpr std::nullptr_t kDontWantListOfKeys = nullptr;
  return loadAllMapsFromFolder(folder_path, kDontWantListOfKeys);
}

template <typename MapType>
void MapManager<MapType>::listAllMapsInFolder(
    const std::string& folder_path, std::vector<std::string>* map_list) {
  CHECK_NOTNULL(map_list)->clear();

  CHECK(!folder_path.empty());
  if (!common::pathExists(folder_path)) {
    LOG(ERROR) << "Folder \"" << folder_path << "\" doesn't exist.";
    return;
  }
  const std::string real_path = common::getRealPath(folder_path);

  std::vector<std::string> folder_list;  // Stores all found folders in path.
  common::getAllFilesAndFoldersInFolder(real_path, nullptr, &folder_list);

  for (std::string& sub_folder_path : folder_list) {
    // Check if the sub-folder is a valid map.
    if (hasMapOnFileSystem(sub_folder_path)) {
      map_list->emplace_back(sub_folder_path);
    }
  }
}

template <typename MapType>
bool MapManager<MapType>::loadAllMapsFromFolder(
    const std::string& folder_path, std::unordered_set<std::string>* new_keys) {
  CHECK(!folder_path.empty());
  if (new_keys != nullptr) {
    new_keys->clear();
  }

  if (!common::pathExists(folder_path)) {
    LOG(ERROR) << "Folder \"" << folder_path << "\" doesn't exist.";
    return false;
  }

  std::vector<std::string> map_list;  // Stores path to all maps to load.
  listAllMapsInFolder(folder_path, &map_list);

  std::vector<std::string> key_list;  // Stores keys for the maps to load.
  getDefaultMapKeys(map_list, &key_list);

  if (map_list.empty()) {
    LOG(ERROR) << "No maps found in \"" << folder_path
               << "\" that can be loaded.";
    return false;
  }

  std::stringstream maps_to_load_ss;
  maps_to_load_ss << "Found the following maps to load:\n";
  for (size_t i = 0u; i < map_list.size(); ++i) {
    maps_to_load_ss << "- " << common::formatText(
                                   key_list[i], common::FormatOptions::kBold)
                    << " from " << map_list[i] << "\n";
  }
  VLOG(1) << maps_to_load_ss.str() << "\n";

  aslam::ScopedWriteLock lock(map_storage_->getContainerMutex());
  std::unordered_set<std::string> key_set;
  for (const std::string& map_key : key_list) {
    if (!key_set.emplace(map_key).second) {
      LOG(ERROR) << "Found duplicate map key: " << map_key << " in folder "
                 << folder_path << ". No maps will be loaded.";
      return false;
    }
    if (map_storage_->hasMap(map_key)) {
      LOG(ERROR) << "No maps will be loaded because a map with key \""
                 << map_key << "\" already exists in the storage.";
      return false;
    }
  }

  if (new_keys != nullptr) {
    new_keys->insert(key_list.cbegin(), key_list.cend());
  }

  // Load and insert all maps.
  CHECK_EQ(map_list.size(), key_list.size());
  for (size_t i = 0u; i < map_list.size(); ++i) {
    const std::string& map_folder = map_list[i];
    const std::string& key_name = key_list[i];
    CHECK(!map_folder.empty());
    CHECK(!key_name.empty());
    CHECK(isKeyValid(key_name));
    CHECK(!map_storage_->hasMap(key_name));

    AlignedUniquePtr<MapType> map = aligned_unique<MapType>();
    CHECK(traits<MapType>::loadFromFolder(map_folder, map.get()))
        << "Loading map " << map_folder << " failed.";
    map_storage_->addMap(key_name, map);
    VLOG(1) << "Loaded map " << key_name;
  }
  return true;
}

template <typename MapType>
bool MapManager<MapType>::saveMapToFolder(
    const std::string& key, const std::string& folder_path) const {
  const SaveConfig config;
  return saveMapToFolder(key, folder_path, config);
}

template <typename MapType>
bool MapManager<MapType>::saveMapToFolder(
    const std::string& key, const std::string& folder_path,
    const SaveConfig& config) const {
  CHECK(!key.empty());
  CHECK(!folder_path.empty());

  map_storage_->getContainerMutex()->acquireReadLock();
  if (!map_storage_->hasMap(key)) {
    map_storage_->getContainerMutex()->releaseReadLock();
    LOG(ERROR) << "Map with key \"" << key << "\" doesn't exist.";
    return false;
  }
  MapWriteAccess map = map_storage_->getMapWriteAccess(key);
  map_storage_->getContainerMutex()->releaseReadLock();
  return traits<MapType>::saveToFolder(folder_path, config, map.get());
}

template <typename MapType>
bool MapManager<MapType>::saveMapToMapFolder(const std::string& key) const {
  const SaveConfig config;
  return saveMapToMapFolder(key, config);
}

template <typename MapType>
bool MapManager<MapType>::saveMapToMapFolder(
    const std::string& key, const SaveConfig& config) const {
  std::string map_folder;
  getMapFolder(key, &map_folder);
  CHECK(!map_folder.empty());
  return saveMapToFolder(key, map_folder, config);
}

template <typename MapType>
bool MapManager<MapType>::saveAllMapsToFolder(
    const std::string& folder_path) const {
  const SaveConfig config;
  return saveAllMapsToFolder(folder_path, config);
}

template <typename MapType>
bool MapManager<MapType>::saveAllMapsToFolder(
    const std::string& folder_path, const SaveConfig& config) const {
  // Create path if it doesn't exist.
  if (!folder_path.empty() && !common::pathExists(folder_path)) {
    if (!common::createPath(folder_path)) {
      LOG(ERROR) << "Could not create path to file!";
      return false;
    }
  }

  // Get all map keys.
  aslam::ScopedReadLock lock(map_storage_->getContainerMutex());
  std::unordered_set<std::string> all_map_keys_list;
  map_storage_->getAllMapKeys(&all_map_keys_list);

  if (all_map_keys_list.empty()) {
    LOG(ERROR) << "No maps stored that could be saved.";
    return false;
  }

  std::unordered_map<std::string, std::string> key_to_folder_map;

  // Check if all maps can be saved.
  for (const std::string& key : all_map_keys_list) {
    std::string complete_folder_path;
    if (folder_path.empty()) {
      // If the map gets saved into the map folder, there is no need to append
      // the key.
      typename common::Monitor<MapType>::ReadAccess map =
          map_storage_->getMapReadAccess(key);
      if (!traits<MapType>::hasMapFolder(*map)) {
        LOG(ERROR) << "Can't save map \"" << key
                   << "\" to map folder because it doesn't have a map folder "
                      "associated with it.";
        return false;
      }
      traits<MapType>::getMapFolder(*map, &complete_folder_path);
    } else {
      common::concatenateFolderAndFileName(
          folder_path, key, &complete_folder_path);
    }

    key_to_folder_map.emplace(key, complete_folder_path);

    common::concatenateFolderAndFileName(
        complete_folder_path, traits<MapType>::getSubFolderName(),
        &complete_folder_path);
    if (!config.overwrite_existing_files &&
        (common::pathExists(complete_folder_path) ||
         common::fileExists(complete_folder_path))) {
      LOG(ERROR) << "No maps will be saved because this folder \""
                 << complete_folder_path << "\" already contains a map!";
      return false;
    }
  }

  // Save all maps.
  for (const std::string& key : all_map_keys_list) {
    MapWriteAccess map = map_storage_->getMapWriteAccess(key);
    CHECK(
        traits<MapType>::saveToFolder(
            key_to_folder_map[key], config, map.get()));
  }

  return true;
}

template <typename MapType>
bool MapManager<MapType>::saveAllMapsToMapFolder() const {
  const SaveConfig config;
  return saveAllMapsToMapFolder(config);
}

template <typename MapType>
bool MapManager<MapType>::saveAllMapsToMapFolder(
    const SaveConfig& config) const {
  const std::string kUseMapFolder = "";
  return saveAllMapsToFolder(kUseMapFolder, config);
}

template <typename MapType>
bool MapManager<MapType>::hasMapOnFileSystem(
    const std::string& folder_path) const {
  return traits<MapType>::hasMapOnFileSystem(folder_path);
}

template <typename MapType>
bool MapManager<MapType>::getListOfExistingMapFiles(
    const std::string& folder_path,
    std::vector<std::string>* list_of_map_files) {
  CHECK_NOTNULL(list_of_map_files)->clear();
  CHECK(!folder_path.empty());

  CHECK(!folder_path.empty());
  if (!common::pathExists(folder_path)) {
    LOG(ERROR) << "Folder \"" << folder_path << "\" doesn't exist.";
    return false;
  }
  const std::string real_folder_path = common::getRealPath(folder_path);
  std::string map_folder_path_without_trailing_slash(real_folder_path);
  if (map_folder_path_without_trailing_slash.back() == '/') {
    map_folder_path_without_trailing_slash.erase(
        map_folder_path_without_trailing_slash.end() - 1);
  }
  CHECK(!map_folder_path_without_trailing_slash.empty());
  return traits<MapType>::getListOfExistingMapFiles(
      map_folder_path_without_trailing_slash, list_of_map_files);
}

template <typename MapType>
void MapManager<MapType>::getDefaultMapKeys(
    const std::vector<std::string>& map_list,
    std::vector<std::string>* key_list) {
  CHECK_NOTNULL(key_list)->clear();
  key_list->reserve(map_list.size());

  for (const std::string& map_folder : map_list) {
    CHECK(!map_folder.empty());
    std::string key_name, folder_without_key;
    common::splitPathAndFilename(map_folder, &folder_without_key, &key_name);
    key_name = map_storage_->removeProhibitedCharactersFromKey(key_name);
    key_list->emplace_back(key_name);
  }

  CHECK_EQ(map_list.size(), key_list->size());
}

}  // namespace backend

#endif  // MAP_MANAGER_MAP_MANAGER_INL_H_
