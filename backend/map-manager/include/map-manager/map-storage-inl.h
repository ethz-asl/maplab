#ifndef MAP_MANAGER_MAP_STORAGE_INL_H_
#define MAP_MANAGER_MAP_STORAGE_INL_H_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/reader-writer-lock.h>
#include <glog/logging.h>

#include "map-manager/map-storage.h"

namespace backend {

template <typename MapType>
MapType* MapStorage<MapType>::getMapMutable(const std::string& key) {
  MapStorageContainerIterator it = map_storage_container_.find(key);
  CHECK(it != map_storage_container_.end()) << "Map with key \"" << key
                                            << "\" does not exist!";
  CHECK(it->second->map != nullptr) << "Stored map is nullptr!";
  return it->second->map.get();
}

template <typename MapType>
const MapType& MapStorage<MapType>::getMap(const std::string& key) const {
  MapStorageContainerConstIterator it = map_storage_container_.find(key);
  CHECK(it != map_storage_container_.end()) << "Map with key \"" << key
                                            << "\" does not exist!";
  CHECK(it->second->map != nullptr) << "Stored map is nullptr!";
  return *(it->second->map);
}

template <typename MapType>
typename common::Monitor<MapType>::WriteAccess
MapStorage<MapType>::getMapWriteAccess(const std::string& key) {
  const MapStorageContainerConstIterator it = map_storage_container_.find(key);
  CHECK(it != map_storage_container_.end()) << "Map with key \"" << key
                                            << "\" does not exist!";
  CHECK(it->second->map != nullptr) << "Stored map is nullptr!";
  MapAndMutex& map_handle = *(it->second);
  return typename common::Monitor<MapType>::WriteAccess(
      map_handle.map.get(), &map_handle.map_mutex);
}

template <typename MapType>
typename common::Monitor<MapType>::ReadAccess
MapStorage<MapType>::getMapReadAccess(const std::string& key) const {
  const MapStorageContainerConstIterator it = map_storage_container_.find(key);
  CHECK(it != map_storage_container_.end()) << "Map with key \"" << key
                                            << "\" does not exist!";
  CHECK(it->second->map != nullptr) << "Stored map is nullptr!";
  MapAndMutex& map_handle = *(it->second);
  return typename common::Monitor<MapType>::ReadAccess(
      map_handle.map.get(), &map_handle.map_mutex);
}

template <typename MapType>
bool MapStorage<MapType>::isKeyValid(const std::string& key) const {
  if (key.empty()) {
    VLOG(3) << "The key \"" << key << "\" is invalid because it's empty.";
    return false;
  }
  // Using a regex instead of looping through every character might be nicer,
  // but std::regex isn't
  // supported by GCC 4.8 (default on Ubuntu 14.04).
  for (const char& character : key) {
    if (isCharacterProhibited(character)) {
      VLOG(3) << "The key \"" << key << "\" is invalid because the character \""
              << character << "\" is not allowed.";
      return false;
    }
  }
  return true;
}

template <typename MapType>
bool MapStorage<MapType>::isCharacterProhibited(const char character) const {
  const std::string kAllowedNonAlphaNumericCharacters = "-_";
  return !std::isalnum(character) &&
         kAllowedNonAlphaNumericCharacters.find(character) == std::string::npos;
}

template <typename MapType>
std::string MapStorage<MapType>::removeProhibitedCharactersFromKey(
    const std::string& key) const {
  std::string result(key);
  result.erase(
      std::remove_if(
          result.begin(), result.end(),
          std::bind(
              &MapStorage<MapType>::isCharacterProhibited, this,
              std::placeholders::_1)),
      result.end());
  CHECK(isKeyValid(result));
  return result;
}

template <typename MapType>
bool MapStorage<MapType>::hasMap(const std::string& key) const {
  return map_storage_container_.count(key) > 0u;
}

template <typename MapType>
void MapStorage<MapType>::getAllMapKeys(
    std::unordered_set<std::string>* all_map_keys_list) const {
  CHECK_NOTNULL(all_map_keys_list)->clear();
  for (const MapStorageContainerValueType& key_value_pair :
       map_storage_container_) {
    all_map_keys_list->emplace(key_value_pair.first);
  }
}

template <typename MapType>
void MapStorage<MapType>::getAllMapKeys(
    std::vector<std::string>* all_map_keys_list) const {
  CHECK_NOTNULL(all_map_keys_list)->clear();
  for (const MapStorageContainerValueType& key_value_pair :
       map_storage_container_) {
    all_map_keys_list->emplace_back(key_value_pair.first);
  }
}

template <typename MapType>
void MapStorage<MapType>::addMap(
    const std::string& key, AlignedUniquePtr<MapType>& map) {
  std::unique_ptr<MapAndMutex> map_handle(new MapAndMutex(map));
  CHECK(map == nullptr);
  addMap(key, map_handle);
}

template <typename MapType>
void MapStorage<MapType>::addMap(
    const std::string& key, std::unique_ptr<MapAndMutex>& map_handle) {
  CHECK(map_handle != nullptr) << "Map handle cannot be null!";
  CHECK(map_handle->map != nullptr) << "Map cannot be null!";
  CHECK(!key.empty()) << "Map key cannot be empty!";
  CHECK(isKeyValid(key)) << "Key \"" << key << "\" is not a valid key.";
  CHECK(!hasMap(key)) << "Map with key \"" << key << "\" already exists!";

  CHECK(map_storage_container_.emplace(key, std::move(map_handle)).second)
      << "Failure inserting map.";
  CHECK(hasMap(key) && map_handle == nullptr) << "Failure inserting map.";
}

template <typename MapType>
std::unique_ptr<typename MapStorage<MapType>::MapAndMutex>
MapStorage<MapType>::releaseMapAndMutex(const std::string& key) {
  std::unique_ptr<MapAndMutex> map_and_mutex;
  MapStorageContainerIterator it = map_storage_container_.find(key);
  CHECK(it != map_storage_container_.end())
      << "Map cannot be released because map with key \"" << key
      << "\" does not exist.";
  map_and_mutex = std::move(it->second);
  CHECK(it->second == nullptr);
  map_storage_container_.erase(it);
  return std::move(map_and_mutex);
}

template <typename MapType>
void MapStorage<MapType>::renameMap(
    const std::string& old_key, const std::string& new_key) {
  CHECK(!new_key.empty() && !old_key.empty())
      << "Source and target keys cannot be empty!";
  CHECK(isKeyValid(new_key)) << "Target key \"" << new_key
                             << "\" is not a valid key.";
  CHECK(!hasMap(new_key)) << "Map cannot be moved because key \"" << new_key
                          << "\" already exists!";

  // Remove old entry.
  MapStorageContainerIterator it = map_storage_container_.find(old_key);
  CHECK(it != map_storage_container_.end())
      << "Map with key \"" << old_key
      << "\" cannot be moved because it does not exist!";
  std::unique_ptr<MapAndMutex> map_handle(std::move(it->second));
  CHECK(it->second == nullptr)
      << "Failure renaming map: couldn't erase old entry.";
  map_storage_container_.erase(it);
  CHECK(!hasMap(old_key)) << "Failure renaming map: couldn't erase old entry.";

  addMap(new_key, map_handle);
}

template <typename MapType>
aslam::ReaderWriterMutex* MapStorage<MapType>::getContainerMutex() const {
  return &container_mutex_;
}

}  // namespace backend

#endif  // MAP_MANAGER_MAP_STORAGE_INL_H_
