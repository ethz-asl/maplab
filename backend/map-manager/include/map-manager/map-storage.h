#ifndef MAP_MANAGER_MAP_STORAGE_H_
#define MAP_MANAGER_MAP_STORAGE_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <aslam/common/memory.h>
#include <aslam/common/reader-writer-lock.h>
#include <maplab-common/macros.h>
#include <maplab-common/monitor.h>

namespace backend {

template <typename MapType>
class MapManager;

/// \brief Stores and owns all maps.
///
/// The MapStorage is responsible for storing all the maps. The maps are stored
/// as a pair of a key
/// and a unique pointer to the map. Each MapType has its own separate
/// MapStorage. The methods in
/// this class are not thread safe. If thread safe access is desired, the
/// methods in MapManager
/// should be used.
/// \tparam MapType Type of the map to be managed.
template <typename MapType>
class MapStorage {
 private:
  friend class MapManager<MapType>;

  /// \brief Holds a map and its mutex together.
  /// \tparam MapType Type of the map.
  struct MapAndMutex {
   private:
    MAPLAB_POINTER_TYPEDEFS(MapAndMutex);
    friend class MapStorage<MapType>;
    friend class MapManager<MapType>;

    /// \brief Map that is stored.
    AlignedUniquePtr<MapType> map;

    /// \brief Mutex to lock access to the map in this handle.
    mutable aslam::ReaderWriterMutex map_mutex;

    explicit MapAndMutex(AlignedUniquePtr<MapType>& map_)  // NOLINT
        : map(std::move(map_)) {}
  };

  /// \brief Get an instance of this map storage.
  /// \returns Pointer to the MapStorage object for this MapType.
  static MapStorage<MapType>* getInstance() {
    static MapStorage<MapType> map_storage;
    return &map_storage;
  }

 public:
  /// \brief Gets a map with a given key.
  ///
  /// Crashes when no map can be found under the given key or when the stored
  /// map is nullptr.
  /// \param[in] key Key of the map.
  /// \returns Pointer to the map.
  MapType* getMapMutable(const std::string& key);
  const MapType& getMap(const std::string& key) const;

  /// \brief Returns a map for write access to use in a threadsafe context. The
  /// map is automatically
  /// locked when using this function.
  ///
  /// Crashes when no map can be found under the given key.
  /// \param key Key of the map to be returned.
  /// \returns Thread safe write access of the map.
  typename common::Monitor<MapType>::WriteAccess getMapWriteAccess(
      const std::string& key);

  /// \brief Returns a const map for read access to use in a threadsafe context.
  /// The map is
  /// automatically locked when using this function.
  ///
  /// Crashes when no map can be found under the given key.
  /// \param key Key of the map to be returned.
  /// \returns Thread safe read access of the map.
  typename common::Monitor<MapType>::ReadAccess getMapReadAccess(
      const std::string& key) const;

  /// \brief Checks if a map with the given key exists.
  /// \param[in] key Key of the map to check.
  /// \returns True if a map with the given key exists.
  bool hasMap(const std::string& key) const;

  /// \brief Checks if the key is valid.
  ///
  /// A key is valid if it is not empty and only contains alphanumeric
  /// characters, "-" or "_". Other
  /// characters are prohibited because they might not be supported on all
  /// filesystems.
  ///
  /// This only checks for general validity of the key, this doesn't check if a
  /// map is already
  /// stored under the given key. Use hasMap() to see if the key is already
  /// used.
  /// \param key The key to check.
  /// \returns True if the key is valid.
  bool isKeyValid(const std::string& key) const;

  /// \brief Removes any prohibited character from the string.
  ///
  /// A prohibited character is any character that isn't alphanumeric, "-" or
  /// "_". The prohibited
  /// characters contain characters that are not supported by all file systems
  /// or might need
  /// escaping on a command line.
  /// \param key Key where prohibited characters should be removed.
  /// \returns Original key without any prohibited character.
  std::string removeProhibitedCharactersFromKey(const std::string& key) const;

  /// \brief Queries the number of maps stored.
  /// \returns Number of maps stored for this MapType.
  size_t numberOfMaps() const {
    return map_storage_container_.size();
  }

  /// \brief Fills a list with keys of all maps saved.
  ///
  /// Crashes when the input is not a valid pointer.
  /// \param[out] all_map_keys_list Set in which the keys should be outputted.
  /// The set will be cleared at the start of the method.
  void getAllMapKeys(std::unordered_set<std::string>* all_map_keys_list) const;
  void getAllMapKeys(std::vector<std::string>* all_map_keys_list) const;

  /// \brief Inserts a map into the storage.
  ///
  /// Crashes when the key already exists in the storage, the key is invalid or
  /// the map is nullptr.
  /// \param key[in] Key under which the map should be saved.
  /// \param map[in,out] Unique pointer containing the map to be saved. On
  /// success, the given unique
  /// pointer will point to nullptr after this operation.
  void addMap(const std::string& key, AlignedUniquePtr<MapType>& map);

  /// \brief Removes a map and mutex from the storage and returns a unique
  /// pointer pointing to it.
  ///
  /// Crashes if no map with the given key exists.
  /// \param key[in] Key of the map to be released.
  /// returns Unique pointer to the released map and mutex.
  std::unique_ptr<MapAndMutex> releaseMapAndMutex(const std::string& key);

  /// \brief Renames a certain map by changing the key associated with it.
  ///
  /// Crashes when no map exists under old_key or when a map already exists
  /// under new_key.
  /// \param old_key Key under which the map is currently saved.
  /// \param new_key Key under which the map should be saved from now on.
  void renameMap(const std::string& old_key, const std::string& new_key);

  /// \brief Returns a ReaderWriterMutex to lock the MapStorage for threadsafe
  /// access.
  /// \returns Pointer to the MapStorage's ReaderWriterMutex.
  aslam::ReaderWriterMutex* getContainerMutex() const;

 private:
  /// \brief Container which stores all the maps.
  typedef typename std::unordered_map<std::string, std::unique_ptr<MapAndMutex>>
      MapStorageContainer;
  typedef typename MapStorageContainer::value_type MapStorageContainerValueType;
  typedef typename MapStorageContainer::iterator MapStorageContainerIterator;
  typedef typename MapStorageContainer::const_iterator
      MapStorageContainerConstIterator;

  MapStorage() {}

  MapStorage(const MapStorage&) = delete;
  MapStorage& operator=(const MapStorage&) = delete;

  /// \brief Checks if the given character is allowed in key names.
  ///
  /// Please check MapStorage::isKeyValid() or
  /// MapStorage::removeProhibitedCharactersFromKey() for
  /// more details.
  /// \param character Character to check.
  /// \returns True if the character is prohibited form the use in keys.
  bool isCharacterProhibited(const char character) const;

  void addMap(const std::string& key, std::unique_ptr<MapAndMutex>& map_handle);

  MapStorageContainer map_storage_container_;

  mutable aslam::ReaderWriterMutex container_mutex_;
};

}  // namespace backend

#include "map-manager/map-storage-inl.h"

#endif  // MAP_MANAGER_MAP_STORAGE_H_
