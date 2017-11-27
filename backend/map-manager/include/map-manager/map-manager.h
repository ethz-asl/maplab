#ifndef MAP_MANAGER_MAP_MANAGER_H_
#define MAP_MANAGER_MAP_MANAGER_H_

#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <maplab-common/map-manager-config.h>
#include <maplab-common/monitor.h>

#include "map-manager/map-storage.h"

namespace backend {

/// \brief Provides an interface to access the the maps in the MapStorage.
///
/// Some of the methods of MapManager directly forward the calls to MapStorage.
/// Check with the
/// documentation of MapStorage to get a list of possible failure cases. The
/// methods in here lock
/// the MapStorage.
///
/// Sample usage:
/// \code{.cc}
/// // Create a local map manager.
/// backend::MapManager<MapType> map_manager;
/// // E.g. to use the MapManager for VIMaps, you can set MapType to
/// `vi_map::VIMap`.
///
/// // Get a map.
/// MapType& map = map_manager.getMap("key_of_map");
///
/// // Do work on map.
/// // ...
/// \endcode
///
/// When you are done with the changes to the map, you can simply let the
/// reference to the map and
/// the MapManager go out of scope. There is no need to commit the changes.
///
/// If you want to access the map in a threadsafe way, use:
/// \code{.cc}
/// // Create a local map manager.
/// backend::MapManager map_manager;
///
/// // Get a map (threadsafe).
/// backend::MapManager<MapType>::MapThreadSafeAccess vi_map =
///     map_manager.getMapThreadSafe("key_of_map");
/// // Map is now locked, other threads asking for a threadsafe map will be
/// blocked.
///
/// // Do work on map.
/// // ...
/// \endcode
///
/// \tparam MapType Type of the map.
template <typename MapType>
class MapManager {
 public:
  /// \brief A container for the map that automatically locks the mutex
  /// belonging to the map until
  /// the thread safe access is out of scope.
  ///
  /// See common::Monitor for details.
  typedef typename common::Monitor<MapType>::WriteAccess MapWriteAccess;
  typedef typename common::Monitor<MapType>::ReadAccess MapReadAccess;

  MapManager();
  ~MapManager() {}

  /// \brief Gets a map from the storage.
  ///
  /// Calls MapStorage::getMap().
  /// \param[in] key Key of the map to be returned.
  /// \return Reference to the map.
  MapType* getMapMutable(const std::string& key);
  const MapType& getMap(const std::string& key) const;

  /// \brief Returns a map for write access to use in a thread safe context. The
  /// map is
  /// automatically locked when using this function.
  ///
  /// Crashes when no map can be found under the given key.
  /// \param key Key of the map to be returned.
  /// \returns Thread safe write access of the map.
  MapWriteAccess getMapWriteAccess(const std::string& key);

  /// \brief Returns a const map for read access to use in a thread safe
  /// context. The map is
  /// automatically locked when using this function.
  ///
  /// Crashes when no map can be found under the given key.
  /// \param key Key of the map to be returned.
  /// \returns Thread safe read access of the map.
  MapReadAccess getMapReadAccess(const std::string& key) const;

  /// \brief Checks if a specified map exists.
  ///
  /// Calls MapStorage::hasMap().
  /// \param[in] key Key of the map.
  /// \return True if map exists.
  bool hasMap(const std::string& key) const;

  /// \brief Checks if the given key is valid and doesn't contain any prohibited
  /// characters.
  ///
  /// Calls MapStorage::isKeyValid().
  /// \param key Key to check.
  /// \returns True if the key is valid.
  bool isKeyValid(const std::string& key) const;

  /// \brief Removes any prohibited character from the string.
  ///
  /// Calls MapStorage::removeProhibitedCharactersFromKey().
  /// \param key Key where prohibited characters should be removed.
  /// \returns Original key without any prohibited character.
  std::string removeProhibitedCharactersFromKey(const std::string& key) const;

  /// \brief Gets a list with keys of all maps saved in the storage.
  ///
  /// Calls MapStorage::getAllMapKeys().
  /// \param[out] all_map_keys_list Set where map keys are inserted.
  void getAllMapKeys(std::unordered_set<std::string>* all_map_keys_list) const;
  void getAllMapKeys(std::vector<std::string>* all_map_keys_list) const;

  /// \brief Gets the number of maps saved in the storage.
  ///
  /// Calls MapStorage::numberOfMaps().
  /// \returns Number of maps saved in the storage.
  size_t numberOfMaps() const;

  /// \brief Inserts a map into the storage.
  ///
  /// Calls MapStorage::addMap().
  /// \param[in] key Key under which the map will be stored.
  /// \param[in, out] map Pointer to the map that will be inserted into the
  /// storage. If the
  /// insertion was successful, the given unique pointer will point to nullptr.
  /// Otherwise, the map
  /// remains managed by the given unique pointer.
  void addMap(
      const std::string& key, AlignedUniquePtr<MapType>& map);  // NOLINT

  /// \brief Removes a map from the storage.
  ///
  /// Calls MapStorage::deleteMap().
  /// \param[in] key Key of the map to be removed.
  void deleteMap(const std::string& key);

  /// \brief Removes a map from the storage and returns a unique pointer
  /// pointing to it.
  ///
  /// Calls MapStorage::releaseMap().
  /// \param key[in] Key of the map to be released.
  /// returns Unique pointer to the released map.
  AlignedUniquePtr<MapType> releaseMap(const std::string& key);

  /// \brief Renames a map in the storage.
  ///
  /// Calls MapStorage::renameMap().
  /// \param[in] old_key The key under which the map is currently stored.
  /// \param[in] new_key The new key that the map should have.
  void renameMap(const std::string& old_key, const std::string& new_key);

  /// \brief Copies the map specified by \p source_key into \p target_key.
  ///
  /// Crashes if the source map doesn't exist or if a map under \p target_key
  /// already exists.
  /// \param source_key Key of the map to be copied.
  /// \param target_key Key under which the copy will be stored.
  void copyMap(const std::string& source_key, const std::string& target_key);

  /// \brief Merges \p source_key_merge_from into \p source_key_merge_base.
  /// \param source_key_merge_base The base map for the merge operation.
  /// \param source_key_merge_from The map which will be merged into the base
  /// map.
  /// \param generate_new_ids If set to true, this will generate new ids for the
  /// missions, vertices,
  /// edges and landmarks in \p source_key_b.
  void mergeMaps(
      const std::string& source_key_merge_base,
      const std::string& source_key_merge_from);

  // TODO(eggerk): implement split.

  /// \brief Return the currently set map folder of a map. Crashes if the map
  /// doesn't contain any
  /// metadata.
  /// \param[in] map_key Key of the map.
  /// \param[out] map_folder Pointer to return the map folder.
  void getMapFolder(const std::string& map_key, std::string* map_folder) const;

  /// \brief Sets the map folder of a map.
  /// \param[in] map_key Key of the map.
  /// \param[in] map_folder New map folder.
  void setMapFolder(const std::string& map_key, const std::string& map_folder);

  /// \brief Loads a map from the given file path into the storage.
  ///
  /// Loads the map from the folder given by \p path. The folder should contain
  /// a map folder, its
  /// folder name is implementation specific and can be checked by using the
  /// getSubFolderName()
  /// function.
  /// \param[in] path Path to the folder containing the map data.
  /// \param[in] key_in Key under which the map is stored. If no key is
  /// provided, the key is
  /// generated from the folder name.
  /// \param[out] key_out If the map key should be automatically generated, the
  /// function will write
  /// the generated key into this variable. Optional variable.
  /// \returns True if the operation was successful.
  bool loadMapFromFolder(
      const std::string& folder_path, const std::string& key_in);
  bool loadMapFromFolder(const std::string& folder_path, std::string* key_out);
  bool loadMapFromFolder(const std::string& folder_path);

  /// \brief Loads all maps in a given folder.
  ///
  /// This method recursively scans the given folder and searches for a map file
  /// in all its
  /// subdirectories.
  ///
  /// The keys under which the maps are stored are determined by the filename.
  /// If a key already
  /// exists in the storage, this operation will fail.
  /// \param folder_path Path of the folder containing the maps to load.
  /// \returns True if all maps in the folder are successfully loaded and at
  /// least one map has been loaded.
  bool loadAllMapsFromFolder(const std::string& folder_path);
  bool loadAllMapsFromFolder(
      const std::string& folder_path,
      std::unordered_set<std::string>* new_keys);

  /// \brief Saves a map to a folder.
  ///
  /// By default, this will create a folder folder_path/key and save the map in
  /// there. The
  /// implementation will create another folder in there with the name given by
  /// getSubFolderName().
  /// \p folder_path.
  /// \param key Key of the map to save.
  /// \param overwrite_existing_file If set to true, any already existing file
  /// will be overwritten.
  /// Set to false by default.
  /// \param folder_path Folder in which the map should be saved.
  /// \returns True if the operation was successful.
  bool saveMapToFolder(
      const std::string& key, const std::string& folder_path) const;
  bool saveMapToFolder(
      const std::string& key, const std::string& folder_path,
      const SaveConfig& config) const;

  /// \brief Saves a map into its map folder as specified by the map's metadata.
  /// \param key Key of the map to save.
  /// \param overwrite_existing_file If set to true, any already existing file
  /// will be overwritten.
  /// Set to false by default.
  /// \returns True if the operation was successful.
  bool saveMapToMapFolder(const std::string& key) const;
  bool saveMapToMapFolder(
      const std::string& key, const SaveConfig& config) const;

  /// \brief Saves all map in the storage to a given folder on the file system.
  ///
  /// Creates a folder named after the key for each map in \p folder_path. The
  /// map data is saved in
  /// its folder. The save implementation for each map will create an additional
  /// folder in there
  /// whose name can be checked using the getSubFolderName() function.
  /// \param folder_path Path of the folder where the maps will be saved. If
  /// folder_path is an empty
  /// string, all maps will be saved into their respective map folder.
  /// \param overwrite_existing_file If set to true, any already existing file
  /// will be overwritten.
  /// Set to false by default.
  /// \returns True if all maps are successfully saved.
  bool saveAllMapsToFolder(const std::string& folder_path) const;
  bool saveAllMapsToFolder(
      const std::string& folder_path, const SaveConfig& config) const;

  /// \brief Saves all maps into each map's map folder as specified by the map's
  /// metadata.
  /// \param overwrite_existing_file If set to true, any already existing file
  /// will be overwritten.
  /// Set to false by default.
  /// \returns True if all maps are successfully saved.
  bool saveAllMapsToMapFolder() const;
  bool saveAllMapsToMapFolder(const SaveConfig& config) const;

  /// \brief Checks if all map data exists on the file system.
  /// \param folder_path Path to the folder containing the map.
  /// \returns True if the map exists on the file system.
  bool hasMapOnFileSystem(const std::string& folder_path) const;

  /// \brief Returns the list of all existing map files in the given map folder.
  ///
  /// \param map_folder The folder containing the map files.
  /// \param list_of_map_files List of all files of the given map.
  /// \returns True if the minimum required map files exist.
  bool getListOfExistingMapFiles(
      const std::string& map_folder,
      std::vector<std::string>* list_of_map_files);

  /// \brief Returns the default map keys for a given list of maps folders.
  ///
  /// \param map_list List of map folders.
  /// \param key_list: The default keys of the given maps.
  void getDefaultMapKeys(const std::vector<std::string>& map_list,
                         std::vector<std::string>* key_list);

  /// \brief Lists all maps in a given folder.
  ///
  /// This method recursively scans the given folder and searches for map files
  /// in all its subdirectories.
  /// \param folder_path Folder to search.
  /// \param map_list: List of map folders found in the given folder.
  void listAllMapsInFolder(const std::string& folder_path,
                           std::vector<std::string>* map_list);

 protected:
  MapStorage<MapType>* map_storage_;
};

}  // namespace backend

#include "map-manager/map-manager-inl.h"

#endif  // MAP_MANAGER_MAP_MANAGER_H_
