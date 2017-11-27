#ifndef MAPLAB_COMMON_MAP_TRAITS_H_
#define MAPLAB_COMMON_MAP_TRAITS_H_

#include <string>
#include <vector>

#include <glog/logging.h>
#include <maplab-common/map-manager-config.h>

namespace backend {

/// This needs to be implemented for each map type in order to use the map
/// manager. Derive from
/// MapTraits below to specify the supported functionality, e.g.:
/// \code{.cc}
/// template <>
/// struct traits<MyMapType> : public MapTraits<MyMapType> {};
/// \endcode
/// This defines the trait functions for the type MyMapType.
template <typename MapType>
struct traits;

/// Derive traits from this struct in order to have the functions defined.
/// Your map should derive from backend::ResourceMap (from map_resources) and
/// backend::MapInterface<MapType> (defined in this file) to get the needed
/// functions.
template <typename MapType>
struct MapTraits {
  // Functions provided by ResourceMap.
  // Map folder refers to the default location to save new resources. On a new
  // map, it's
  // unspecified. If a map is loaded from disk, the map folder is set to the
  // path of the map on
  // disk, so that new resources can be stored next to the map. See
  // backend::ResourceMap for more
  // details.
  static bool hasMapFolder(const MapType& map) {
    return map.hasMapFolder();
  }
  static void getMapFolder(const MapType& map, std::string* map_folder) {
    map.getMapFolder(map_folder);
  }
  static void setMapFolder(const std::string& map_folder, MapType* map) {
    CHECK_NOTNULL(map)->setMapFolder(map_folder);
  }

  // Copy/merge.
  static void deepCopy(const MapType& source_map, MapType* target_map) {
    CHECK_NOTNULL(target_map)->deepCopyFrom(source_map);
  }
  static void mergeTwoMaps(
      const MapType& source_map_merge_from, MapType* map_merge_base) {
    CHECK_NOTNULL(map_merge_base)
        ->mergeAllMissionsFromMap(source_map_merge_from);
  }

  // Save/load.

  // Sub folder name refers to the name the map will be stored in. E.g., the
  // implementation for
  // VIMap will return "vi_map" as sub folder name which means that the maps are
  // saved in the folder
  // user/provided/folder/path/vi_map.
  static std::string getSubFolderName() {
    return MapType::getSubFolderName();
  }
  static bool getListOfExistingMapFiles(
      const std::string& map_folder,
      std::vector<std::string>* list_of_map_files) {
    return MapType::getListOfExistingMapFiles(map_folder, list_of_map_files);
  }
  static bool hasMapOnFileSystem(const std::string& folder_path) {
    return MapType::hasMapOnFileSystem(folder_path);
  }

  static bool loadFromFolder(const std::string& folder_path, MapType* map) {
    return CHECK_NOTNULL(map)->loadFromFolder(folder_path);
  }
  static bool saveToFolder(
      const std::string& folder_path, const SaveConfig& config, MapType* map) {
    return CHECK_NOTNULL(map)->saveToFolder(folder_path, config);
  }
};

/// This struct defines the basic functions if you want to use advanced map
/// manager commands (e.g. save or merge) in your map.
///
/// To use these for your map type and use these functions from the map manager,
/// first create the
/// traits:
/// \code{.cc}
/// template <>
/// struct traits<MyMapType> : public backend::MapTraits<MyMapType> {};
/// \endcode
///
/// Then adjust the definition of MyMapType:
/// \code{.cc}
/// class MyMapType : public MapInterface<MyMapType> {
///   // Your members here.
///   ...
///
///   // Also implement the functions from MapInterface.
///   virtual void deepCopyFrom(const MapType& other) override;
///   ...
/// };
/// \endcode
/// Note: You may also need to inherit from backend::ResourceMap.
template <typename MapType>
struct MapInterface {
  virtual void deepCopy(const MapType& other) = 0;
  virtual void mergeAllMissionsFromMap(const MapType& other) = 0;
  virtual bool loadFromFolder(const std::string& folder_path) = 0;
  virtual bool saveToFolder(
      const std::string& folder_path, const SaveConfig& config) = 0;
};

}  // namespace backend

#endif  // MAPLAB_COMMON_MAP_TRAITS_H_
