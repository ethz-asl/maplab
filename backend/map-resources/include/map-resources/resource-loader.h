#ifndef MAP_RESOURCES_RESOURCE_LOADER_H_
#define MAP_RESOURCES_RESOURCE_LOADER_H_

#include <string>

#include "map-resources/resource-cache.h"
#include "map-resources/resource-common.h"

namespace backend {

class ResourceLoader {
 public:
  ResourceLoader()
      : resource_types_file_suffixes_(getResourceTypesFileSuffixes()) {}

  void migrateResource(
      const ResourceId& id, const ResourceType& type,
      const std::string& old_folder, const std::string& new_folder,
      const bool move_resource);

  template <typename DataType>
  void deleteResource(
      const ResourceId& id, const ResourceType& type,
      const std::string& folder);

  void deleteResourceNoDataType(
      const ResourceId& id, const ResourceType& type,
      const std::string& folder);

  template <typename DataType>
  void addResource(
      const ResourceId& id, const ResourceType& type, const std::string& folder,
      const DataType& resource);

  template <typename DataType>
  void getResource(
      const ResourceId& id, const ResourceType& type, const std::string& folder,
      DataType* resource) const;

  template <typename DataType>
  bool checkResourceFile(
      const ResourceId& id, const ResourceType& type,
      const std::string& folder) const;

  template <typename DataType>
  void replaceResource(
      const ResourceId& id, const ResourceType& type, const std::string& folder,
      const DataType& resource);

  const CacheStatistic& getCacheStatistic() const;

  const ResourceCache::Config& getCacheConfig() const;

  bool resourceFileExists(
      const ResourceId& id, const ResourceType& type,
      const std::string& folder) const;

  void getResourceFilePath(
      const ResourceId& id, const ResourceType& type, const std::string& folder,
      std::string* file_path) const;

  void getPointCloudResourceFilePath(
      const ResourceId& id, const ResourceType& type, const std::string& folder,
      const bool compressed, std::string* file_path) const;

  void getResourceTypeFileSuffix(
      const ResourceType& type, std::string* file_suffix);

  void deleteResourceFile(
      const ResourceId& id, const ResourceType& type,
      const std::string& folder);

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Implement and add declaration below.
  template <typename DataType>
  void saveResourceToFile(
      const std::string& file_path, const ResourceType& type,
      const DataType& resource) const;

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Implement and add declaration below.
  template <typename DataType>
  bool loadResourceFromFile(
      const std::string& file_path, const ResourceType& type,
      DataType* resource) const;

 private:
  mutable ResourceCache cache_;
  const std::array<std::string, backend::kNumResourceTypes>
      resource_types_file_suffixes_;
};

template <>
void ResourceLoader::getResource(
    const ResourceId& id, const ResourceType& type, const std::string& folder,
    resources::PointCloud* resource) const;

// Implementation for cv::Mat resources.
template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& type,
    const cv::Mat& resource) const;
template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& type,
    cv::Mat* resource) const;

// Implementation for std::string resources.
template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& type,
    const std::string& resource) const;
template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& type,
    std::string* resource) const;

// Implementation for voxblox::TsdfMap resources.
template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& type,
    const voxblox::TsdfMap& resource) const;
template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& type,
    voxblox::TsdfMap* resource) const;

// Implementation for voxblox::EsdfMap resources.
template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& type,
    const voxblox::EsdfMap& resource) const;
template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& type,
    voxblox::EsdfMap* resource) const;

// Implementation for voxblox::OccupancyMap resources.
template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& type,
    const voxblox::OccupancyMap& resource) const;
template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& type,
    voxblox::OccupancyMap* resource) const;

// Implementation for PointCloud.
template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& type,
    const resources::PointCloud& resource) const;
template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& type,
    resources::PointCloud* resource) const;

// Implementation for ObjectInstanceBoundingBox resources.
template <>
void ResourceLoader::saveResourceToFile(
    const std::string& file_path, const ResourceType& type,
    const resources::ObjectInstanceBoundingBoxes& resource) const;
template <>
bool ResourceLoader::loadResourceFromFile(
    const std::string& file_path, const ResourceType& type,
    resources::ObjectInstanceBoundingBoxes* resource) const;

}  // namespace backend

#include "map-resources/resource-loader-inl.h"

#endif  // MAP_RESOURCES_RESOURCE_LOADER_H_
