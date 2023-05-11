#ifndef MAP_RESOURCES_RESOURCE_CACHE_H_
#define MAP_RESOURCES_RESOURCE_CACHE_H_

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <aslam/common/unique-id.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include "map-resources/resource-common.h"

namespace backend {

struct CacheStatistic {
  std::vector<size_t> hit = std::vector<size_t>(kNumResourceTypes, 0u);
  std::vector<size_t> miss = std::vector<size_t>(kNumResourceTypes, 0u);
  std::vector<size_t> cache_size = std::vector<size_t>(kNumResourceTypes, 0u);

  void reset();
  void printToLog(int verbosity) const;
  std::string print() const;

  size_t getNumHits(const ResourceType& type) const;
  size_t getNumMiss(const ResourceType& type) const;
};

class ResourceCache {
  friend struct CacheStatistic;

 public:
  ResourceCache();

  size_t getMaxCacheSize() const;
  void setMaxCacheSize(size_t max_cache_size) const;

  bool getAlwaysCacheNewestResource() const;
  void setAlwaysCacheNewestResource(bool always_cache_newest_resource) const;

  const CacheStatistic& getCacheStatistic() const;

 protected:
  template <typename DataType>
  bool getCacheResource(
      const ResourceId& id, const ResourceType& type, DataType* resource) const;

  template <typename DataType>
  void putCacheResource(
      const ResourceId& id, const ResourceType& type,
      const DataType& resource) const;

  template <typename DataType>
  bool deleteCacheResource(
      const ResourceId& id, const ResourceType& type) const;
  void deleteCacheResourceNoDataType(
      const ResourceId& id, const ResourceType& type) const;

  void resetCacheStatistic() const;

  mutable size_t max_cache_size_;
  mutable bool always_cache_newest_resource_;

 private:
  template <typename DataType>
  struct Cache {
    typedef std::pair<ResourceId, DataType> Element;
    typedef std::deque<std::pair<ResourceId, DataType>> ResourceDeque;
    typedef std::unique_ptr<ResourceDeque> ResourceDequePtr;
    typedef std::unique_ptr<const ResourceDeque> ResourceDequeConstPtr;
    typedef std::unordered_map<ResourceType, ResourceDequePtr, ResourceTypeHash>
        ResourceTypeMap;
    typedef typename ResourceDeque::const_iterator ConstIterator;
    typedef typename ResourceDeque::iterator Iterator;
  };

  template <typename DataType>
  typename Cache<DataType>::ResourceDeque* getCache(
      const ResourceType& type) const;

  template <typename DataType>
  typename Cache<DataType>::ResourceDeque* initCache(
      const ResourceType& type) const;

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Implement and add declaration below.
  template <typename DataType>
  typename Cache<DataType>::ResourceDequePtr& getCachePtr(
      const ResourceType& type) const;

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Add member.
  mutable Cache<cv::Mat>::ResourceTypeMap image_cache_;
  mutable Cache<std::string>::ResourceTypeMap text_cache_;
  mutable Cache<resources::PointCloud>::ResourceTypeMap pointcloud_cache_;
  mutable Cache<voxblox::TsdfMap>::ResourceTypeMap voxblox_tsdf_map_cache_;
  mutable Cache<voxblox::EsdfMap>::ResourceTypeMap voxblox_esdf_map_cache_;
  mutable Cache<voxblox::OccupancyMap>::ResourceTypeMap
      voxblox_occupancy_map_cache_;
  mutable Cache<resources::ObjectInstanceBoundingBoxes>::ResourceTypeMap
      bounding_boxes_cache_;

  mutable CacheStatistic statistic_;
};

template <>
typename ResourceCache::Cache<cv::Mat>::ResourceDequePtr&
ResourceCache::getCachePtr<cv::Mat>(const ResourceType& type) const;

template <>
typename ResourceCache::Cache<std::string>::ResourceDequePtr&
ResourceCache::getCachePtr<std::string>(const ResourceType& type) const;

template <>
typename ResourceCache::Cache<resources::PointCloud>::ResourceDequePtr&
ResourceCache::getCachePtr<resources::PointCloud>(
    const ResourceType& type) const;

template <>
typename ResourceCache::Cache<voxblox::TsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::TsdfMap>(const ResourceType& type) const;

template <>
typename ResourceCache::Cache<voxblox::EsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::EsdfMap>(const ResourceType& type) const;

template <>
typename ResourceCache::Cache<voxblox::OccupancyMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::OccupancyMap>(
    const ResourceType& type) const;

template <>
typename ResourceCache::Cache<
    resources::ObjectInstanceBoundingBoxes>::ResourceDequePtr&
ResourceCache::getCachePtr<resources::ObjectInstanceBoundingBoxes>(
    const ResourceType& type) const;

template <typename DataType>
void updateCacheSizeStatistic(
    const ResourceType& type,
    const typename ResourceCache::Cache<DataType>::ResourceDeque& cache,
    CacheStatistic* statistic);

}  // namespace backend

#include "map-resources/resource-cache-inl.h"

#endif  // MAP_RESOURCES_RESOURCE_CACHE_H_
