#ifndef MAP_RESOURCES_RESOURCE_CACHE_H_
#define MAP_RESOURCES_RESOURCE_CACHE_H_

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <maplab-common/unique-id.h>
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
  ResourceCache() {}

  enum class Strategy { kFIFO = 0u };

  struct Config {
    size_t allocated_cache_size = 0u;
    size_t max_cache_size = 100u;
    bool cache_newest_resource = false;
    Strategy strategy = Strategy::kFIFO;
  };

  explicit ResourceCache(const Config& cache_config) : config_(cache_config) {}

  template <typename DataType>
  bool getResource(
      const ResourceId& id, const ResourceType& type, DataType* resource);

  template <typename DataType>
  void putResource(
      const ResourceId& id, const ResourceType& type, const DataType& resource);

  template <typename DataType>
  bool deleteResource(const ResourceId& id, const ResourceType& type);

  void resetStatistic();

  const CacheStatistic& getStatistic() const;

  const Config& getConfig() const;

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

 private:
  template <typename DataType>
  typename Cache<DataType>::ResourceDeque* getCache(const ResourceType& type);

  template <typename DataType>
  typename Cache<DataType>::ResourceDeque* initCache(const ResourceType& type);

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Implement and add declaration below.
  template <typename DataType>
  typename Cache<DataType>::ResourceDequePtr& getCachePtr(
      const ResourceType& type);

  // NOTE: [ADD_RESOURCE_DATA_TYPE] Add member.
  Cache<cv::Mat>::ResourceTypeMap image_cache_;
  Cache<std::string>::ResourceTypeMap text_cache_;
  Cache<resources::PointCloud>::ResourceTypeMap pointcloud_cache_;
  Cache<voxblox::TsdfMap>::ResourceTypeMap voxblox_tsdf_map_cache_;
  Cache<voxblox::EsdfMap>::ResourceTypeMap voxblox_esdf_map_cache_;
  Cache<voxblox::OccupancyMap>::ResourceTypeMap voxblox_occupancy_map_cache_;

  CacheStatistic statistic_;

  Config config_;
};

template <>
typename ResourceCache::Cache<cv::Mat>::ResourceDequePtr&
ResourceCache::getCachePtr<cv::Mat>(const ResourceType& type);

template <>
typename ResourceCache::Cache<std::string>::ResourceDequePtr&
ResourceCache::getCachePtr<std::string>(const ResourceType& type);

template <>
typename ResourceCache::Cache<resources::PointCloud>::ResourceDequePtr&
ResourceCache::getCachePtr<resources::PointCloud>(const ResourceType& type);

template <>
typename ResourceCache::Cache<voxblox::TsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::TsdfMap>(const ResourceType& type);

template <>
typename ResourceCache::Cache<voxblox::EsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::EsdfMap>(const ResourceType& type);

template <>
typename ResourceCache::Cache<voxblox::OccupancyMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::OccupancyMap>(const ResourceType& type);

template <typename DataType>
void updateCacheSizeStatistic(
    const ResourceType& type,
    const typename ResourceCache::Cache<DataType>::ResourceDeque& cache,
    CacheStatistic* statistic);

}  // namespace backend

#include "map-resources/resource-cache-inl.h"

#endif  // MAP_RESOURCES_RESOURCE_CACHE_H_
