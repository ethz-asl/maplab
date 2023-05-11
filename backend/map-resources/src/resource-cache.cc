#include "map-resources/resource-cache.h"

namespace backend {

ResourceCache::ResourceCache() {
  // Set the default cache size to very small to not waste memory through
  // default behavior. Individual modules can increase it when necessary.
  max_cache_size_ = 10u;

  // Disable caching a resource when added. This is useful only in very
  // particular cases during map building when performing other online
  // tasks. Let those modules separately enable this behavior.
  always_cache_newest_resource_ = false;
}

size_t ResourceCache::getMaxCacheSize() const {
  return max_cache_size_;
}

void ResourceCache::setMaxCacheSize(size_t max_cache_size) const {
  max_cache_size_ = max_cache_size;
}

bool ResourceCache::getAlwaysCacheNewestResource() const {
  return always_cache_newest_resource_;
}

void ResourceCache::setAlwaysCacheNewestResource(
    bool always_cache_newest_resource) const {
  always_cache_newest_resource_ = always_cache_newest_resource;
}

template <>
typename ResourceCache::Cache<cv::Mat>::ResourceDequePtr&
ResourceCache::getCachePtr<cv::Mat>(const ResourceType& type) const {
  return image_cache_[type];
}

template <>
typename ResourceCache::Cache<std::string>::ResourceDequePtr&
ResourceCache::getCachePtr<std::string>(const ResourceType& type) const {
  return text_cache_[type];
}

template <>
typename ResourceCache::Cache<resources::PointCloud>::ResourceDequePtr&
ResourceCache::getCachePtr<resources::PointCloud>(
    const ResourceType& type) const {
  return pointcloud_cache_[type];
}

template <>
typename ResourceCache::Cache<voxblox::TsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::TsdfMap>(const ResourceType& type) const {
  return voxblox_tsdf_map_cache_[type];
}

template <>
typename ResourceCache::Cache<voxblox::EsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::EsdfMap>(const ResourceType& type) const {
  return voxblox_esdf_map_cache_[type];
}

template <>
typename ResourceCache::Cache<voxblox::OccupancyMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::OccupancyMap>(
    const ResourceType& type) const {
  return voxblox_occupancy_map_cache_[type];
}

template <>
typename ResourceCache::Cache<
    resources::ObjectInstanceBoundingBoxes>::ResourceDequePtr&
ResourceCache::getCachePtr<resources::ObjectInstanceBoundingBoxes>(
    const ResourceType& type) const {
  return bounding_boxes_cache_[type];
}

void ResourceCache::resetCacheStatistic() const {
  statistic_.reset();
}

// NOTE: [ADD_RESOURCE_DATA_TYPE] [ADD_RESOURCE_TYPE] Make sure the cache of the
// new data type is also listed here.
void ResourceCache::deleteCacheResourceNoDataType(
    const ResourceId& id, const ResourceType& type) const {
  switch (type) {
    case ResourceType::kRawImage:
    case ResourceType::kUndistortedImage:
    case ResourceType::kRectifiedImage:
    case ResourceType::kImageForDepthMap:
    case ResourceType::kRawColorImage:
    case ResourceType::kUndistortedColorImage:
    case ResourceType::kRectifiedColorImage:
    case ResourceType::kColorImageForDepthMap:
    case ResourceType::kRawDepthMap:
    case ResourceType::kOptimizedDepthMap:
    case ResourceType::kDisparityMap:
    case ResourceType::kObjectInstanceMasks:
      deleteCacheResource<cv::Mat>(id, type);
      break;
    case ResourceType::kText:
    case ResourceType::kPmvsReconstructionPath:
    case ResourceType::kTsdfGridPath:
    case ResourceType::kEsdfGridPath:
    case ResourceType::kOccupancyGridPath:
      deleteCacheResource<std::string>(id, type);
      break;
    case ResourceType::kPointCloudXYZ:
    case ResourceType::kPointCloudXYZRGBN:
    case ResourceType::kPointCloudXYZI:
    case ResourceType::kPointCloudXYZL:
      deleteCacheResource<resources::PointCloud>(id, type);
      break;
    case ResourceType::kVoxbloxTsdfMap:
      deleteCacheResource<voxblox::TsdfMap>(id, type);
      break;
    case ResourceType::kVoxbloxEsdfMap:
      deleteCacheResource<voxblox::EsdfMap>(id, type);
      break;
    case ResourceType::kVoxbloxOccupancyMap:
      deleteCacheResource<voxblox::OccupancyMap>(id, type);
      break;
    case ResourceType::kObjectInstanceBoundingBoxes:
      deleteCacheResource<resources::ObjectInstanceBoundingBoxes>(id, type);
      break;
    default:
      LOG(FATAL) << "Removing a resource from this cache is not implemented "
                 << "for this resource type!";
      break;
  }
}

const CacheStatistic& ResourceCache::getCacheStatistic() const {
  return statistic_;
}

size_t CacheStatistic::getNumHits(const ResourceType& type) const {
  return hit[static_cast<size_t>(type)];
}

size_t CacheStatistic::getNumMiss(const ResourceType& type) const {
  return miss[static_cast<size_t>(type)];
}

void CacheStatistic::reset() {
  for (size_t idx = 0u; idx < kNumResourceTypes; ++idx) {
    hit[idx] = 0u;
    miss[idx] = 0u;
  }
}

void CacheStatistic::printToLog(int verbosity) const {
  VLOG(verbosity) << print();
}

std::string CacheStatistic::print() const {
  CHECK_EQ(hit.size(), miss.size());

  std::stringstream ss;
  ss << "Resource Cache Statistics:\n";
  for (size_t type_idx = 0u; type_idx < kNumResourceTypes; ++type_idx) {
    std::stringstream ss_name;
    ss_name << std::left << std::setw(30)
            << ("Cache [" + ResourceTypeNames[type_idx] + "]:");
    const std::string& padded_name = ss_name.str();

    ss << "  " << padded_name << "\t"
       << " entries: " << cache_size[type_idx] << " hits: " << hit[type_idx]
       << " miss: " << miss[type_idx] << std::endl;
  }
  return ss.str();
}

}  // namespace backend
