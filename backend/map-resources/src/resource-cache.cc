#include "map-resources/resource-cache.h"

#include <gflags/gflags.h>

DEFINE_int64(resources_max_cache_size, 100, "");

namespace backend {

ResourceCache::Config ResourceCache::Config::fromGflags() {
  ResourceCache::Config config;
  config.max_cache_size = FLAGS_resources_max_cache_size;
  return config;
}

template <>
typename ResourceCache::Cache<cv::Mat>::ResourceDequePtr&
ResourceCache::getCachePtr<cv::Mat>(const ResourceType& type) {
  return image_cache_[type];
}

template <>
typename ResourceCache::Cache<std::string>::ResourceDequePtr&
ResourceCache::getCachePtr<std::string>(const ResourceType& type) {
  return text_cache_[type];
}

template <>
typename ResourceCache::Cache<resources::PointCloud>::ResourceDequePtr&
ResourceCache::getCachePtr<resources::PointCloud>(const ResourceType& type) {
  return pointcloud_cache_[type];
}

template <>
typename ResourceCache::Cache<voxblox::TsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::TsdfMap>(const ResourceType& type) {
  return voxblox_tsdf_map_cache_[type];
}

template <>
typename ResourceCache::Cache<voxblox::EsdfMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::EsdfMap>(const ResourceType& type) {
  return voxblox_esdf_map_cache_[type];
}

template <>
typename ResourceCache::Cache<voxblox::OccupancyMap>::ResourceDequePtr&
ResourceCache::getCachePtr<voxblox::OccupancyMap>(const ResourceType& type) {
  return voxblox_occupancy_map_cache_[type];
}

template <>
typename ResourceCache::Cache<
    resources::ObjectInstanceBoundingBoxes>::ResourceDequePtr&
ResourceCache::getCachePtr<resources::ObjectInstanceBoundingBoxes>(
    const ResourceType& type) {
  return bounding_boxes_map_cache_[type];
}

void ResourceCache::resetStatistic() {
  statistic_.reset();
}

// NOTE: [ADD_RESOURCE_DATA_TYPE] [ADD_RESOURCE_TYPE] Make sure the cache of the
// new data type is also listed here.
void ResourceCache::deleteResourceNoDataType(
    const ResourceId& id, const ResourceType& type) {
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
      deleteResource<cv::Mat>(id, type);
      break;
    case ResourceType::kText:
    case ResourceType::kPmvsReconstructionPath:
    case ResourceType::kTsdfGridPath:
    case ResourceType::kEsdfGridPath:
    case ResourceType::kOccupancyGridPath:
      deleteResource<std::string>(id, type);
      break;
    case ResourceType::kPointCloudXYZ:
    case ResourceType::kPointCloudXYZRGBN:
    case ResourceType::kPointCloudXYZI:
    case ResourceType::kPointCloudXYZL:
      deleteResource<resources::PointCloud>(id, type);
      break;
    case ResourceType::kPointCloudXYZIRT:
      deleteResource<resources::PointCloud>(id, type);
      break;
    case ResourceType::kVoxbloxTsdfMap:
      deleteResource<voxblox::TsdfMap>(id, type);
      break;
    case ResourceType::kVoxbloxEsdfMap:
      deleteResource<voxblox::EsdfMap>(id, type);
      break;
    case ResourceType::kVoxbloxOccupancyMap:
      deleteResource<voxblox::OccupancyMap>(id, type);
      break;
    case ResourceType::kObjectInstanceBoundingBoxes:
      deleteResource<resources::ObjectInstanceBoundingBoxes>(id, type);
      break;
    default:
      LOG(FATAL) << "Removing a resource from this cache is not implemented "
                 << "for this resource type!";
      break;
  }
}

const CacheStatistic& ResourceCache::getStatistic() const {
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

const ResourceCache::Config& ResourceCache::getConfig() const {
  return config_;
}

}  // namespace backend
