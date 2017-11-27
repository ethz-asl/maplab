#include "map-resources/resource-common.h"

#include <string>

#include <opencv2/core.hpp>
#include <voxblox/utils/layer_utils.h>

namespace backend {

// NOTE: [ADD_RESOURCE_DATA_TYPE] Implement.
template <>
bool isSameResource(const cv::Mat& resource_A, const cv::Mat& resource_B) {
  bool is_same_resource = true;
  is_same_resource &= resource_A.size() == resource_B.size();
  is_same_resource &= resource_A.empty() == resource_B.empty();
  is_same_resource &= cv::countNonZero(resource_A != resource_B) == 0;
  return is_same_resource;
}

template <>
bool isSameResource(
    const std::string& resource_A, const std::string& resource_B) {
  return resource_A == resource_B;
}

template <>
bool isSameResource(
    const resources::PointCloud& point_cloud_A,
    const resources::PointCloud& point_cloud_B) {
  if (point_cloud_A.xyz != point_cloud_B.xyz) {
    return false;
  }
  if (point_cloud_A.normals != point_cloud_B.normals) {
    return false;
  }
  if (point_cloud_A.colors != point_cloud_B.colors) {
    return false;
  }
  return true;
}

template <>
bool isSameResource(
    const voxblox::TsdfMap& map_A, const voxblox::TsdfMap& map_B) {
  return voxblox::utils::isSameLayer(
      map_A.getTsdfLayer(), map_B.getTsdfLayer());
}

template <>
bool isSameResource(
    const voxblox::EsdfMap& map_A, const voxblox::EsdfMap& map_B) {
  return voxblox::utils::isSameLayer(
      map_A.getEsdfLayer(), map_B.getEsdfLayer());
}

template <>
bool isSameResource(
    const voxblox::OccupancyMap& map_A, const voxblox::OccupancyMap& map_B) {
  return voxblox::utils::isSameLayer(
      map_A.getOccupancyLayer(), map_B.getOccupancyLayer());
}

}  // namespace backend
