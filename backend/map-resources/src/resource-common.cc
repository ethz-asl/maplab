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
  return point_cloud_A == point_cloud_B;
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

template <>
bool isSameResource(
    const resources::ObjectInstanceBoundingBoxes& bboxes_A,
    const resources::ObjectInstanceBoundingBoxes& bboxes_B) {
  if (bboxes_A.size() != bboxes_B.size()) {
    return false;
  }

  for (size_t idx = 0; idx < bboxes_A.size(); ++idx) {
    const resources::ObjectInstanceBoundingBox& bbox_A = bboxes_A[idx];
    const resources::ObjectInstanceBoundingBox& bbox_B = bboxes_B[idx];

    if (bbox_A != bbox_B) {
      return false;
    }
  }
  return true;
}

bool csvStringToResourceTypeList(
    const std::string& csv_resource_types,
    std::vector<ResourceType>* resource_type_list) {
  CHECK_NOTNULL(resource_type_list)->clear();
  if (csv_resource_types.empty()) {
    return true;
  }

  static const std::string kDelimiter = ",";

  std::vector<std::string> resource_type_strings;
  common::tokenizeString(
      csv_resource_types, kDelimiter, &resource_type_strings);

  if (resource_type_strings.empty()) {
    // Nothing to do here.
    return true;
  }

  for (const std::string& resource_type_string : resource_type_strings) {
    bool is_valid = false;
    try {
      const int resource_type_int = std::stoi(resource_type_string);
      if (resource_type_int >= 0 &&
          resource_type_int < static_cast<int>(ResourceType::kCount)) {
        is_valid = true;
        resource_type_list->emplace_back(
            static_cast<ResourceType>(resource_type_int));
      }
    } catch (std::invalid_argument const& e) {
      // Fall through intended.
    } catch (std::out_of_range const& e) {
      // Fall through intended.
    }
    if (!is_valid) {
      LOG(ERROR) << "'" << resource_type_string
                 << "' is not a valid resource type number!";
      resource_type_list->clear();
      return false;
    }
  }
  return true;
}

}  // namespace backend
