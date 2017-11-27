#ifndef MAP_RESOURCES_RESOURCE_CONVERSION_INL_H_
#define MAP_RESOURCES_RESOURCE_CONVERSION_INL_H_

#include <vector>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <glog/logging.h>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <voxblox/core/common.h>

#include "map-resources/resource-typedefs.h"

namespace backend {

// Adds a point to a point cloud at a specific index. This function assumes that
// the point cloud has already been resized to allow for direct insertion at
// this index.
template <typename PointCloudType>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const resources::RgbaColor& color,
    const size_t index, PointCloudType* point_cloud);

template <typename PointCloudType>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    PointCloudType* point_cloud);

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const resources::RgbaColor& color,
    const size_t index, pose::Position3DVector* point_cloud);

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const resources::RgbaColor& color,
    const size_t index, resources::VoxbloxColorPointCloud* point_cloud);
template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const resources::RgbaColor& color,
    const size_t index, resources::PointCloud* point_cloud);

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    pose::Position3DVector* point_cloud);

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::VoxbloxColorPointCloud* point_cloud);
template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::PointCloud* point_cloud);

template <typename PointCloudType>
bool hasColorInformation(const PointCloudType& point_cloud);

template <>
bool hasColorInformation(const pose::Position3DVector& point_cloud);

template <>
bool hasColorInformation(const resources::VoxbloxColorPointCloud& point_cloud);

template <>
bool hasColorInformation(const resources::PointCloud& point_cloud);

template <typename PointCloudType>
void resizePointCloud(const size_t size, PointCloudType* point_cloud);

template <>
void resizePointCloud(const size_t size, pose::Position3DVector* point_cloud);

template <>
void resizePointCloud(
    const size_t size, resources::VoxbloxColorPointCloud* point_cloud);

template <>
void resizePointCloud(const size_t size, resources::PointCloud* point_cloud);

template <typename PointCloudType>
size_t getPointCloudSize(const PointCloudType& point_cloud);

template <>
size_t getPointCloudSize(const pose::Position3DVector& point_cloud);

template <>
size_t getPointCloudSize(const resources::VoxbloxColorPointCloud& point_cloud);

template <>
size_t getPointCloudSize(const resources::PointCloud& point_cloud);

template <typename PointCloudType>
void getPointFromPointCloud(
    const PointCloudType& point_cloud, const size_t index,
    Eigen::Vector3d* point_C, resources::RgbaColor* color);

template <typename PointCloudType>
void getPointFromPointCloud(
    const PointCloudType& point_cloud, const size_t index,
    Eigen::Vector3d* point_C) {
  getPointFromPointCloud(point_cloud, index, point_C, nullptr);
}

template <>
void getPointFromPointCloud(
    const pose::Position3DVector& point_cloud, const size_t index,
    Eigen::Vector3d* point_C, resources::RgbaColor* color);

template <>
void getPointFromPointCloud(
    const resources::VoxbloxColorPointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C, resources::RgbaColor* color);

template <>
void getPointFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C, resources::RgbaColor* color);

template <typename PointCloudType>
bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    PointCloudType* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK(!depth_map.empty());
  CHECK_GT(depth_map.rows, 0);
  CHECK_GT(depth_map.cols, 0);
  CHECK_EQ(CV_MAT_TYPE(depth_map.type()), CV_16U);
  // Image should either be grayscale 8bit image or color 3x8bit.
  CHECK(
      CV_MAT_TYPE(image.type()) == CV_8UC1 ||
      CV_MAT_TYPE(image.type()) == CV_8UC3);

  const bool has_image =
      (depth_map.rows == image.rows) && (depth_map.cols == image.cols);

  const bool has_three_channels = CV_MAT_TYPE(image.type()) == CV_8UC3;

  const size_t valid_depth_entries = cv::countNonZero(depth_map);

  if (valid_depth_entries == 0u) {
    VLOG(3) << "Depth map has no valid depth measurements!";
    return false;
  }

  resizePointCloud(valid_depth_entries, point_cloud);

  constexpr double kMillimetersToMeters = 1e-3;
  constexpr double kEpsilon = 1e-6;

  resources::RgbaColor color(255u, 255u, 255u, 255u);
  const uint16_t* depth_map_ptr;
  size_t num_points = 0u;
  for (int v = 0; v < depth_map.rows; ++v) {
    depth_map_ptr = depth_map.ptr<uint16_t>(v);
    for (int u = 0; u < depth_map.cols; ++u) {
      const uint16_t depth = depth_map_ptr[u];
      if (depth == 0u) {
        continue;
      }

      Eigen::Vector3d point_C;
      Eigen::Vector2d image_point;
      image_point << u, v;
      camera.backProject3(image_point, &point_C);
      const double depth_in_meters =
          static_cast<double>(depth) * kMillimetersToMeters;

      if (point_C.z() < kEpsilon) {
        continue;
      }

      point_C /= point_C.z();
      point_C = depth_in_meters * point_C;

      if (has_image) {
        if (has_three_channels) {
          const cv::Vec3b& cv_color = image.at<cv::Vec3b>(v, u);
          // NOTE: Assumes the image are stored as BGR.
          color[0] = cv_color[2];
          color[1] = cv_color[1];
          color[2] = cv_color[0];
        } else {
          color[0] = image.at<uint8_t>(v, u);
          color[1] = color[0];
          color[2] = color[0];
        }
        color[3] = 255u;
      }

      addPointToPointCloud(point_C, color, num_points, point_cloud);
      ++num_points;
    }
  }
  VLOG(3) << "Converted depth map to a point cloud of size " << num_points
          << ".";

  if (num_points == 0u) {
    VLOG(3) << "Depth map has no valid depth measurements!";
    return false;
  }

  return true;
}

template <typename InputPointCloud, typename OutputPointCloud>
bool convertPointCloudType(
    const InputPointCloud& input_cloud, OutputPointCloud* output_cloud) {
  CHECK_NOTNULL(output_cloud);

  const bool input_has_color = hasColorInformation(input_cloud);

  const size_t num_points = getPointCloudSize(input_cloud);
  resizePointCloud(num_points, output_cloud);

  for (size_t point_idx = 0u; point_idx < num_points; ++point_idx) {
    Eigen::Vector3d point_C;
    resources::RgbaColor color;
    if (input_has_color) {
      getPointFromPointCloud(input_cloud, point_idx, &point_C, &color);
      addPointToPointCloud(point_C, color, point_idx, output_cloud);
    } else {
      getPointFromPointCloud(input_cloud, point_idx, &point_C);
      addPointToPointCloud(point_C, point_idx, output_cloud);
    }
  }
  return true;
}

}  // namespace backend

#endif  // MAP_RESOURCES_RESOURCE_CONVERSION_INL_H_
