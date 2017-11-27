#include "map-resources/resource-conversion.h"

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <glog/logging.h>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <voxblox/core/common.h>

namespace backend {

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);

  cv::Mat image(1, 1, CV_8UC1);
  return convertDepthMapToPointCloud(depth_map, image, camera, point_cloud);
}

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    pose::Position3DVector* point_cloud) {
  CHECK_NOTNULL(point_cloud)->clear();

  cv::Mat image(1, 1, CV_8UC1);
  return convertDepthMapToPointCloud(depth_map, image, camera, point_cloud);
}

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);

  return convertDepthMapToPointCloud(depth_map, image, camera, point_cloud);
}

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    pose::Position3DVector* points_C, voxblox::Colors* colors) {
  CHECK_NOTNULL(points_C)->clear();
  CHECK_NOTNULL(colors);
  resources::VoxbloxColorPointCloud voxblox_point_cloud;
  voxblox_point_cloud.points_C = points_C;
  voxblox_point_cloud.colors = colors;
  return convertDepthMapToPointCloud(
      depth_map, image, camera, &voxblox_point_cloud);
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    pose::Position3DVector* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_GT(point_cloud->size(), index);

  pose::Position3D& point = (*point_cloud)[index];
  point.x() = point_C.x();
  point.y() = point_C.y();
  point.z() = point_C.z();
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::VoxbloxColorPointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(point_cloud->points_C);
  CHECK_GT(point_cloud->points_C->size(), index);

  pose::Position3D& point = (*point_cloud->points_C)[index];
  point.x() = point_C.x();
  point.y() = point_C.y();
  point.z() = point_C.z();
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  const size_t start_index = 3 * index;
  CHECK_LT(start_index + 2, point_cloud->xyz.size());

  point_cloud->xyz[start_index] = point_C.x();
  point_cloud->xyz[start_index + 1] = point_C.y();
  point_cloud->xyz[start_index + 2] = point_C.z();
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const resources::RgbaColor& /*color*/,
    const size_t index, pose::Position3DVector* point_cloud) {
  addPointToPointCloud(point_C, index, point_cloud);
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const resources::RgbaColor& color,
    const size_t index, resources::VoxbloxColorPointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(point_cloud->colors);
  CHECK_NOTNULL(point_cloud->points_C);
  CHECK_GT(point_cloud->points_C->size(), index);
  CHECK_GT(point_cloud->colors->size(), index);
  CHECK_EQ(point_cloud->colors->size(), point_cloud->points_C->size());

  voxblox::Color& voxblox_color = (*point_cloud->colors)[index];
  voxblox_color.r = color[0];
  voxblox_color.g = color[1];
  voxblox_color.b = color[2];
  voxblox_color.a = 255u;

  pose::Position3D& point = (*point_cloud->points_C)[index];
  point.x() = point_C.x();
  point.y() = point_C.y();
  point.z() = point_C.z();
}

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const resources::RgbaColor& color,
    const size_t index, resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  const size_t start_index = 3 * index;
  CHECK_LT(start_index + 2, point_cloud->xyz.size());

  point_cloud->xyz[start_index] = point_C.x();
  point_cloud->xyz[start_index + 1] = point_C.y();
  point_cloud->xyz[start_index + 2] = point_C.z();

  if (!point_cloud->colors.empty()) {
    CHECK_LT(start_index + 2, point_cloud->colors.size());
    point_cloud->colors[start_index] = color[0];
    point_cloud->colors[start_index + 1] = color[1];
    point_cloud->colors[start_index + 2] = color[2];
  }
}

template <>
void getPointFromPointCloud(
    const pose::Position3DVector& point_cloud, const size_t index,
    Eigen::Vector3d* point_C, resources::RgbaColor* /*color*/) {
  CHECK_NOTNULL(point_C);
  Eigen::Vector3d& point_C_out = *point_C;
  DCHECK_GT(point_cloud.size(), index);
  const pose::Position3D& point_C_in = point_cloud[index];
  point_C_out[0] = point_C_in[0];
  point_C_out[1] = point_C_in[1];
  point_C_out[2] = point_C_in[2];

  // No color, can be nullptr.
}

template <>
void getPointFromPointCloud(
    const resources::VoxbloxColorPointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C, resources::RgbaColor* color) {
  CHECK_NOTNULL(point_C);
  CHECK_NOTNULL(point_cloud.points_C);

  DCHECK_GT(point_cloud.points_C->size(), index);
  *point_C = (*point_cloud.points_C)[index];

  if (color != nullptr && point_cloud.points_C != nullptr) {
    DCHECK_GT(point_cloud.colors->size(), index);
    const voxblox::Color& color_in = (*point_cloud.colors)[index];
    resources::RgbaColor& color_out = *color;

    color_out[0] = color_in.r;
    color_out[1] = color_in.g;
    color_out[2] = color_in.b;
    color_out[3] = color_in.a;
  }
}

template <>
void getPointFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C, resources::RgbaColor* color) {
  CHECK_NOTNULL(point_C);

  Eigen::Vector3d& point_C_out = *point_C;

  const size_t real_index = index * 3u;
  DCHECK_GT(point_cloud.xyz.size(), real_index + 2u);
  point_C_out[0] = point_cloud.xyz[real_index];
  point_C_out[1] = point_cloud.xyz[real_index + 1u];
  point_C_out[2] = point_cloud.xyz[real_index + 2u];

  if (color != nullptr) {
    DCHECK_GT(point_cloud.colors.size(), real_index + 2u);
    resources::RgbaColor& color_out = *color;

    color_out[0] = point_cloud.colors[real_index];
    color_out[1] = point_cloud.colors[real_index + 1u];
    color_out[2] = point_cloud.colors[real_index + 2u];
    color_out[3] = 255;
  }
}

template <>
void resizePointCloud(const size_t size, pose::Position3DVector* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  point_cloud->resize(size);
}

template <>
void resizePointCloud(
    const size_t size, resources::VoxbloxColorPointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_NOTNULL(point_cloud->colors);
  CHECK_NOTNULL(point_cloud->points_C);

  point_cloud->points_C->resize(size);
  point_cloud->colors->resize(size);
}

template <>
void resizePointCloud(const size_t size, resources::PointCloud* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  point_cloud->resize(size);
}

void createCameraWithoutDistortion(
    const aslam::Camera& camera,
    aslam::Camera::Ptr* camera_without_distortion) {
  CHECK_NOTNULL(camera_without_distortion);

  switch (camera.getType()) {
    case aslam::Camera::Type::kPinhole:
      *camera_without_distortion = aslam::createCamera<aslam::PinholeCamera>(
          camera.getParameters(), camera.imageWidth(), camera.imageHeight());
      break;
    case aslam::Camera::Type::kUnifiedProjection:
      *camera_without_distortion =
          aslam::createCamera<aslam::UnifiedProjectionCamera>(
              camera.getParameters(), camera.imageWidth(),
              camera.imageHeight());
      break;
    default:
      LOG(FATAL) << "Unknown camera type: " << camera.getType();
  }
}

template <>
size_t getPointCloudSize(const pose::Position3DVector& point_cloud) {
  return point_cloud.size();
}

template <>
size_t getPointCloudSize(const resources::VoxbloxColorPointCloud& point_cloud) {
  const size_t num_points = point_cloud.points_C->size();
  if (point_cloud.colors != nullptr) {
    const size_t num_colors = point_cloud.colors->size();
    if (num_colors > 0u) {
      CHECK_EQ(num_points, num_colors);
    }
  }
  return num_points;
}

template <>
size_t getPointCloudSize(const resources::PointCloud& point_cloud) {
  return point_cloud.size();
}

template <>
bool hasColorInformation(const pose::Position3DVector& /*point_cloud*/) {
  return false;
}

template <>
bool hasColorInformation(const resources::VoxbloxColorPointCloud& point_cloud) {
  return point_cloud.colors->size() == point_cloud.points_C->size();
}

template <>
bool hasColorInformation(const resources::PointCloud& point_cloud) {
  return point_cloud.xyz.size() == point_cloud.colors.size();
}

}  // namespace backend
