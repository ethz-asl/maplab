#include "depth-integration/depth-integration.h"

#include <algorithm>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/resource-conversion.h>
#include <map-resources/temporal-resource-id-buffer.h>
#include <maplab-common/progress-bar.h>
#include <maplab-common/sigint-breaker.h>
#include <posegraph/unique-id.h>
#include <vi-map/landmark.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>

DEFINE_bool(
    dense_depth_integrator_use_closest_to_vertex, false,
    "If enabled, the depth integrator is using only depth resources that are "
    "closest in time to a vertex.");

DEFINE_bool(
    dense_depth_integrator_visualize_only_with_known_baseframe, false,
    "If enabled, the depth integrator is only integrating missions that have a "
    "known baseframe.");

DEFINE_bool(
    dense_depth_integrator_enable_sigint_breaker, true,
    "If enabled, the depth integrator can be interrupted with Ctrl-C. Should "
    "be disabled if depth integration is used in a headless console mode, such "
    "as as part of the maplab server.");

DEFINE_int64(
    dense_depth_integrator_timeshift_resource_to_imu_ns, 0,
    "Timeshift that is applied to the resource timestamp. The shift is applied "
    "as follows: t_resource_new = t_resource + shift.");

namespace depth_integration {

template <>
void integratePointCloud(
    const int64_t /*timestamp*/, const aslam::Transformation& T_G_C,
    const resources::PointCloud& points_C,
    IntegrationFunctionPointCloudVoxblox integration_function) {
  CHECK(integration_function);

  voxblox::Pointcloud tmp_points_C;
  voxblox::Colors tmp_colors;

  resources::VoxbloxColorPointCloud voxblox_point_cloud;
  voxblox_point_cloud.points_C = &tmp_points_C;
  voxblox_point_cloud.colors = &tmp_colors;
  CHECK(backend::convertPointCloudType(points_C, &voxblox_point_cloud));

  // If there is no color, create an empty vector of matchin size.
  if (tmp_colors.empty()) {
    tmp_colors.resize(tmp_points_C.size());
  }

  const Eigen::Matrix<voxblox::FloatingPoint, 4, 4> T_G_C_mat =
      T_G_C.getTransformationMatrix().cast<voxblox::FloatingPoint>();
  const voxblox::Transformation T_G_C_voxblox(T_G_C_mat);

  // Call the voxblox integration function.
  integration_function(T_G_C_voxblox, tmp_points_C, tmp_colors);
}

template <>
void integratePointCloud(
    const int64_t /*timestamp*/, const aslam::Transformation& T_G_C,
    const resources::PointCloud& points_C,
    IntegrationFunctionPointCloudMaplab integration_function) {
  CHECK(integration_function);
  integration_function(T_G_C, points_C);
}

template <>
void integratePointCloud(
    const int64_t /*timestamp*/, const aslam::Transformation& /*T_G_C*/,
    const resources::PointCloud& /*points_C*/,
    IntegrationFunctionDepthImage /*integration_function*/) {
  LOG(WARNING) << "Cannot integrate point cloud type resources using the depth "
                  "map integration function! Skipping";
}

template <>
void integrateDepthMap(
    const int64_t /*timestamp*/, const aslam::Transformation& T_G_C,
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    IntegrationFunctionPointCloudVoxblox integration_function) {
  CHECK(integration_function);
  CHECK_EQ(CV_MAT_TYPE(depth_map.type()), CV_16UC1);
  CHECK_EQ(CV_MAT_TYPE(image.type()), CV_8UC1);

  voxblox::Pointcloud point_cloud;
  voxblox::Colors colors;
  if (image.empty()) {
    backend::convertDepthMapWithImageToPointCloud(
        depth_map, image, camera, &point_cloud, &colors);
  } else {
    backend::convertDepthMapToPointCloud(depth_map, camera, &point_cloud);
    colors.resize(point_cloud.size());
  }

  const Eigen::Matrix<voxblox::FloatingPoint, 4, 4> T_G_C_mat =
      T_G_C.getTransformationMatrix().cast<voxblox::FloatingPoint>();
  const voxblox::Transformation T_G_C_voxblox(T_G_C_mat);

  CHECK_EQ(point_cloud.size(), colors.size());
  integration_function(T_G_C_voxblox, point_cloud, colors);
}

template <>
void integrateDepthMap(
    const int64_t /*timestamp*/, const aslam::Transformation& T_G_C,
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    IntegrationFunctionPointCloudMaplab integration_function) {
  CHECK(integration_function);
  CHECK_EQ(CV_MAT_TYPE(depth_map.type()), CV_16UC1);
  CHECK_EQ(CV_MAT_TYPE(image.type()), CV_8UC1);

  resources::PointCloud point_cloud;
  if (image.empty()) {
    backend::convertDepthMapToPointCloud(depth_map, camera, &point_cloud);
  } else {
    backend::convertDepthMapToPointCloud(
        depth_map, image, camera, &point_cloud);
  }

  integration_function(T_G_C, point_cloud);
}

template <>
void integrateDepthMap(
    const int64_t /*timestamp*/, const aslam::Transformation& T_G_C,
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    IntegrationFunctionDepthImage integration_function) {
  CHECK(integration_function);
  CHECK_EQ(CV_MAT_TYPE(depth_map.type()), CV_16UC1);
  CHECK_EQ(CV_MAT_TYPE(image.type()), CV_8UC1);

  integration_function(T_G_C, camera, depth_map, image);
}

template <>
bool isSupportedResourceType<IntegrationFunctionPointCloudMaplab>(
    const backend::ResourceType& resource_type) {
  return kIntegrationFunctionPointCloudSupportedTypes.count(resource_type) > 0u;
}

template <>
bool isSupportedResourceType<IntegrationFunctionPointCloudVoxblox>(
    const backend::ResourceType& resource_type) {
  return kIntegrationFunctionPointCloudSupportedTypes.count(resource_type) > 0u;
}

template <>
bool isSupportedResourceType<IntegrationFunctionDepthImage>(
    const backend::ResourceType& resource_type) {
  return kIntegrationFunctionDepthImageSupportedTypes.count(resource_type) > 0u;
}

}  // namespace depth_integration
