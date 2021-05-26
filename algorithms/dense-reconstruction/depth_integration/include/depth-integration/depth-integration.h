#ifndef DEPTH_INTEGRATION_DEPTH_INTEGRATION_H_
#define DEPTH_INTEGRATION_DEPTH_INTEGRATION_H_

#include <functional>
#include <unordered_set>

#include <vi-map/vi-map.h>
#include <voxblox/core/common.h>

DECLARE_bool(dense_depth_integrator_enable_sigint_breaker);
DECLARE_bool(dense_depth_integrator_use_closest_to_vertex);
DECLARE_bool(dense_depth_integrator_visualize_only_with_known_baseframe);
DECLARE_int64(dense_depth_integrator_timeshift_resource_to_imu_ns);

namespace depth_integration {

// Point cloud integration functions
typedef std::function<void(
    const voxblox::Transformation& /*T_G_S*/,
    const voxblox::Pointcloud& /*points_S*/, const voxblox::Colors& /*colors*/)>
    IntegrationFunctionPointCloudVoxblox;

typedef std::function<void(
    const aslam::Transformation /*T_G_S*/&,
    const resources::PointCloud& /*points_S*/)>
    IntegrationFunctionPointCloudMaplab;

typedef std::function<void(
    const int64_t /*ts_ns*/, const aslam::Transformation /*T_G_S*/&,
    const resources::PointCloud& /*points_S*/)>
    IntegrationFunctionPointCloudMaplabWithTs;

// Set of supported resource types when using the depth integrator with the
// point cloud integration function.
static std::unordered_set<backend::ResourceType, backend::ResourceTypeHash>
    kIntegrationFunctionPointCloudSupportedTypes{
        backend::ResourceType::kRawDepthMap,
        backend::ResourceType::kOptimizedDepthMap,
        backend::ResourceType::kPointCloudXYZ,
        backend::ResourceType::kPointCloudXYZI,
        backend::ResourceType::kPointCloudXYZRGBN,
        backend::ResourceType::kPointCloudXYZL,
        backend::ResourceType::kPointCloudXYZIRT};

// Depth map integration function.
typedef std::function<void(
    const aslam::Transformation& /*T_G_C*/, const aslam::Camera& /*camera*/,
    const cv::Mat& /*depth_image*/, const cv::Mat& /*intensity_image*/)>
    IntegrationFunctionDepthImage;

// Function to determine which resource should be integrated, this can be done
// based on the information about the resource provided to this function.
typedef std::function<bool(
    const int64_t /*timestamp_ns*/, const aslam::Transformation& /*T_G_S*/)>
    ResourceSelectionFunction;

// Set of supported resource types when using the depth integrator with the
// depth map integration function.
static std::unordered_set<backend::ResourceType, backend::ResourceTypeHash>
    kIntegrationFunctionDepthImageSupportedTypes{
        backend::ResourceType::kRawDepthMap,
        backend::ResourceType::kOptimizedDepthMap};

// Calls the integration function for all depth frame resources from the
// selected missions using the integration function. For depth maps you can use
// the 'use_undistorted_camera_for_depth_maps' parameter to determine if it
// should be reprojected with or without the distortion of the camera.
template <typename IntegrationFunctionType>
void integrateAllFrameDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const vi_map::VIMap& vi_map, IntegrationFunctionType integration_function,
    ResourceSelectionFunction resource_selection_function = nullptr);

// Calls the integration function for all optional depth resources from the
// selected missions using the integration function. For depth maps you can use
// the 'use_undistorted_camera_for_depth_maps' parameter to determine if it
// should be reprojected with or without the distortion of the camera.
template <typename IntegrationFunctionType>
void integrateAllSensorDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const vi_map::VIMap& vi_map, IntegrationFunctionType integration_function,
    ResourceSelectionFunction resource_selection_function = nullptr);

// Calls the integration function for all depth (optional and frame) resources.
template <typename IntegrationFunctionType>
void integrateAllDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const vi_map::VIMap& vi_map, IntegrationFunctionType integration_function,
    ResourceSelectionFunction resource_selection_function = nullptr);

// Wrapper that converts the native point cloud type to the type requested by
// the integration function and calls it.
template <typename IntegrationFunctionType>
void integratePointCloud(
    const int64_t timestamp_ns, const aslam::Transformation& T_G_C,
    const resources::PointCloud& points_C,
    IntegrationFunctionType integration_function);

// Wrapper that converts the native depth map type to the type requested by the
// integration function and calls it.
template <typename IntegrationFunctionType>
void integrateDepthMap(
    const int64_t timestamp_ns, const aslam::Transformation& T_G_C,
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    IntegrationFunctionType integration_function);

template <typename IntegrationFunctionType>
bool isSupportedResourceType(const backend::ResourceType& resource_type);

}  // namespace depth_integration

#include "depth-integration/depth-integration-inl.h"

#endif  // DEPTH_INTEGRATION_DEPTH_INTEGRATION_H_
