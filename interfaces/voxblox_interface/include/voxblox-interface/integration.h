#ifndef VOXBLOX_INTERFACE_INTEGRATION_H_
#define VOXBLOX_INTERFACE_INTEGRATION_H_

#include <unordered_set>

#include <vi-map/vi-map.h>
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>

namespace voxblox_interface {

// Creates a TSDF map by integrating all good quality landmarks from a VI map.
// The VI map landmarks need to be retriangulated beforehand.
void integrateAllLandmarks(
    const vi_map::VIMap& vi_map,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    voxblox::TsdfMap* tsdf_map);

static std::unordered_set<backend::ResourceType, backend::ResourceTypeHash>
    kSupportedDepthInputTypes{
        backend::ResourceType::kRawDepthMap,
        backend::ResourceType::kOptimizedDepthMap,
        backend::ResourceType::kPointCloudXYZ,
        backend::ResourceType::kPointCloudXYZI,
        backend::ResourceType::kPointCloudXYZRGBN,
    };

// Integrates all depth frame resources from the selected missions into a TSDF
// voxblox grid. For depth maps you can use the 'use_distorted_camera' parameter
// to determine if it should be reprojected with or without the distortion of
// the camera.
bool integrateAllFrameDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    const vi_map::VIMap& vi_map, voxblox::TsdfMap* tsdf_map);

// Integrates all optional depth resources from the selected missions into a
// TSDF voxblox grid. For depth maps you can use the 'use_distorted_camera'
// parameter to determine if it should be reprojected with or without the
// distortion of the camera. Depending on whether this function is called with
// vi_map::SensorId or aslam::CameraId, it will either integrate all optional
// depth resources associated to a Sensor or a Camera.
template <typename SensorOrCameraId>
bool integrateAllOptionalSensorDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    const vi_map::VIMap& vi_map, voxblox::TsdfMap* tsdf_map);

// Integrate all depth (optional and frame) resources.
bool integrateAllDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_undistorted_camera_for_depth_maps,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    const vi_map::VIMap& vi_map, voxblox::TsdfMap* tsdf_map);

// Integrates a 3D point cloud into a TSDF map.
void integratePointCloud(
    const pose::Transformation& T_G_C, const voxblox::Pointcloud& points_C,
    voxblox::TsdfIntegratorBase* tsdf_integrator);

// Integrates a 3D point cloud into a TSDF map.
void integratePointCloud(
    const pose::Transformation& T_G_C, const resources::PointCloud& points_C,
    voxblox::TsdfIntegratorBase* tsdf_integrator);

// Integrates a color 3D point cloud into a TSDF map.
void integrateColorPointCloud(
    const pose::Transformation& T_G_C, const voxblox::Pointcloud& points_C,
    const voxblox::Colors& colors,
    voxblox::TsdfIntegratorBase* tsdf_integrator);

// Integrates a depth map into a TSDF map.
void integrateDepthMap(
    const pose::Transformation& T_G_C, const cv::Mat& depth_map,
    const aslam::Camera& camera, voxblox::TsdfIntegratorBase* tsdf_integrator);

// Integrates a depth map with a color image into a TSDF map.
void integrateDepthMap(
    const pose::Transformation& T_G_C, const cv::Mat& depth_map,
    const cv::Mat& image, const aslam::Camera& camera,
    voxblox::TsdfIntegratorBase* tsdf_integrator);

}  // namespace voxblox_interface

#include "voxblox-interface/integration-inl.h"

#endif  // VOXBLOX_INTERFACE_INTEGRATION_H_
