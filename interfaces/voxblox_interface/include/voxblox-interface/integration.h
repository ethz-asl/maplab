#ifndef VOXBLOX_INTERFACE_INTEGRATION_H_
#define VOXBLOX_INTERFACE_INTEGRATION_H_

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

// Integrates all depth maps from the selected missions into a TSDF voxblox
// grid. For depth maps it assumes the depth map has been created based on an
// undistorted image, therefore it will remove the distortion from the camera to
// reproject the depth map (unless use_distorted_camera is set to true).
bool integrateAllDepthResourcesOfType(
    const vi_map::MissionIdList& mission_ids,
    const backend::ResourceType& input_resource_type,
    const bool use_distorted_camera,
    const voxblox::TsdfIntegratorBase::Config& integrator_config,
    vi_map::VIMap* vi_map, voxblox::TsdfMap* tsdf_map);

// Integrates a 3D point cloud into a TSDF map.
void integratePointCloud(
    const pose::Transformation& T_G_C, const pose::Position3DVector& points_C,
    voxblox::TsdfIntegratorBase* tsdf_integrator);

// Integrates a 3D point cloud into a TSDF map.
void integratePointCloud(
    const pose::Transformation& T_G_C, const resources::PointCloud& points_C,
    voxblox::TsdfIntegratorBase* tsdf_integrator);

// Integrates a color 3D point cloud into a TSDF map.
void integrateColorPointCloud(
    const pose::Transformation& T_G_C, const pose::Position3DVector& points_C,
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

#endif  // VOXBLOX_INTERFACE_INTEGRATION_H_
