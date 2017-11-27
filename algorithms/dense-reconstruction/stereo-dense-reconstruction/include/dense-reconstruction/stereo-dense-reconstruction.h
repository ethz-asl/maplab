#ifndef DENSE_RECONSTRUCTION_STEREO_DENSE_RECONSTRUCTION_H_
#define DENSE_RECONSTRUCTION_STEREO_DENSE_RECONSTRUCTION_H_

#include <aslam/cameras/camera.h>
#include <map-resources/resource-common.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace dense_reconstruction {

void computeDepthForAllStereoCameras(
    const backend::ResourceType& depth_resource_type, vi_map::VIMap* vi_map);

// With no mission selected, it will process all missions.
void computeDepthForAllStereoCameras(
    const backend::ResourceType& depth_resource_type,
    const vi_map::MissionIdList& selected_mission_ids, vi_map::VIMap* vi_map);

void computeDepthForStereoCamerasOfMission(
    const aslam::CameraId& first_camera_id,
    const aslam::CameraId& second_camera_id,
    const aslam::Transformation& T_C2_C1, const vi_map::MissionId& mission_id,
    const backend::ResourceType& depth_resource_type, vi_map::VIMap* vi_map);

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_STEREO_DENSE_RECONSTRUCTION_H_
