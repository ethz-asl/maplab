#ifndef DENSE_RECONSTRUCTION_PMVS_INTERFACE_H_
#define DENSE_RECONSTRUCTION_PMVS_INTERFACE_H_

#include <string>
#include <vector>

#include <aslam/pipeline/undistorter-mapped.h>
#include <glog/logging.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>

#include "dense-reconstruction/pmvs-common.h"
#include "dense-reconstruction/pmvs-config.h"
#include "dense-reconstruction/pmvs-file-utils.h"

DECLARE_uint64(pmvs_num_visual_frames_per_cluster);

namespace dense_reconstruction {

bool exportVIMapToPmvsSfmInputData(
    const PmvsConfig& pmvs_config, vi_map::VIMap* vi_map);

bool exportVIMapToPmvsSfmInputData(
    const PmvsConfig& pmvs_config, const vi_map::MissionIdList& mission_ids,
    vi_map::VIMap* vi_map);

void getObserverPosesFromOptionalCameras(
    const vi_map::VIMap& vi_map, const PmvsConfig& pmvs_settings,
    const vi_map::MissionIdList& mission_ids,
    const ObserverCameraMap& observer_cameras, ObserverPosesMap* observer_poses,
    size_t* num_observers);

void getObserverPosesFromNCamera(
    const vi_map::VIMap& vi_map, const PmvsConfig& pmvs_settings,
    const vi_map::MissionIdList& mission_ids,
    const ObserverCameraMap& observer_cameras, ObserverPosesMap* observer_poses,
    size_t* num_observers);

void getObservedLandmarksAndCovisibilityInformation(
    const vi_map::VIMap& vi_map, const PmvsConfig& config,
    const vi_map::MissionIdList& mission_ids,
    const ObserverCameraMap& observer_cameras,
    const ObserverPosesMap& observer_poses,
    ObservedLandmarks* observed_landmarks);

bool isLandmarkVisibleForObserverCamera(
    const vi_map::VIMap& vi_map, const vi_map::Vertex& vertex,
    const vi_map::LandmarkId& landmark_id, const Eigen::Vector3d& p_G,
    const ObserverCamera& observer_camera, const ObserverPose& observer_pose);

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_PMVS_INTERFACE_H_
