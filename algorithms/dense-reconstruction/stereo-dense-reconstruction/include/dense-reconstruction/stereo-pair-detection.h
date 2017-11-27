#ifndef DENSE_RECONSTRUCTION_STEREO_PAIR_DETECTION_H_
#define DENSE_RECONSTRUCTION_STEREO_PAIR_DETECTION_H_

#include <unordered_map>
#include <utility>
#include <vector>

#include <aslam/cameras/camera.h>
#include <map-resources/resource-common.h>
#include <vi-map/vi-map.h>

namespace dense_reconstruction {

struct StereoPairIdentifier {
  aslam::CameraId first_camera_id;
  aslam::CameraId second_camera_id;
  aslam::Transformation T_C2_C1;
};

typedef std::vector<StereoPairIdentifier> StereoPairIdsVector;

typedef std::pair<vi_map::MissionId, StereoPairIdsVector>
    StereoPairIdsPerMission;
typedef std::unordered_map<vi_map::MissionId, StereoPairIdsVector>
    StereoPairsPerMissionMap;

namespace stereo {

// List all the stereo cameras found to provide an overview.
void printStereoCamerasPerMission(
    const StereoPairsPerMissionMap& stereo_camera_ids_per_mission,
    const int verbosity);

// Look for potential stereo cameras in the vi_map based on how suitable they
// are for planar rectification.
void findAllStereoCameras(
    const vi_map::VIMap& vi_map,
    StereoPairsPerMissionMap* stereo_camera_ids_per_mission);

}  // namespace stereo
}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_STEREO_PAIR_DETECTION_H_
