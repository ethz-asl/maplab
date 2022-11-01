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

struct StereoPairIdentifierHash {
  std::size_t operator()(const StereoPairIdentifier& k) const {
    return std::hash<aslam::CameraId>()(k.first_camera_id) ^
           (std::hash<aslam::CameraId>()(k.second_camera_id) << 1);
  }
};

struct StereoPairIdentifierEqual {
  bool operator()(
      const StereoPairIdentifier& lhs, const StereoPairIdentifier& rhs) const {
    return lhs.first_camera_id == rhs.first_camera_id &&
           lhs.second_camera_id == rhs.second_camera_id;
  }
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

// Look for potential stereo cameras in a camera rig.
void findAllStereoCamerasForNCamera(
    const aslam::NCamera& camera_rig, StereoPairIdsVector* stereo_pairs);
}  // namespace stereo
}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_STEREO_PAIR_DETECTION_H_
