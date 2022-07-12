#ifndef VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_
#define VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_

#include <unordered_map>
#include <utility>
#include <vector>
#include <vi-map/vi-map.h>

namespace vi_map {
class VIMap;
typedef std::unordered_map<vi_map::KeypointIdentifier, int>
    KeypointToTrackIdMap;
typedef std::unordered_map<int, vi_map::KeypointIdentifierList>
    TrackIdToKeypointsMap;
}  // namespace vi_map

namespace vi_map_helpers {
double computeSquaredReprojectionError(
    const vi_map::Vertex& vertex, const int frame_idx, const int keypoint_idx,
    const Eigen::Vector3d& landmark_p_C);
void evaluateLandmarkQuality(vi_map::VIMap* map);
void evaluateLandmarkQuality(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* map);
void removeInvalidLandmarkObservations(vi_map::VIMap* map);
void removeInvalidLandmarkObservations(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* map);
void resetLandmarkQualityToUnknown(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* map);
void findAndDetachInferiorQualityTracks(
    vi_map::VIMap* map, const vi_map::MissionId& mission_id,
    vi_map::Landmark* landmark, size_t* num_bad_tracks,
    size_t* num_bad_observations);
}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_
