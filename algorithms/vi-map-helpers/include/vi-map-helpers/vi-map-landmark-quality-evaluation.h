#ifndef VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_
#define VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_

#include <unordered_map>
#include <utility>
#include <vector>

#include <vi-map/vi-map.h>

namespace vi_map {
class VIMap;
typedef std::pair<vi_map::KeypointIdentifier, int> KeypointWithTrackId;
typedef std::vector<KeypointWithTrackId> KeypointWithTrackIdList;
typedef std::unordered_map<int, vi_map::KeypointIdentifierList>
    TrackKeypointMap;
typedef std::vector<TrackKeypointMap> TrackKeypointMapList;
}  // namespace vi_map

namespace vi_map_helpers {
double computeSquaredReprojectionError(
    const vi_map::Vertex& vertex, const int frame_idx, const int keypoint_idx,
    const Eigen::Vector3d& landmark_p_C);
void evaluateLandmarkQuality(vi_map::VIMap* map);
void evaluateLandmarkQuality(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* map);
void resetLandmarkQualityToUnknown(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* map);
void findTracksOfInferiorDuplicateLandmarkObservations(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    vi_map::TrackKeypointMap* tracks_with_keypoints);
void detachTracksFromLandmarks(
    const vi_map::TrackKeypointMap& tracks_with_keypoints, vi_map::VIMap* map);
void initializeNewLandmarksFromTracks(
    const vi_map::TrackKeypointMap& tracks_with_keypoints,
    const vi_map::MissionId& mission_id, vi_map::VIMap* map);

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_LANDMARK_QUALITY_EVALUATION_H_
