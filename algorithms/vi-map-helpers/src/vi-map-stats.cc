#include "vi-map-helpers/vi-map-stats.h"

#include <fstream>  // NOLINT
#include <sstream>  // NOLINT

#include <maplab-common/file-system-tools.h>
#include <maplab-common/progress-bar.h>
#include <posegraph/unique-id.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

VIMapStats::VIMapStats(const vi_map::VIMap& map)
    : map_(map), map_queries_(map) {}

std::string VIMapStats::toString() const {
  std::ostringstream oss;

  statistics::Accumulatord landmark_associated_keypoint_ratio;
  getLandmarkAssociatedKeypointsRatio(&landmark_associated_keypoint_ratio);
  oss << "Keypoints per Vertex associated with a landmark: "
      << landmark_associated_keypoint_ratio.min() << " "
      << landmark_associated_keypoint_ratio.Mean() << " "
      << landmark_associated_keypoint_ratio.max() << std::endl;

  statistics::Accumulatord landmark_observer_count;
  getLandmarkObserverCount(&landmark_observer_count);
  oss << "Observations per landmark: " << landmark_observer_count.min() << " "
      << landmark_observer_count.Mean() << " " << landmark_observer_count.max()
      << std::endl;

  std::unordered_map<vi_map::MissionId, bool>
      are_landmark_associations_track_scoped;
  areLandmarkAssociationsTrackScoped(&are_landmark_associations_track_scoped);
  for (const std::unordered_map<vi_map::MissionId, bool>::value_type&
           mission_result : are_landmark_associations_track_scoped) {
    if (mission_result.second) {
      oss << "Landmark associations of mission " << mission_result.first
          << " are track scoped." << std::endl;
    } else {
      oss << "Landmark associations of mission " << mission_result.first
          << " are NOT track scoped." << std::endl;
    }
  }

  return oss.str();
}

void VIMapStats::getLandmarkAssociatedKeypointsRatio(
    statistics::Accumulatord* accumulator) const {
  CHECK_NOTNULL(accumulator);

  map_.forEachVisualFrame(
      [accumulator](
          const aslam::VisualFrame& visual_frame, const vi_map::Vertex& vertex,
          size_t frame_idx) {
        const Eigen::Matrix2Xd& image_points_distorted =
            visual_frame.getKeypointMeasurements();
        statistics::Accumulatord this_frame_ratio;
        for (int i = 0; i < image_points_distorted.cols(); ++i) {
          const vi_map::LandmarkId& landmark_id =
              vertex.getObservedLandmarkId(frame_idx, i);
          this_frame_ratio.Add((landmark_id.isValid() ? 1. : 0.));
        }
        accumulator->Add(this_frame_ratio.Mean());
      });
}

void VIMapStats::getLandmarkObserverCount(
    statistics::Accumulatord* accumulator) const {
  CHECK_NOTNULL(accumulator);

  map_.forEachLandmark([accumulator](const vi_map::Landmark& landmark) {
    accumulator->Add(landmark.numberOfObservations());
  });
}

void VIMapStats::areLandmarkAssociationsTrackScoped(
    std::unordered_map<vi_map::MissionId, bool>* result) const {
  CHECK_NOTNULL(result)->clear();
  vi_map::MissionIdList all_mission_ids;
  map_.getAllMissionIds(&all_mission_ids);
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    result->emplace(mission_id, areLandmarkAssociationsTrackScoped(mission_id));
  }
}

bool VIMapStats::areLandmarkAssociationsTrackScoped(
    const vi_map::MissionId& mission_id) const {
  CHECK(map_.hasMission(mission_id));
  enum class State { kTracking, kComplete };
  typedef std::unordered_map<vi_map::LandmarkId, State> LandmarkStateMap;
  LandmarkStateMap landmark_states;

  bool result = true;
  map_queries_.forIdsOfObservedLandmarksOfEachVertexAlongGraphWhile(
      mission_id,
      [&](const vi_map::LandmarkIdSet& landmark_ids) -> bool {
        for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
          LandmarkStateMap::iterator found =
              landmark_states.find(landmark_id);
          if (found != landmark_states.end()) {
            if (found->second == State::kComplete) {
              result = false;
              return false;
            }
          } else {
            landmark_states[landmark_id] = State::kTracking;
          }
        }
        for (LandmarkStateMap::value_type& landmark_state : landmark_states) {
          if (landmark_ids.count(landmark_state.first) == 0) {
            landmark_state.second = State::kComplete;
          }
        }
        return true;
      });
  return result;
}

void VIMapStats::getNumLandmarksObservedByOtherMissionsForEachVertexAlongGraph(
    const vi_map::MissionId& mission_id, std::vector<size_t>* result) const {
  CHECK_NOTNULL(result)->clear();
  pose_graph::VertexIdList ordered_vertices_of_mission;
  map_.getAllVertexIdsInMissionAlongGraph(
      mission_id, &ordered_vertices_of_mission);
  result->resize(ordered_vertices_of_mission.size());

  VLOG(3) << "Counting amount of landmarks observed by other missions...";
  common::ProgressBar progress_bar(ordered_vertices_of_mission.size());
  // DO NOT PARALLELIZE (not worth due to cache mutexes)!
  for (size_t i = 0u; i < ordered_vertices_of_mission.size(); ++i) {
    VIMapQueries::VertexCommonLandmarksCountVector common_landmarks;
    map_queries_.getVerticesWithCommonLandmarks(
        ordered_vertices_of_mission[i], 0, &common_landmarks);

    (*result)[i] = 0u;
    for (const VIMapQueries::VertexCommonLandmarksCount& count :
         common_landmarks) {
      if (map_.getVertex(count.vertex_id).getMissionId() != mission_id) {
        (*result)[i] += count.in_common;
      }
    }
    progress_bar.increment();
  }
}

}  // namespace vi_map_helpers
