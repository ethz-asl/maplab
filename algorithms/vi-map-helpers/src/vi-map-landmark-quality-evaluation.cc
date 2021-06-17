#include "vi-map-helpers/vi-map-landmark-quality-evaluation.h"

#include <glog/logging.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/threading-helpers.h>
#include <vi-map/landmark-quality-metrics.h>
#include <vi-map/landmark.h>

namespace vi_map_helpers {

double computeSquaredReprojectionError(
    const vi_map::Vertex& vertex, const int frame_idx, const int keypoint_idx,
    const Eigen::Vector3d& landmark_p_C) {
  Eigen::Vector2d reprojected_point;
  aslam::ProjectionResult projection_result =
      vertex.getCamera(frame_idx)->project3(landmark_p_C, &reprojected_point);

  if (projection_result == aslam::ProjectionResult::KEYPOINT_VISIBLE ||
      projection_result ==
          aslam::ProjectionResult::KEYPOINT_OUTSIDE_IMAGE_BOX) {
    return (reprojected_point -
            vertex.getVisualFrame(frame_idx).getKeypointMeasurement(
                keypoint_idx))
        .squaredNorm();
  }
  return std::numeric_limits<double>::max();
}

void evaluateLandmarkQuality(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  constexpr bool kEvaluateLandmarkQuality = true;

  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(map->hasMission(mission_id));

    vi_map::LandmarkIdList landmark_ids;
    map->getAllLandmarkIdsInMission(mission_id, &landmark_ids);
    const size_t num_landmarks = landmark_ids.size();

    VLOG(1) << "Evaluating quality of landmarks of " << num_landmarks
            << " landmarks of mission " << mission_id.hexString();

    const size_t num_threads = common::getNumHardwareThreads();
    vi_map::TrackKeypointMapList outlier_track_ids_with_observations_per_thread(
        num_threads);
    const bool kAlwaysParallelize = false;
    common::MultiThreadedProgressBar progress_bar;
    std::function<void(const std::vector<size_t>&)> evaluator =
        [&landmark_ids, map, &progress_bar,
         &outlier_track_ids_with_observations_per_thread, &num_threads,
         kAlwaysParallelize](const std::vector<size_t>& batch) {
          const size_t num_items = landmark_ids.size();
          size_t thread_idx;
          if (num_items < num_threads * 2 && !kAlwaysParallelize) {
            thread_idx = 0;
          } else {
            const size_t num_items_per_block = std::ceil(
                static_cast<double>(num_items) /
                static_cast<double>(num_threads));
            thread_idx = batch[0] / num_items_per_block;
          }
          progress_bar.setNumElements(batch.size());
          size_t num_processed = 0u;

          for (size_t idx : batch) {
            CHECK_LT(idx, landmark_ids.size());
            const vi_map::LandmarkId& landmark_id = landmark_ids[idx];
            CHECK(landmark_id.isValid());
            vi_map::Landmark& landmark = map->getLandmark(landmark_id);
            landmark.setQuality(
                vi_map::isLandmarkWellConstrained(
                    *map, landmark, kEvaluateLandmarkQuality)
                    ? vi_map::Landmark::Quality::kGood
                    : vi_map::Landmark::Quality::kBad);

            findTracksOfInferiorDuplicateLandmarkObservations(
                *map, landmark,
                &outlier_track_ids_with_observations_per_thread[thread_idx]);

            progress_bar.update(++num_processed);
          }
        };

    common::ParallelProcess(
        num_landmarks, evaluator, kAlwaysParallelize, num_threads);

    vi_map::TrackKeypointMap outlier_track_ids_with_observations;
    for (const auto& outliers_in_thread :
         outlier_track_ids_with_observations_per_thread) {
      outlier_track_ids_with_observations.insert(
          outliers_in_thread.begin(), outliers_in_thread.end());
    }

    detachTracksFromLandmarks(outlier_track_ids_with_observations, map);
    initializeNewLandmarksFromTracks(
        outlier_track_ids_with_observations, mission_id, map);
  }
}

void evaluateLandmarkQuality(vi_map::VIMap* map) {
  vi_map::MissionIdList mission_ids;
  CHECK_NOTNULL(map)->getAllMissionIds(&mission_ids);
  evaluateLandmarkQuality(mission_ids, map);
}

void resetLandmarkQualityToUnknown(
    const vi_map::MissionIdList& mission_ids, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(map->hasMission(mission_id));

    vi_map::LandmarkIdList landmark_ids;
    map->getAllLandmarkIdsInMission(mission_id, &landmark_ids);
    const size_t num_landmarks = landmark_ids.size();

    common::MultiThreadedProgressBar progress_bar;
    std::function<void(const std::vector<size_t>&)> evaluator =
        [&landmark_ids, map, &progress_bar](const std::vector<size_t>& batch) {
          progress_bar.setNumElements(batch.size());
          size_t num_processed = 0u;
          for (size_t idx : batch) {
            CHECK_LT(idx, landmark_ids.size());
            const vi_map::LandmarkId& landmark_id = landmark_ids[idx];
            CHECK(landmark_id.isValid());
            vi_map::Landmark& landmark = map->getLandmark(landmark_id);
            landmark.setQuality(vi_map::Landmark::Quality::kUnknown);
            progress_bar.update(++num_processed);
          }
        };

    static constexpr bool kAlwaysParallelize = false;
    const size_t num_threads = common::getNumHardwareThreads();
    common::ParallelProcess(
        num_landmarks, evaluator, kAlwaysParallelize, num_threads);
  }
}

void findTracksOfInferiorDuplicateLandmarkObservations(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    vi_map::TrackKeypointMap* tracks_with_keypoints) {
  // Store landmark observations based on frame and store track ids
  const vi_map::KeypointIdentifierList& landmark_observations =
      landmark.getObservations();
  std::unordered_map<
      vi_map::VisualFrameIdentifier, vi_map::KeypointIdentifierList>
      frames_with_observations;
  vi_map::TrackKeypointMap track_ids_with_observations;
  std::unordered_map<vi_map::KeypointIdentifier, int>
      observations_with_track_id;
  for (const auto& keypoint : landmark_observations) {
    vi_map::KeypointIdentifierList& frame_observation =
        frames_with_observations[keypoint.frame_id];
    frame_observation.push_back(keypoint);
    const vi_map::Vertex& vertex = map.getVertex(keypoint.frame_id.vertex_id);
    const int track_id = vertex.getVisualFrame(keypoint.frame_id.frame_index)
                             .getTrackId(keypoint.keypoint_index);
    vi_map::KeypointIdentifierList& track_id_observations =
        track_ids_with_observations[track_id];
    track_id_observations.push_back(keypoint);
    observations_with_track_id[keypoint] = track_id;
  }

  std::vector<int> outlier_track_ids;
  static constexpr int kMaxNumSameLandmarkId = 1;
  for (auto it = frames_with_observations.begin();
       it != frames_with_observations.end(); ++it) {
    // If the landmark is only observed once in the frame we're good
    if (it->second.size() <= kMaxNumSameLandmarkId) {
      continue;
    }

    // Otherwise find observation with smallest reprojection error
    const vi_map::Vertex& vertex = map.getVertex(it->first.vertex_id);
    const Eigen::Vector3d& p_C_fi =
        map.getLandmark_p_C_fi(landmark.id(), vertex, it->first.frame_index);

    // if (p_C_fi[2] <= 0.0) {
    //   local_outlier_landmarks.emplace_back(landmark_id);
    //   break;
    // }
    double min_sq_reprojection_error = std::numeric_limits<double>::max();
    vi_map::KeypointIdentifier min_error_keypoint;
    for (const vi_map::KeypointIdentifier& keypoint : it->second) {
      const double reprojection_error_sq = computeSquaredReprojectionError(
          vertex, keypoint.frame_id.frame_index, keypoint.keypoint_index,
          p_C_fi);
      if (reprojection_error_sq < min_sq_reprojection_error) {
        min_error_keypoint = keypoint;
        min_sq_reprojection_error = reprojection_error_sq;
      }
    }
    for (const vi_map::KeypointIdentifier& keypoint : it->second) {
      if (keypoint == min_error_keypoint) {
        continue;
      }
      // Mark the track ids of the observations with non minimal
      // reprojection error as outliers
      if (std::find(
              outlier_track_ids.begin(), outlier_track_ids.end(),
              observations_with_track_id[keypoint]) ==
          outlier_track_ids.end()) {
        outlier_track_ids.push_back(observations_with_track_id[keypoint]);
      }
    }
  }

  // Store the outlier track ids with the respective observations
  for (const int outlier_track_id : outlier_track_ids) {
    (*tracks_with_keypoints)[outlier_track_id] =
        track_ids_with_observations[outlier_track_id];
  }
}
void detachTracksFromLandmarks(
    const vi_map::TrackKeypointMap& tracks_with_keypoints, vi_map::VIMap* map) {
  // Remove all outlier observations from the landmark and set their
  // observed landmark invalid
  static const vi_map::LandmarkId invalid_landmark_id;
  for (auto it = tracks_with_keypoints.begin();
       it != tracks_with_keypoints.end(); ++it) {
    for (const auto& keypoint : it->second) {
      vi_map::Vertex& vertex = map->getVertex(keypoint.frame_id.vertex_id);
      const vi_map::LandmarkId previously_observed_landmark_id =
          vertex.getObservedLandmarkId(keypoint);
      vi_map::Landmark& previously_observed_landmark =
          map->getLandmark(previously_observed_landmark_id);
      previously_observed_landmark.removeObservation(keypoint);
      vertex.setObservedLandmarkId(keypoint, invalid_landmark_id);
    }
  }
}

void initializeNewLandmarksFromTracks(
    const vi_map::TrackKeypointMap& tracks_with_keypoints,
    const vi_map::MissionId& mission_id, vi_map::VIMap* map) {
  std::vector<std::pair<vi_map::KeypointIdentifier, int>>
      keypoints_with_track_id;
  pose_graph::VertexIdList keypoint_vertex_ids;
  for (auto it = tracks_with_keypoints.begin();
       it != tracks_with_keypoints.end(); ++it) {
    for (const auto& keypoint : it->second) {
      keypoint_vertex_ids.push_back(keypoint.frame_id.vertex_id);
      keypoints_with_track_id.push_back(std::make_pair(keypoint, it->first));
    }
  }

  // Sort the keypoints along the pose graph
  vi_map::KeypointWithTrackIdList keypoints_with_track_id_sorted;
  pose_graph::VertexIdList all_vertices_in_mission;
  map->getAllVertexIdsInMissionAlongGraph(mission_id, &all_vertices_in_mission);
  for (const auto& vertex_id_along_graph : all_vertices_in_mission) {
    for (size_t idx = 0u; idx < keypoint_vertex_ids.size(); ++idx) {
      if (vertex_id_along_graph == keypoint_vertex_ids[idx]) {
        keypoints_with_track_id_sorted.push_back(keypoints_with_track_id[idx]);
      }
    }
  }

  // Initialize new landmarks from unused track ids and associate
  // observations with them
  std::unordered_map<int, vi_map::LandmarkId> track_id_to_landmark_id;
  for (const auto& keypoint_with_track_id : keypoints_with_track_id_sorted) {
    const int track_id = keypoint_with_track_id.second;
    const vi_map::KeypointIdentifier& keypoint = keypoint_with_track_id.first;
    // Skip non-tracked landmark observation.
    if (track_id < 0) {
      continue;
    }

    // Check whether this track has already a global landmark id
    // associated.
    const vi_map::LandmarkId* landmark_id_ptr =
        common::getValuePtr(track_id_to_landmark_id, track_id);

    if (landmark_id_ptr != nullptr && map->hasLandmark(*landmark_id_ptr)) {
      map->associateKeypointWithExistingLandmark(
          keypoint.frame_id.vertex_id, keypoint.frame_id.frame_index,
          keypoint.keypoint_index, *landmark_id_ptr);
    } else {
      // Assign a new global landmark id to this track if it hasn't
      // been seen before and add a new landmark to the map.
      vi_map::LandmarkId landmark_id =
          aslam::createRandomId<vi_map::LandmarkId>();
      // operator[] intended as this is either overwriting an old outdated
      // entry or creating a new one.
      track_id_to_landmark_id[track_id] = landmark_id;

      vi_map::KeypointIdentifier keypoint_id;
      keypoint_id.frame_id.frame_index = keypoint.frame_id.frame_index;
      keypoint_id.frame_id.vertex_id = keypoint.frame_id.vertex_id;
      keypoint_id.keypoint_index = keypoint.keypoint_index;
      map->addNewLandmark(landmark_id, keypoint_id);
    }
  }
}

}  // namespace vi_map_helpers
