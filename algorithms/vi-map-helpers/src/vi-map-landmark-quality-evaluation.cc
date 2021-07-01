#include "vi-map-helpers/vi-map-landmark-quality-evaluation.h"

#include <glog/logging.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/threading-helpers.h>
#include <vi-map-helpers/vi-map-manipulation.h>
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
  VIMapManipulation manipulation(map);
  constexpr bool kEvaluateLandmarkQuality = true;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(map->hasMission(mission_id));

    vi_map::LandmarkIdList landmark_ids;
    map->getAllLandmarkIdsInMission(mission_id, &landmark_ids);
    size_t num_landmarks = landmark_ids.size();

    const size_t num_threads = common::getNumHardwareThreads();
    const bool kAlwaysParallelize = false;
    common::MultiThreadedProgressBar progress_bar;

    vi_map::TrackIdToKeypointsMap outlier_track_ids_with_observations;
    std::mutex outlier_track_ids_mutex;

    // Detect invalid landmark observations and their respective tracks
    std::function<void(const std::vector<size_t>&)> detector =
        [&landmark_ids, map, &mission_id, &progress_bar,
         &outlier_track_ids_with_observations,
         &outlier_track_ids_mutex](const std::vector<size_t>& batch) {
          progress_bar.setNumElements(batch.size());
          size_t num_processed = 0u;
          for (size_t idx : batch) {
            CHECK_LT(idx, landmark_ids.size());
            const vi_map::LandmarkId& landmark_id = landmark_ids[idx];
            CHECK(landmark_id.isValid());
            vi_map::Landmark& landmark = map->getLandmark(landmark_id);

            findTracksOfInferiorDuplicateLandmarkObservations(
                *map, mission_id, landmark,
                &outlier_track_ids_with_observations, &outlier_track_ids_mutex);

            progress_bar.update(++num_processed);
          }
        };

    VLOG(1) << "Removing invalid landmark observations of mission "
            << mission_id.hexString();
    common::ParallelProcess(
        num_landmarks, detector, kAlwaysParallelize, num_threads);

    if (outlier_track_ids_with_observations.size() > 0u) {
      detachTracksFromLandmarks(outlier_track_ids_with_observations, map);
      const size_t num_new_landmarks =
          manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(
              mission_id);
      landmark_triangulation::retriangulateLandmarksOfMission(mission_id, map);
    }

    progress_bar.reset();
    map->getAllLandmarkIdsInMission(mission_id, &landmark_ids);
    num_landmarks = landmark_ids.size();

    std::function<void(const std::vector<size_t>&)> evaluator =
        [&landmark_ids, map, &progress_bar](const std::vector<size_t>& batch) {
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
            progress_bar.update(++num_processed);
          }
        };

    VLOG(1) << "Evaluating quality of landmarks of " << num_landmarks
            << " landmarks of mission " << mission_id.hexString();

    common::ParallelProcess(
        num_landmarks, evaluator, kAlwaysParallelize, num_threads);
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
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const vi_map::Landmark& landmark,
    vi_map::TrackIdToKeypointsMap* track_ids_with_keypoints,
    std::mutex* track_ids_with_keypoints_mutex) {
  const vi_map::KeypointIdentifierList& landmark_observations =
      landmark.getObservations();

  std::unordered_map<
      vi_map::VisualFrameIdentifier, vi_map::KeypointIdentifierList>
      frames_with_observations;
  vi_map::TrackIdToKeypointsMap track_id_to_observations;
  vi_map::KeypointToTrackIdMap observation_to_track_id;
  for (const auto& keypoint : landmark_observations) {
    const pose_graph::VertexId& vertex_id = keypoint.frame_id.vertex_id;
    CHECK(vertex_id.isValid());
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    if (vertex.getMissionId() != mission_id) {
      continue;
    }

    CHECK(vertex.hasVisualNFrame());
    const int track_id = vertex.getVisualFrame(keypoint.frame_id.frame_index)
                             .getTrackId(keypoint.keypoint_index);

    frames_with_observations[keypoint.frame_id].emplace_back(keypoint);
    track_id_to_observations[track_id].emplace_back(keypoint);
    observation_to_track_id[keypoint] = track_id;
  }

  std::unordered_set<int> outlier_track_ids;
  static constexpr int kMaxNumSameLandmarkId = 1;
  for (auto it = frames_with_observations.begin();
       it != frames_with_observations.end(); ++it) {
    // If the landmark is only observed once nothing has to be done.

    if (it->second.size() <= kMaxNumSameLandmarkId) {
      continue;
    }

    const vi_map::Vertex& vertex = map.getVertex(it->first.vertex_id);
    const Eigen::Vector3d& p_C_fi =
        map.getLandmark_p_C_fi(landmark.id(), vertex, it->first.frame_index);

    vi_map::KeypointIdentifier min_error_keypoint;

    // If landmark lies behind the camera plane we remove all observations
    // from this frame from the landmark. Otherwise we keep the observation
    // with the smallest reprojection error.
    if (p_C_fi[2] > 0.0) {
      double min_sq_reprojection_error = std::numeric_limits<double>::max();
      for (const vi_map::KeypointIdentifier& keypoint : it->second) {
        const double reprojection_error_sq = computeSquaredReprojectionError(
            vertex, keypoint.frame_id.frame_index, keypoint.keypoint_index,
            p_C_fi);
        if (reprojection_error_sq < min_sq_reprojection_error) {
          min_error_keypoint = keypoint;
          min_sq_reprojection_error = reprojection_error_sq;
        }
      }
    }

    for (const vi_map::KeypointIdentifier& keypoint : it->second) {
      if (keypoint == min_error_keypoint) {
        continue;
      }

      // Mark the track ids of the observations with non minimal
      // reprojection error as outliers
      outlier_track_ids.insert(observation_to_track_id[keypoint]);
    }
  }

  // Store the outlier track ids with the respective observations
  {
    std::lock_guard<std::mutex> lock(*track_ids_with_keypoints_mutex);
    for (const int outlier_track_id : outlier_track_ids) {
      track_ids_with_keypoints->emplace(
          outlier_track_id, track_id_to_observations[outlier_track_id]);
    }
  }
}

void detachTracksFromLandmarks(
    const vi_map::TrackIdToKeypointsMap& track_ids_with_keypoints,
    vi_map::VIMap* map) {
  // Remove all observations from the landmark and set their
  // observed landmark invalid
  static const vi_map::LandmarkId invalid_landmark_id;
  for (auto it = track_ids_with_keypoints.begin();
       it != track_ids_with_keypoints.end(); ++it) {
    for (const auto& keypoint : it->second) {
      const pose_graph::VertexId& vertex_id = keypoint.frame_id.vertex_id;
      CHECK(vertex_id.isValid());
      CHECK(map->hasVertex(vertex_id));
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      const vi_map::LandmarkId previously_observed_landmark_id =
          vertex.getObservedLandmarkId(keypoint);
      vi_map::Landmark& previously_observed_landmark =
          map->getLandmark(previously_observed_landmark_id);
      previously_observed_landmark.removeObservation(keypoint);
      vertex.setObservedLandmarkId(keypoint, invalid_landmark_id);
      CHECK(!previously_observed_landmark.hasObservation(keypoint));
    }
  }
}

}  // namespace vi_map_helpers
