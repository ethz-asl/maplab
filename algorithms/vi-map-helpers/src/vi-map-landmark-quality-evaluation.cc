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
  constexpr bool kEvaluateLandmarkQuality = true;

  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(map->hasMission(mission_id));

    vi_map::LandmarkIdList landmark_ids;
    map->getAllLandmarkIdsInMission(mission_id, &landmark_ids);
    const size_t num_landmarks = landmark_ids.size();

    VLOG(1) << "Evaluating quality of landmarks of " << num_landmarks
            << " landmarks of mission " << mission_id.hexString();

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
            landmark.setQuality(
                vi_map::isLandmarkWellConstrained(
                    *map, landmark, kEvaluateLandmarkQuality)
                    ? vi_map::Landmark::Quality::kGood
                    : vi_map::Landmark::Quality::kBad);
            progress_bar.update(++num_processed);
          }
        };

    static constexpr bool kAlwaysParallelize = false;
    const size_t num_threads = common::getNumHardwareThreads();
    common::ParallelProcess(
        num_landmarks, evaluator, kAlwaysParallelize, num_threads);
  }
}

void evaluateLandmarkQuality(vi_map::VIMap* map) {
  vi_map::MissionIdList mission_ids;
  CHECK_NOTNULL(map)->getAllMissionIds(&mission_ids);
  evaluateLandmarkQuality(mission_ids, map);
}

void removeInvalidLandmarkObservations(vi_map::VIMap* map) {
  vi_map::MissionIdList mission_ids;
  CHECK_NOTNULL(map)->getAllMissionIds(&mission_ids);
  removeInvalidLandmarkObservations(mission_ids, map);
}

void removeInvalidLandmarkObservations(
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

    size_t num_bad_tracks = 0u;
    size_t num_bad_observations = 0u;
    vi_map::LandmarkIdSet updated_landmark_ids;
    std::mutex stats_counter_mutex;

    // Detect invalid landmark observations and their respective tracks
    std::function<void(const std::vector<size_t>&)> detector =
        [&landmark_ids, map, &mission_id, &progress_bar, &num_bad_tracks,
         &num_bad_observations, &updated_landmark_ids,
         &stats_counter_mutex](const std::vector<size_t>& batch) {
          progress_bar.setNumElements(batch.size());
          size_t num_processed = 0u;
          size_t thread_num_bad_tracks = 0u;
          size_t thread_num_bad_observations = 0u;
          vi_map::LandmarkIdSet thread_updated_landmark_ids;
          for (size_t idx : batch) {
            CHECK_LT(idx, landmark_ids.size());
            const vi_map::LandmarkId& landmark_id = landmark_ids[idx];
            CHECK(landmark_id.isValid());
            vi_map::Landmark& landmark = map->getLandmark(landmark_id);
            const size_t thread_num_bad_tracks_before = thread_num_bad_tracks;
            findAndDetachInferiorQualityTracks(
                map, mission_id, &landmark, &thread_num_bad_tracks,
                &thread_num_bad_observations);
            if (thread_num_bad_tracks > thread_num_bad_tracks_before) {
              thread_updated_landmark_ids.emplace(landmark_id);
            }
            progress_bar.update(++num_processed);
          }

          {
            std::lock_guard<std::mutex> lock(stats_counter_mutex);
            num_bad_tracks += thread_num_bad_tracks;
            num_bad_observations += thread_num_bad_observations;
            updated_landmark_ids.insert(
                thread_updated_landmark_ids.begin(),
                thread_updated_landmark_ids.end());
          }
        };

    VLOG(1) << "Removing invalid landmark observations of mission "
            << mission_id.hexString();
    common::ParallelProcess(
        num_landmarks, detector, kAlwaysParallelize, num_threads);
    VLOG(1) << "Removed " << num_bad_tracks << " bad tracks and "
            << num_bad_observations
            << " observations. Consider reevaluating "
               "the landmark quality using 'elq'.";
    if (num_bad_tracks > 0u || num_bad_observations > 0u) {
      vi_map::LandmarkIdList new_landmark_ids;
      const size_t num_new_landmarks =
          manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(
              mission_id, &new_landmark_ids);
      if (num_bad_tracks != num_new_landmarks) {
        LOG(WARNING) << "The number of detached tracks and new initialized "
                     << "landmarks should match but doesnt' (" << num_bad_tracks
                     << " vs " << num_new_landmarks << "). This might be ok if "
                     << "there were other uninitialized landmarks in the map "
                     << "for another reason.";
      }
      updated_landmark_ids.insert(
          new_landmark_ids.begin(), new_landmark_ids.end());
      landmark_triangulation::retriangulateLandmarksOfMission(
          mission_id, map, updated_landmark_ids, true);
    }
  }
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

void findAndDetachInferiorQualityTracks(
    vi_map::VIMap* map, const vi_map::MissionId& mission_id,
    vi_map::Landmark* landmark, size_t* num_bad_tracks,
    size_t* num_bad_observations) {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(landmark);
  CHECK(mission_id.isValid());
  CHECK_NOTNULL(landmark);
  CHECK_NOTNULL(num_bad_tracks);
  CHECK_NOTNULL(num_bad_observations);

  const vi_map::KeypointIdentifierList& landmark_observations =
      landmark->getObservations();

  // Preprocess the observations of the landmark to obtain per frame and per
  // track information
  std::unordered_map<
      vi_map::VisualFrameIdentifier, vi_map::KeypointIdentifierList>
      frames_with_observations;
  vi_map::TrackIdToKeypointsMap track_id_to_observations;
  vi_map::KeypointToTrackIdMap observation_to_track_id;
  for (const auto& keypoint : landmark_observations) {
    const pose_graph::VertexId& vertex_id = keypoint.frame_id.vertex_id;
    CHECK(vertex_id.isValid());
    const vi_map::Vertex& vertex = map->getVertex(vertex_id);

    // We only care about observations for a specific mission
    if (vertex.getMissionId() != mission_id) {
      continue;
    }

    CHECK(vertex.hasVisualNFrame());
    const int track_id = vertex.getVisualFrame(keypoint.frame_id.frame_index)
                             .getTrackId(keypoint.keypoint_index);

    frames_with_observations[keypoint.frame_id].emplace_back(keypoint);
    observation_to_track_id[keypoint] = track_id;

    // Only store information for keypoints that have a valid track id since if
    // they are bad then the whole track needs to be detached from the landmark
    if (track_id != -1) {
      track_id_to_observations[track_id].emplace_back(keypoint);
    }
  }

  std::unordered_set<int> outlier_track_ids;
  static constexpr int kMaxNumSameLandmarkId = 1;
  for (auto it = frames_with_observations.begin();
       it != frames_with_observations.end(); ++it) {
    // If the landmark is only observed once nothing has to be done.
    if (it->second.size() <= kMaxNumSameLandmarkId) {
      continue;
    }

    const vi_map::Vertex& vertex = map->getVertex(it->first.vertex_id);
    const Eigen::Vector3d& p_C_fi =
        map->getLandmark_p_C_fi(landmark->id(), vertex, it->first.frame_index);

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

      // The removal here can be parallelized since each track belongs only
      // to one landmark and there is no collisions with other threads
      const int track_id = observation_to_track_id[keypoint];
      if (track_id == -1) {
        vi_map::Vertex& keypoint_vertex =
            map->getVertex(keypoint.frame_id.vertex_id);
        landmark->removeObservation(keypoint);
        keypoint_vertex.setObservedLandmarkId(keypoint, vi_map::LandmarkId());
        ++(*num_bad_observations);
      } else {
        if (outlier_track_ids.count(track_id) > 0u) {
          continue;
        }

        const vi_map::KeypointIdentifierList& outlier_keypoints =
            track_id_to_observations[track_id];
        for (const vi_map::KeypointIdentifier& keypoint : outlier_keypoints) {
          vi_map::Vertex& keypoint_vertex =
              map->getVertex(keypoint.frame_id.vertex_id);
          landmark->removeObservation(keypoint);
          keypoint_vertex.setObservedLandmarkId(keypoint, vi_map::LandmarkId());
          ++(*num_bad_observations);
        }

        ++(*num_bad_tracks);

        // Mark this track as already processed so that if another observation
        // with the same track id would cause a removal it will be skipped
        outlier_track_ids.insert(track_id);
      }
    }
  }
}

}  // namespace vi_map_helpers
