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
    std::vector<std::unordered_map<int, vi_map::KeypointIdentifierList>>
        outlier_track_ids_with_observations_per_thread(num_threads);
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

            // Store landmark observations based on frame and store track ids
            const vi_map::KeypointIdentifierList& landmark_observations =
                landmark.getObservations();
            std::unordered_map<
                vi_map::VisualFrameIdentifier, vi_map::KeypointIdentifierList>
                frames_with_observations;
            std::unordered_map<int, vi_map::KeypointIdentifierList>
                track_ids_with_observations;
            std::unordered_map<vi_map::KeypointIdentifier, int>
                observations_with_track_id;
            for (const auto& keypoint : landmark_observations) {
              vi_map::KeypointIdentifierList& frame_observation =
                  frames_with_observations[keypoint.frame_id];
              frame_observation.push_back(keypoint);
              const vi_map::Vertex& vertex =
                  map->getVertex(keypoint.frame_id.vertex_id);
              const int track_id =
                  vertex.getVisualFrame(keypoint.frame_id.frame_index)
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
              const vi_map::Vertex& vertex =
                  map->getVertex(it->first.vertex_id);
              const Eigen::Vector3d& p_C_fi = map->getLandmark_p_C_fi(
                  landmark_id, vertex, it->first.frame_index);

              // if (p_C_fi[2] <= 0.0) {
              //   local_outlier_landmarks.emplace_back(landmark_id);
              //   break;
              // }
              double min_sq_reprojection_error =
                  std::numeric_limits<double>::max();
              vi_map::KeypointIdentifier min_error_keypoint;
              for (const vi_map::KeypointIdentifier& keypoint : it->second) {
                const double reprojection_error_sq =
                    computeSquaredReprojectionError(
                        vertex, keypoint.frame_id.frame_index,
                        keypoint.keypoint_index, p_C_fi);
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
                  outlier_track_ids.push_back(
                      observations_with_track_id[keypoint]);
                }
              }
            }

            // Store the outlier track ids with the respective observations
            for (const int outlier_track_id : outlier_track_ids) {
              outlier_track_ids_with_observations_per_thread
                  [thread_idx][outlier_track_id] =
                      track_ids_with_observations[outlier_track_id];
            }
            progress_bar.update(++num_processed);
          }
        };

    common::ParallelProcess(
        num_landmarks, evaluator, kAlwaysParallelize, num_threads);

    const vi_map::LandmarkId invalid_landmark_id;

    // Remove all outlier observations from the landmark and set their
    // observed landmark invalid
    for (const auto& outlier_track_ids_with_observations :
         outlier_track_ids_with_observations_per_thread) {
      for (auto it = outlier_track_ids_with_observations.begin();
           it != outlier_track_ids_with_observations.end(); ++it) {
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

}  // namespace vi_map_helpers
