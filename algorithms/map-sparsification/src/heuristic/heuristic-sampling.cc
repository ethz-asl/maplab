#include "map-sparsification/heuristic/heuristic-sampling.h"

#include <algorithm>
#include <map>
#include <unordered_map>

namespace map_sparsification {
namespace sampling {

void LandmarkSamplingWithCostFunctions::registerScoringFunction(
    const ScoringFunction::ConstPtr& scoring) {
  scoring_functions_.push_back(scoring);
}

void LandmarkSamplingWithCostFunctions::registerCostFunction(
    const SamplingCostFunction::ConstPtr& cost) {
  cost_functions_.push_back(cost);
}

void LandmarkSamplingWithCostFunctions::sampleMapSegment(
    const vi_map::VIMap& map, unsigned int desired_num_landmarks,
    unsigned int /*time_limit_seconds*/,
    const vi_map::LandmarkIdSet& segment_landmark_id_set,
    const pose_graph::VertexIdList& segment_vertex_id_list,
    vi_map::LandmarkIdSet* summary_landmark_ids) {
  CHECK_NOTNULL(summary_landmark_ids);
  *summary_landmark_ids = segment_landmark_id_set;

  LandmarkScoreMap landmark_scores;
  for (const vi_map::LandmarkId& landmark_id : *summary_landmark_ids) {
    double landmark_score = 0.0;
    for (unsigned int i = 0; i < scoring_functions_.size(); ++i) {
      landmark_score += (*scoring_functions_[i])(landmark_id, map);
    }
    CHECK(landmark_scores.emplace(landmark_id, landmark_score).second);
  }

  KeyframeKeypointCountMap keyframe_keypoint_counts;
  for (const pose_graph::VertexId& vertex_id : segment_vertex_id_list) {
    vi_map::LandmarkIdList vertex_all_observed_landmark_ids;
    map.getVertex(vertex_id).getAllObservedLandmarkIds(
        &vertex_all_observed_landmark_ids);

    unsigned int num_valid_keypoints = 0;
    for (const vi_map::LandmarkId& landmark_id :
         vertex_all_observed_landmark_ids) {
      if (landmark_id.isValid()) {
        if (segment_landmark_id_set.count(landmark_id) > 0) {
          ++num_valid_keypoints;
        }
      }
    }
    CHECK(
        keyframe_keypoint_counts.emplace(vertex_id, num_valid_keypoints)
            .second);
  }

  const size_t kNumInitialLandmarks = summary_landmark_ids->size();
  const int kNumLandmarksToRemove =
      summary_landmark_ids->size() - desired_num_landmarks;
  LOG(INFO) << "Will remove " << kNumLandmarksToRemove << " out of "
            << summary_landmark_ids->size() << " landmarks.";

  while (summary_landmark_ids->size() > desired_num_landmarks) {
    // Remove half but no more than:
    // * number left to reach the desired number
    // * 10 percent of the initial landmark number
    const unsigned int num_landmarks_to_remove = std::min(
        std::min(
            summary_landmark_ids->size() / 2u,
            summary_landmark_ids->size() - desired_num_landmarks),
        kNumInitialLandmarks / 10 + 1);

    LOG(INFO) << "Iteration: will remove " << num_landmarks_to_remove
              << " landmark IDs out of " << summary_landmark_ids->size();

    unsigned int num_landmarks_removed_in_iter = 0;
    while (num_landmarks_removed_in_iter < num_landmarks_to_remove) {
      VLOG(3) << "\tInner iteration, already removed: "
              << num_landmarks_removed_in_iter;

      LandmarkScoreMap landmark_costs;
      unsigned int num_zero_cost_landmarks = 0;
      // Evaluate cost for each (still present) landmark.
      for (const vi_map::LandmarkId& landmark_id : *summary_landmark_ids) {
        double landmark_cost_values = 0.0;
        for (unsigned int i = 0; i < cost_functions_.size(); ++i) {
          landmark_cost_values +=
              (*cost_functions_[i])(landmark_id, map, keyframe_keypoint_counts);
        }
        CHECK(landmark_costs.emplace(landmark_id, landmark_cost_values).second);
      }

      std::vector<StoreLandmarkIdScorePair> sorted_scores_and_costs;
      for (const vi_map::LandmarkId& landmark_id : *summary_landmark_ids) {
        // Cost is added as it represents the cost of removal of a particular
        // landmark.
        double score_and_cost =
            landmark_scores[landmark_id] + landmark_costs[landmark_id];
        sorted_scores_and_costs.push_back(
            std::make_pair(landmark_id, score_and_cost));
      }

      // Sort the vector with scores, descending.
      std::sort(
          sorted_scores_and_costs.begin(), sorted_scores_and_costs.end(),
          [](const StoreLandmarkIdScorePair& lhs,
             const StoreLandmarkIdScorePair& rhs) {
            return lhs.second > rhs.second;
          });

      // Count zero cost landmarks within this inner iteration.
      for (unsigned int i = sorted_scores_and_costs.size() - 1;
           i >= sorted_scores_and_costs.size() - num_landmarks_to_remove; --i) {
        const vi_map::LandmarkId& store_landmark_id =
            sorted_scores_and_costs[i].first;
        if (landmark_costs[store_landmark_id] == 0.0) {
          ++num_zero_cost_landmarks;
        }
      }

      pose_graph::VertexIdSet sparsified_keyframes;
      // Cut kNumLandmarkToRemovePerIter with worst total score.
      for (unsigned int i = sorted_scores_and_costs.size() - 1;
           i >= sorted_scores_and_costs.size() - num_landmarks_to_remove; --i) {
        const vi_map::LandmarkId& store_landmark_id =
            sorted_scores_and_costs[i].first;

        // There are still some landmarks with zero cost of removal, so
        // no need to remove the current one. Let's continue.
        if (num_zero_cost_landmarks > 0u &&
            landmark_costs[store_landmark_id] > 0.0) {
          continue;
        }

        pose_graph::VertexIdSet store_landmark_id_observers;
        map.getLandmarkObserverVertices(
            store_landmark_id, &store_landmark_id_observers);

        bool are_landmark_observers_already_sparsified = false;
        for (const pose_graph::VertexId& vertex_id :
             store_landmark_id_observers) {
          if (sparsified_keyframes.count(vertex_id) > 0u) {
            // At least one of the observer keyframes of the current landmark
            // was already affected by previous landmark deletions.
            are_landmark_observers_already_sparsified = true;
            break;
          }
        }

        if (!are_landmark_observers_already_sparsified) {
          ++num_landmarks_removed_in_iter;

          sparsified_keyframes.insert(
              store_landmark_id_observers.begin(),
              store_landmark_id_observers.end());

          for (const pose_graph::VertexId& vertex_id :
               store_landmark_id_observers) {
            // Verify if the vertex belongs to the current segment.
            if (keyframe_keypoint_counts.count(vertex_id) > 0u) {
              --keyframe_keypoint_counts[vertex_id];
            }
          }

          summary_landmark_ids->erase(sorted_scores_and_costs[i].first);
        }
      }
    }
  }
}

}  // namespace sampling
}  // namespace map_sparsification
