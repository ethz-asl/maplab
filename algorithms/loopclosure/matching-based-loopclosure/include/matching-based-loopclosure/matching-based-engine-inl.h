#ifndef MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_INL_H_
#define MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_INL_H_

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <aslam/common/reader-writer-lock.h>
#include <vi-map/unique-id.h>

#include "matching-based-loopclosure/matching-based-engine.h"

namespace std {
template <>
struct hash<vi_map::FrameKeyPointToStructureMatch> {
  std::size_t operator()(
      const vi_map::FrameKeyPointToStructureMatch& value) const {
    const std::size_t h0(
        std::hash<vi_map::KeypointIdentifier>()(value.keypoint_id_query));
    const std::size_t h1(
        std::hash<vi_map::VisualFrameIdentifier>()(value.keyframe_id_result));
    const std::size_t h2(
        std::hash<vi_map::LandmarkId>()(value.landmark_result));
    return h0 ^ h1 ^ h2;
  }
};

template <>
struct hash<std::pair<vi_map::KeypointIdentifier, vi_map::LandmarkId>> {
  std::size_t operator()(
      const std::pair<vi_map::KeypointIdentifier, vi_map::LandmarkId>& value)
      const {
    const std::size_t h1(std::hash<vi_map::KeypointIdentifier>()(value.first));
    const std::size_t h2(std::hash<vi_map::LandmarkId>()(value.second));
    return h1 ^ h2;
  }
};
}  // namespace std

namespace matching_based_loopclosure {

template <typename IdType>
void MatchingBasedLoopDetector::doCovisibilityFiltering(
    const loop_closure::IdToMatches<IdType>& id_to_matches_map,
    const bool make_matches_unique,
    loop_closure::FrameToMatches* frame_matches_ptr,
    std::mutex* frame_matches_mutex) const {
  // WARNING: Do not clear frame matches. It is intended that new matches can
  // be added to already existing matches. The mutex passed to the function
  // can be nullptr, in which case locking is disabled.
  CHECK_NOTNULL(frame_matches_ptr);
  loop_closure::FrameToMatches& frame_matches = *frame_matches_ptr;

  typedef int ComponentId;
  constexpr ComponentId kInvalidComponentId = -1;
  typedef std::unordered_map<loop_closure::Match, ComponentId>
      MatchesToComponents;
  typedef std::unordered_map<ComponentId,
                             std::unordered_set<loop_closure::Match>>
      Components;
  typedef std::unordered_map<vi_map::LandmarkId,
                             std::vector<loop_closure::Match>>
      LandmarkMatches;

  const size_t num_matches_to_filter =
      loop_closure::getNumberOfMatches(id_to_matches_map);
  if (num_matches_to_filter == 0u) {
    return;
  }

  MatchesToComponents matches_to_components;
  LandmarkMatches landmark_matches;
  // To avoid rehashing, we reserve at least twice the number of elements.
  matches_to_components.reserve(num_matches_to_filter * 2u);
  // Reserving the number of matches is still conservative because the number
  // of matched landmarks is smaller than the number of matches.
  landmark_matches.reserve(num_matches_to_filter);

  for (const typename loop_closure::IdToMatches<IdType>::value_type&
           id_matches_pair : id_to_matches_map) {
    for (const loop_closure::Match& match : id_matches_pair.second) {
      landmark_matches[match.landmark_result].emplace_back(match);
      matches_to_components.emplace(match, kInvalidComponentId);
    }
  }

  IdToScoreMap<IdType> id_to_score_map;
  computeRelevantIdsForFiltering(id_to_matches_map, &id_to_score_map);

  ComponentId count_component_index = 0;
  size_t max_component_size = 0u;
  ComponentId max_component_id = kInvalidComponentId;
  Components components;
  for (const MatchesToComponents::value_type& match_to_component :
       matches_to_components) {
    if (match_to_component.second != kInvalidComponentId)
      continue;
    ComponentId component_id = count_component_index++;

    // Find the largest set of keyframes connected by landmark covisibility.
    std::queue<loop_closure::Match> exploration_queue;
    exploration_queue.push(match_to_component.first);
    while (!exploration_queue.empty()) {
      const loop_closure::Match& exploration_match = exploration_queue.front();

      if (skipMatch(id_to_score_map, exploration_match)) {
        exploration_queue.pop();
        continue;
      }

      const MatchesToComponents::iterator exploration_match_and_component =
          matches_to_components.find(exploration_match);
      CHECK(exploration_match_and_component != matches_to_components.end());

      if (exploration_match_and_component->second == kInvalidComponentId) {
        // Not part of a connected component.
        exploration_match_and_component->second = component_id;
        components[component_id].insert(exploration_match);
        // Mark all observations (which are matches) from this ID (keyframe or
        // vertex) as visited.
        const typename loop_closure::IdToMatches<IdType>::const_iterator
            id_and_matches =
                getIteratorForMatch(id_to_matches_map, exploration_match);
        CHECK(id_and_matches != id_to_matches_map.cend());
        const std::vector<loop_closure::Match>& id_matches =
            id_and_matches->second;
        for (const loop_closure::Match& id_match : id_matches) {
          matches_to_components[id_match] = component_id;
          components[component_id].insert(id_match);

          // Put all observations of this landmark on the stack.
          const std::vector<loop_closure::Match>& lm_matches =
              landmark_matches[id_match.landmark_result];
          for (const loop_closure::Match& lm_match : lm_matches) {
            if (matches_to_components[lm_match] == kInvalidComponentId) {
              exploration_queue.push(lm_match);
            }
          }
        }

        if (components[component_id].size() > max_component_size) {
          max_component_size = components[component_id].size();
          max_component_id = component_id;
        }
      }
      exploration_queue.pop();
    }
  }

  // Only store the structure matches if there is a relevant amount of them.
  if (max_component_size > settings_.min_verify_matches_num) {
    const std::unordered_set<loop_closure::Match>& matches_max_component =
        components[max_component_id];
    typedef std::pair<loop_closure::KeypointId, vi_map::LandmarkId>
        KeypointLandmarkPair;
    std::unordered_set<KeypointLandmarkPair> used_matches;
    if (make_matches_unique) {
      // Conservative reserve to avoid rehashing.
      used_matches.reserve(2u * matches_max_component.size());
    }
    auto lock = (frame_matches_mutex == nullptr)
                    ? std::unique_lock<std::mutex>()
                    : std::unique_lock<std::mutex>(*frame_matches_mutex);
    for (const loop_closure::Match& structure_match : matches_max_component) {
      if (make_matches_unique) {
        // clang-format off
        const bool is_match_unique = used_matches.emplace(
                structure_match.keypoint_id_query,
                structure_match.landmark_result).second;
        // clang-format on
        if (!is_match_unique) {
          // Skip duplicate (keypoint to landmark) structure matches.
          continue;
        }
      }
      frame_matches[structure_match.keypoint_id_query.frame_id].push_back(
          structure_match);
    }
  }
}

template <>
typename loop_closure::FrameToMatches::const_iterator
MatchingBasedLoopDetector::getIteratorForMatch(
    const loop_closure::FrameToMatches& frame_to_matches,
    const loop_closure::Match& match) const {
  return frame_to_matches.find(match.keyframe_id_result);
}

template <>
typename loop_closure::VertexToMatches::const_iterator
MatchingBasedLoopDetector::getIteratorForMatch(
    const loop_closure::VertexToMatches& vertex_to_matches,
    const loop_closure::Match& match) const {
  return vertex_to_matches.find(match.keyframe_id_result.vertex_id);
}

template <>
bool MatchingBasedLoopDetector::skipMatch(
    const IdToScoreMap<loop_closure::KeyframeId>& frame_to_score_map,
    const loop_closure::Match& match) const {
  const typename IdToScoreMap<loop_closure::KeyframeId>::const_iterator iter =
      frame_to_score_map.find(match.keyframe_id_result);
  return iter == frame_to_score_map.cend();
}

template <>
bool MatchingBasedLoopDetector::skipMatch(
    const IdToScoreMap<loop_closure::VertexId>& /* vertex_to_score_map */,
    const loop_closure::Match& /* match */) const {
  // We do not skip vertices because we want to consider all keyframes that
  // passed the keyframe covisibility filtering step.
  return false;
}

template <>
void MatchingBasedLoopDetector::computeRelevantIdsForFiltering(
    const loop_closure::FrameToMatches& frame_to_matches,
    IdToScoreMap<loop_closure::KeyframeId>* frame_to_score_map) const {
  CHECK_NOTNULL(frame_to_score_map)->clear();
  // Score each keyframe, then take the part which is in the
  // top fraction and allow only matches to landmarks which are associated with
  // these keyframes.

  scoring::ScoreList<loop_closure::KeyframeId> score_list;
  timing::Timer timer_scoring("Loop Closure: scoring for covisibility filter");
  CHECK(compute_keyframe_scores_);
  compute_keyframe_scores_(
      frame_to_matches, keyframe_id_to_num_descriptors_,
      static_cast<size_t>(NumDescriptors()), &score_list);
  timer_scoring.Stop();

  // We want to take matches from the best n score keyframes, but make sure
  // that we evaluate at minimum a given number.
  constexpr size_t kNumMinimumScoreIdsToEvaluate = 4u;
  size_t num_score_ids_to_evaluate = std::max<size_t>(
      static_cast<size_t>(score_list.size() * settings_.fraction_best_scores),
      kNumMinimumScoreIdsToEvaluate);
  // Ensure staying in bounds.
  num_score_ids_to_evaluate =
      std::min<size_t>(num_score_ids_to_evaluate, score_list.size());
  std::nth_element(
      score_list.begin(), score_list.begin() + num_score_ids_to_evaluate,
      score_list.end(),
      [](const scoring::Score<loop_closure::KeyframeId>& lhs,
         const scoring::Score<loop_closure::KeyframeId>& rhs) -> bool {
        return lhs.second > rhs.second;
      });
  frame_to_score_map->insert(
      score_list.begin(), score_list.begin() + num_score_ids_to_evaluate);
}

template <>
void MatchingBasedLoopDetector::computeRelevantIdsForFiltering(
    const loop_closure::VertexToMatches& /* vertex_to_matches */,
    IdToScoreMap<loop_closure::VertexId>* /* vertex_to_score_map */) const {
  // We do not have to score vertices to filter unlikely matches because this
  // is done already at keyframe level.
}
}  // namespace matching_based_loopclosure

#endif  // MATCHING_BASED_LOOPCLOSURE_MATCHING_BASED_ENGINE_INL_H_
