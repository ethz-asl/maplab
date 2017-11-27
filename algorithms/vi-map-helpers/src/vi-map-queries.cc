#include "vi-map-helpers/vi-map-queries.h"

#include <algorithm>
#include <unordered_set>

#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/landmark-quality-metrics.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

pose_graph::VertexId getVertexIdWithMostOverlappingLandmarks(
    const pose_graph::VertexId& query_vertex_id,
    const vi_map::VertexKeyPointToStructureMatchList& structure_matches,
    const vi_map::VIMap& map, vi_map::LandmarkIdSet* overlap_landmarks) {
  CHECK(!structure_matches.empty());
  CHECK_NOTNULL(overlap_landmarks)->clear();

  typedef std::unordered_map<pose_graph::VertexId, vi_map::LandmarkIdSet>
      VertexOverlapLandmarksMap;
  VertexOverlapLandmarksMap vertex_overlap_map;
  for (const vi_map::VertexKeyPointToStructureMatch& structure_match :
       structure_matches) {
    const vi_map::LandmarkId& landmark_id = structure_match.landmark_result;
    map.getLandmark(landmark_id)
        .forEachObservation([&](const vi_map::KeypointIdentifier& keypoint_id) {
          if (keypoint_id.frame_id.vertex_id != query_vertex_id) {
            vertex_overlap_map[keypoint_id.frame_id.vertex_id].emplace(
                landmark_id);
          }
        });
  }

  size_t max_overlap_landmarks = 0u;
  pose_graph::VertexId largest_overlap_vertex_id;
  for (const VertexOverlapLandmarksMap::value_type& item : vertex_overlap_map) {
    if (item.second.size() > max_overlap_landmarks) {
      largest_overlap_vertex_id = item.first;
      max_overlap_landmarks = item.second.size();
    }
  }

  *overlap_landmarks =
      common::getChecked(vertex_overlap_map, largest_overlap_vertex_id);

  CHECK(largest_overlap_vertex_id.isValid());
  return largest_overlap_vertex_id;
}

VIMapQueries::VIMapQueries(const vi_map::VIMap& map) : map_(map) {}

void VIMapQueries::getIdsOfVerticesWithLandmarkObservations(
    pose_graph::VertexIdList* result) {
  CHECK_NOTNULL(result);
  pose_graph::VertexIdList all_vertices;
  map_.getAllVertexIds(&all_vertices);

  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    CHECK(vertex_id.isValid());
    vi_map::LandmarkIdList observed_landmark_ids;
    map_.getVertex(vertex_id).getAllObservedLandmarkIds(&observed_landmark_ids);
    for (const vi_map::LandmarkId& id : observed_landmark_ids) {
      if (id.isValid()) {
        result->emplace_back(vertex_id);
        break;
      }
    }
  }
  VLOG(3) << result->size() << " of " << all_vertices.size()
          << " have landmark observations.";
}

void VIMapQueries::getAllWellConstrainedLandmarkIds(
    vi_map::LandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  map_.getAllLandmarkIds(landmark_ids);

  for (vi_map::LandmarkIdList::iterator it = landmark_ids->begin();
       it != landmark_ids->end();) {
    if (!vi_map::isLandmarkWellConstrained(map_, map_.getLandmark(*it))) {
      it = landmark_ids->erase(it);
    } else {
      ++it;
    }
  }
}

void VIMapQueries::getAllNotWellConstrainedLandmarkIds(
    vi_map::LandmarkIdList* bad_landmarks_ids) const {
  CHECK_NOTNULL(bad_landmarks_ids)->clear();
  bad_landmarks_ids->reserve(map_.numLandmarks());

  map_.forEachLandmark([&bad_landmarks_ids](const vi_map::Landmark& landmark) {
    if (landmark.getQuality() == vi_map::Landmark::Quality::kBad) {
      bad_landmarks_ids->emplace_back(landmark.id());
    }
  });
}

void VIMapQueries::forIdsOfObservedLandmarksOfEachVertexWhile(
    const vi_map::MissionId& mission_id,
    const std::function<
        bool(const vi_map::LandmarkIdSet& store_landmarks)>&  // NOLINT
    action) const {
  CHECK(action);
  CHECK(map_.hasMission(mission_id));
  pose_graph::VertexIdList all_mission_vertices;
  map_.getAllVertexIdsInMission(mission_id, &all_mission_vertices);
  for (const pose_graph::VertexId& vertex_id : all_mission_vertices) {
    vi_map::LandmarkIdSet store_landmarks;
    getIdsOfLandmarksObservedByVertex(vertex_id, &store_landmarks);
    if (!action(store_landmarks)) {
      break;
    }
  }
}

void VIMapQueries::forIdsOfObservedLandmarksOfEachVertexAlongGraphWhile(
    const vi_map::MissionId& mission_id,
    const std::function<
        bool(const vi_map::LandmarkIdSet& store_landmarks)>&  // NOLINT
    action) const {
  CHECK(action);
  CHECK(map_.hasMission(mission_id));
  pose_graph::VertexId vertex_id =
      map_.getMission(mission_id).getRootVertexId();
  pose_graph::Edge::EdgeType traversal_edge_type =
      map_.getGraphTraversalEdgeType(mission_id);
  do {
    vi_map::LandmarkIdSet store_landmarks;
    getIdsOfLandmarksObservedByVertex(vertex_id, &store_landmarks);
    if (!action(store_landmarks)) {
      break;
    }
  } while (map_.getNextVertex(vertex_id, traversal_edge_type, &vertex_id));
}

void VIMapQueries::getLandmarksObservedByMission(
    const vi_map::MissionId& query_mission,
    vi_map::LandmarkIdSet* observed_landmarks) const {
  CHECK_NOTNULL(observed_landmarks)->clear();

  forIdsOfObservedLandmarksOfEachVertexWhile(
      query_mission, [&](const vi_map::LandmarkIdSet& landmarks) {
        for (const vi_map::LandmarkId& landmark_id : landmarks) {
          observed_landmarks->emplace(landmark_id);
        }
        return true;
      });
}

bool VIMapQueries::observesLandmarksObservedByMultipleMissions(
    const vi_map::MissionId& query_mission) const {
  bool result = false;
  vi_map::LandmarkIdSet checked_landmarks;
  forIdsOfObservedLandmarksOfEachVertexWhile(
      query_mission, [&](const vi_map::LandmarkIdSet& store_landmarks) {
        for (const vi_map::LandmarkId& landmark_id : store_landmarks) {
          if (checked_landmarks.emplace(landmark_id).second) {
            if (isLandmarkObservedByMultipleMissions(landmark_id)) {
              result = true;
              return false;
            }
          }
        }
        return true;
      });
  return result;
}

void VIMapQueries::getMissionsStoringLandmarksObservedByMultipleMissions(
    const vi_map::MissionId& query_mission,
    vi_map::MissionIdSet* result) const {
  CHECK_NOTNULL(result)->clear();
  VLOG(3) << "Getting missions storing landmarks that are observed by multiple "
          << "missions";
  common::ProgressBar progress_bar(map_.numVerticesInMission(query_mission));
  forIdsOfObservedLandmarksOfEachVertexAlongGraphWhile(
      query_mission, [&](const vi_map::LandmarkIdSet& store_landmarks) {
        for (const vi_map::LandmarkId& landmark_id : store_landmarks) {
          if (isLandmarkObservedByMultipleMissions(landmark_id)) {
            result->emplace(
                map_.getLandmarkStoreVertex(landmark_id).getMissionId());
          }
        }
        progress_bar.increment();
        return true;
      });
}

bool VIMapQueries::storesLandmarksObservedByOtherMissions(
    const vi_map::MissionId& query_mission) const {
  vi_map::MissionIdSet owning_missions;
  getMissionsStoringLandmarksObservedByMultipleMissions(
      query_mission, &owning_missions);
  CHECK_NE(owning_missions.size(), 0u) << "Mission hasn't been merged!";
  return owning_missions.count(query_mission) != 0u;
}

void VIMapQueries::getVertexIdsAlongGraphForMissions(
    const vi_map::MissionIdList& mission_id_list,
    vi_map::MissionVertexIdList* mission_to_vertex_ids_map) const {
  CHECK_NOTNULL(mission_to_vertex_ids_map)->clear();

  for (const vi_map::MissionId& mission_id : mission_id_list) {
    CHECK(mission_id.isValid());
    pose_graph::VertexIdList& mission_vertex_id_list =
        (*mission_to_vertex_ids_map)[mission_id];
    map_.getAllVertexIdsInMissionAlongGraph(
        mission_id, &mission_vertex_id_list);
  }
  CHECK_EQ(mission_to_vertex_ids_map->size(), mission_id_list.size());
}

void VIMapQueries::getCommonObserversForLandmarks(
    const vi_map::LandmarkIdList& landmarks,
    pose_graph::VertexIdList* result) const {
  CHECK_NOTNULL(result)->clear();
  size_t num_landmarks = landmarks.size();
  std::unordered_map<pose_graph::VertexId, size_t> vertex_is_observer_count;
  for (const vi_map::LandmarkId& landmark : landmarks) {
    VLOG(4) << "Processing landmark " << landmark;
    pose_graph::VertexIdSet observer_ids_set;
    map_.getLandmarkObserverVertices(landmark, &observer_ids_set);
    for (const pose_graph::VertexId& observer_id : observer_ids_set) {
      ++vertex_is_observer_count[observer_id];
    }
  }
  for (const std::unordered_map<pose_graph::VertexId, size_t>::value_type&
           vertex_id_count : vertex_is_observer_count) {
    if (vertex_id_count.second == num_landmarks) {
      result->push_back(vertex_id_count.first);
    }
  }
}

bool VIMapQueries::isLandmarkObservedByMultipleMissions(
    const vi_map::LandmarkId& landmark_id) const {
  vi_map::MissionIdSet observing_missions;
  map_.getLandmarkObserverMissions(landmark_id, &observing_missions);
  return observing_missions.size() > 1u;
}

void VIMapQueries::getVisualFramesWithCommonLandmarksSortedMostToLeast(
    const vi_map::Vertex& vertex, const size_t frame_index,
    const size_t min_common_landmarks,
    vi_map::VisualFrameIdentifierList* result) const {
  CHECK_NOTNULL(result)->clear();
  vi_map::VisualFrameIdentifier self(vertex.id(), frame_index);
  vi_map::LandmarkIdList frame_landmarks;
  vertex.getFrameObservedLandmarkIds(frame_index, &frame_landmarks);

  std::unordered_map<vi_map::VisualFrameIdentifier, size_t>
      common_landmark_counts;
  for (const vi_map::LandmarkId& landmark_id : frame_landmarks) {
    if (!landmark_id.isValid()) {
      continue;
    }
    const vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
    landmark.forEachObservation(
        [&](const vi_map::KeypointIdentifier& keypoint_id) {
          if (keypoint_id.frame_id == self) {
            return;
          }
          if (common_landmark_counts[keypoint_id.frame_id] == 0) {
            result->push_back(keypoint_id.frame_id);
          }
          ++common_landmark_counts[keypoint_id.frame_id];
        });
  }

  if (min_common_landmarks == 0) {
    map_.forEachVisualFrame([&](const vi_map::VisualFrameIdentifier& frame_id) {
      if (common_landmark_counts[frame_id] == 0 && frame_id != self) {
        result->push_back(frame_id);
      }
    });
  }

  std::sort(
      result->begin(), result->end(),
      [&](const vi_map::VisualFrameIdentifier& a,
          const vi_map::VisualFrameIdentifier& b) {
        return common_landmark_counts[a] > common_landmark_counts[b];
      });

  vi_map::VisualFrameIdentifierList::iterator cutoff = std::upper_bound(
      result->begin(), result->end(), min_common_landmarks,
      [&common_landmark_counts](
          const size_t lhs, const vi_map::VisualFrameIdentifier& rhs) {
        return lhs > common_landmark_counts[rhs];
      });

  result->resize(cutoff - result->begin());
}

size_t VIMapQueries::getNumWellConstrainedLandmarks(
    const vi_map::Vertex& vertex, const size_t frame_index) const {
  CHECK(vertex.isVisualFrameSet(frame_index));

  vi_map::LandmarkIdList landmark_ids;
  vertex.getFrameObservedLandmarkIds(frame_index, &landmark_ids);

  size_t well_constrained_landmarks = 0;
  for (const vi_map::LandmarkId landmark_id : landmark_ids) {
    if (landmark_id.isValid()) {
      const bool is_good = vi_map::isLandmarkWellConstrained(
          map_, map_.getLandmark(landmark_id));
      if (is_good) {
        ++well_constrained_landmarks;
      }
    }
  }
  return well_constrained_landmarks;
}

void VIMapQueries::getFollowingVertexIdsAlongGraph(
    const pose_graph::VertexId& starting_vertex,
    const bool include_starting_vertex,
    pose_graph::VertexIdList* result) const {
  CHECK_NOTNULL(result)->clear();
  const vi_map::MissionId mission_id =
      map_.getVertex(starting_vertex).getMissionId();

  pose_graph::VertexId current_vertex_id;
  if (include_starting_vertex) {
    current_vertex_id = starting_vertex;
  } else {
    if (!map_.getNextVertex(
            starting_vertex, map_.getGraphTraversalEdgeType(mission_id),
            &current_vertex_id)) {
      return;
    }
  }

  do {
    result->push_back(current_vertex_id);
  } while (map_.getNextVertex(
      current_vertex_id, map_.getGraphTraversalEdgeType(mission_id),
      &current_vertex_id));
}
void VIMapQueries::getFollowingVertexIdsAlongGraph(
    const pose_graph::VertexId& starting_vertex,
    pose_graph::VertexIdList* result) const {
  constexpr bool kIncludeStartingVertex = true;
  getFollowingVertexIdsAlongGraph(
      starting_vertex, kIncludeStartingVertex, result);
}

int VIMapQueries::getVerticesWithCommonLandmarks(
    const pose_graph::VertexId& vertex_id, int num_matches_to_return,
    int min_number_common_landmarks,
    pose_graph::VertexIdList* coobserver_vertex_ids) const {
  CHECK_NOTNULL(coobserver_vertex_ids)->clear();
  CHECK(map_.hasVertex(vertex_id));

  VertexCommonLandmarksCountVector coobserver_vertices_with_counts;
  getVerticesWithCommonLandmarks(
      vertex_id, min_number_common_landmarks, &coobserver_vertices_with_counts);

  unsigned int total_count = std::min(
      coobserver_vertices_with_counts.size(),
      static_cast<size_t>(num_matches_to_return));
  coobserver_vertex_ids->reserve(total_count);
  for (unsigned int i = 0; i < total_count; ++i) {
    coobserver_vertex_ids->push_back(
        coobserver_vertices_with_counts[i].vertex_id);
  }
  CHECK_LE(
      static_cast<int>(coobserver_vertex_ids->size()), num_matches_to_return);
  return coobserver_vertex_ids->size();
}

int VIMapQueries::getVerticesWithCommonLandmarks(
    const pose_graph::VertexId& vertex_id, int min_number_common_landmarks,
    VertexCommonLandmarksCountVector* coobserver_vertex_ids) const {
  CHECK_NOTNULL(coobserver_vertex_ids)->clear();
  CHECK(map_.hasVertex(vertex_id));

  vi_map::LandmarkIdList landmark_ids;
  pose_graph::VertexIdSet visited_vertices;
  map_.getVertexPtr(vertex_id)->getAllObservedLandmarkIds(&landmark_ids);
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (landmark_id.isValid()) {
      const vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
      landmark.forEachObservation(
          [&](const vi_map::KeypointIdentifier& backlink) {
            if (visited_vertices.insert(backlink.frame_id.vertex_id).second) {
              // This vertex was not yet visited and was successfully inserted.
              if (map_.hasVertex(backlink.frame_id.vertex_id)) {
                int common = getNumberOfCommonLandmarks(
                    vertex_id, backlink.frame_id.vertex_id);
                if (common >= min_number_common_landmarks) {
                  coobserver_vertex_ids->push_back(
                      VertexCommonLandmarksCount(
                          common, backlink.frame_id.vertex_id));
                }
              }
            }
          });
    }
  }
  std::sort(
      coobserver_vertex_ids->begin(), coobserver_vertex_ids->end(),
      VertexCommonLandmarksCountComparator());
  return coobserver_vertex_ids->size();
}

int VIMapQueries::getNumberOfCommonLandmarks(
    const pose_graph::VertexId& vertex_1,
    const pose_graph::VertexId& vertex_2) const {
  return getNumberOfCommonLandmarks(vertex_1, vertex_2, nullptr);
}

// Returns the number of common landmarks of vertex 1 and 2
// NOTE:  The landmarks argument of this function is allowed to be a
//        nullptr. However if only the number of common landmarks is
//        required, better use: getNumberOfCommonLandmarks(v1, v2)
int VIMapQueries::getNumberOfCommonLandmarks(
    const pose_graph::VertexId& vertex_1_id,
    const pose_graph::VertexId& vertex_2_id,
    vi_map::LandmarkIdSet* landmarks) const {
  if (landmarks) {
    landmarks->clear();
  }

  vi_map::LandmarkIdList vector_landmark_ids_1;
  map_.getVertex(vertex_1_id).getAllObservedLandmarkIds(&vector_landmark_ids_1);
  vi_map::LandmarkIdSet landmark_ids_1(vector_landmark_ids_1.begin(),
                                       vector_landmark_ids_1.end());

  vi_map::LandmarkId invalid_id;
  invalid_id.setInvalid();
  landmark_ids_1.erase(invalid_id);

  vi_map::LandmarkIdList landmark_ids_2;
  map_.getVertex(vertex_2_id).getAllObservedLandmarkIds(&landmark_ids_2);

  int result = 0;
  for (const vi_map::LandmarkId& landmark2_id : landmark_ids_2) {
    if (landmark2_id.isValid()) {
      if (landmark_ids_1.count(landmark2_id) != 0u) {
        ++result;
        if (landmarks) {
          landmarks->emplace(landmark2_id);
        }
      }
    }
  }
  return result;
}

int VIMapQueries::getVertexWithMostLandmarksInCommon(
    const pose_graph::VertexId& vertex_id,
    pose_graph::VertexId* best_match_vertex_id) const {
  CHECK_NOTNULL(best_match_vertex_id);
  CHECK(map_.hasVertex(vertex_id));

  int max = 0;
  pose_graph::VertexId max_vertex;

  vi_map::LandmarkIdList landmark_ids;
  pose_graph::VertexIdSet visited_vertices;
  map_.getVertexPtr(vertex_id)->getAllObservedLandmarkIds(&landmark_ids);
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (landmark_id.isValid()) {
      const vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
      landmark.forEachObservation(
          [&](const vi_map::KeypointIdentifier& backlink) {
            if ((backlink.frame_id.vertex_id != vertex_id) &&
                (visited_vertices.find(backlink.frame_id.vertex_id) ==
                 visited_vertices.end())) {
              visited_vertices.insert(backlink.frame_id.vertex_id);

              if (map_.hasVertex(backlink.frame_id.vertex_id)) {
                int common = getNumberOfCommonLandmarks(
                    vertex_id, backlink.frame_id.vertex_id);
                if (common > max) {
                  max = common;
                  max_vertex = backlink.frame_id.vertex_id;
                }
              }
            }
          });
    }
  }
  if ((max > 0) && (max_vertex.isValid())) {
    CHECK(max_vertex.isValid());
    *best_match_vertex_id = max_vertex;
  } else {
    LOG(WARNING) << "Couldn't find a vertex with common landmarks.";
  }
  return max;
}

void VIMapQueries::getIdsOfLandmarksObservedByVertex(
    const pose_graph::VertexId& vertex_id,
    vi_map::LandmarkIdSet* result) const {
  CHECK_NOTNULL(result);
  const vi_map::Vertex& vertex = map_.getVertex(vertex_id);
  vi_map::LandmarkIdList all_landmark_ids;
  vertex.getAllObservedLandmarkIds(&all_landmark_ids);
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    if (landmark_id.isValid()) {
      result->emplace(landmark_id);
    }
  }
}

void VIMapQueries::getCoobservingVertices(
    const std::unordered_set<pose_graph::VertexId>& given_vertices,
    const double min_coobserved_features_ratio,
    const size_t min_coobsered_features_count,
    std::unordered_set<pose_graph::VertexId>* coobserving_vertices) const {
  CHECK_NOTNULL(coobserving_vertices)->clear();

  pose_graph::VertexIdSet coobserver_candidates;
  for (const pose_graph::VertexId& vertex_id : given_vertices) {
    vi_map::LandmarkIdList landmarks;
    map_.getVertex(vertex_id).getAllObservedLandmarkIds(&landmarks);
    for (const vi_map::LandmarkId& landmark_id : landmarks) {
      if (!landmark_id.isValid()) {
        continue;
      }
      const vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
      landmark.forEachObservation(
          [&](const vi_map::KeypointIdentifier& backlink) {
            CHECK(map_.hasVertex(backlink.frame_id.vertex_id));
            coobserver_candidates.insert(backlink.frame_id.vertex_id);
          });
    }
  }

  for (const pose_graph::VertexId& vertex_id : coobserver_candidates) {
    if (given_vertices.count(vertex_id) > 0u) {
      continue;
    }
    if (min_coobserved_features_ratio > 0.0) {
      if (isCoobservingBasedOnPercentage(
              min_coobserved_features_ratio, given_vertices, vertex_id)) {
        coobserving_vertices->insert(vertex_id);
      }
    } else {
      if (isCoobservingBasedOnAbsoluteCountOfLandmarkObservers(
              min_coobsered_features_count, given_vertices, vertex_id)) {
        coobserving_vertices->insert(vertex_id);
      }
    }
  }
}

bool VIMapQueries::isCoobservingBasedOnPercentage(
    double minimum_percentage_of_shared_observations,
    const pose_graph::VertexIdSet& given_vertices,
    const pose_graph::VertexId& query_vertex_id) const {
  vi_map::LandmarkIdList landmarks;
  map_.getVertex(query_vertex_id).getAllObservedLandmarkIds(&landmarks);

  int num_coobserved_landmarks_from_vertex_in_window = 0;
  int num_coobserved_landmarks_from_vertex_not_in_window = 0;

  for (const vi_map::LandmarkId& landmark_id : landmarks) {
    if (!landmark_id.isValid()) {
      continue;
    }
    const vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
    landmark.forEachObservation(
        [&](const vi_map::KeypointIdentifier& backlink) {
          CHECK(map_.hasVertex(backlink.frame_id.vertex_id));
          if (given_vertices.count(backlink.frame_id.vertex_id) > 0u) {
            ++num_coobserved_landmarks_from_vertex_in_window;
          } else {
            ++num_coobserved_landmarks_from_vertex_not_in_window;
          }
        });
  }
  double actual_visibility_ratio = 0;

  if (num_coobserved_landmarks_from_vertex_not_in_window != 0) {
    actual_visibility_ratio =
        static_cast<double>(num_coobserved_landmarks_from_vertex_in_window) /
        num_coobserved_landmarks_from_vertex_not_in_window;
  }
  return actual_visibility_ratio >= minimum_percentage_of_shared_observations;
}

bool VIMapQueries::isCoobservingBasedOnAbsoluteCountOfLandmarkObservers(
    unsigned int minimum_observers_needed,
    const pose_graph::VertexIdSet& given_vertices,
    const pose_graph::VertexId& query_vertex_id) const {
  CHECK(query_vertex_id.isValid());
  vi_map::LandmarkIdList landmarks;
  map_.getVertex(query_vertex_id).getAllObservedLandmarkIds(&landmarks);
  unsigned int given_vertices_observer_count = 0u;
  for (const vi_map::LandmarkId& landmark_id : landmarks) {
    if (!landmark_id.isValid()) {
      continue;
    }
    const vi_map::Landmark& landmark = map_.getLandmark(landmark_id);
    landmark.forEachObservation(
        [&](const vi_map::KeypointIdentifier& backlink) {
          CHECK(map_.hasVertex(backlink.frame_id.vertex_id));
          if (given_vertices.count(backlink.frame_id.vertex_id) > 0u) {
            ++given_vertices_observer_count;
          }
        });
    if (given_vertices_observer_count >= minimum_observers_needed) {
      return true;
    }
  }
  return false;
}

void VIMapQueries::getBoundaryVertexIds(
    const pose_graph::VertexIdList& inner_vertices,
    pose_graph::VertexIdList* boundary_vertices) const {
  pose_graph::VertexIdSet inner_vertices_set(
      inner_vertices.begin(), inner_vertices.end());
  getBoundaryVertexIds(inner_vertices_set, boundary_vertices);
}
void VIMapQueries::getBoundaryVertexIds(
    const pose_graph::VertexIdSet& inner_vertices,
    pose_graph::VertexIdList* boundary_vertices) const {
  CHECK_NOTNULL(boundary_vertices)->clear();

  pose_graph::VertexId candidate_vertex_id;

  // A set is used to filter out duplicate ids.
  pose_graph::VertexIdSet boundary_vertex_set;
  for (const pose_graph::VertexId& inner_vertex_id : inner_vertices) {
    CHECK(map_.hasVertex(inner_vertex_id));
    vi_map::MissionId mission_id =
        map_.getVertex(inner_vertex_id).getMissionId();
    CHECK(mission_id.isValid());

    if (map_.getNextVertex(
            inner_vertex_id, map_.getGraphTraversalEdgeType(mission_id),
            &candidate_vertex_id)) {
      if (inner_vertices.count(candidate_vertex_id) == 0u) {
        boundary_vertex_set.insert(candidate_vertex_id);
      }
    }
    if (map_.getPreviousVertex(
            inner_vertex_id, map_.getGraphTraversalEdgeType(mission_id),
            &candidate_vertex_id)) {
      if (inner_vertices.count(candidate_vertex_id) == 0u) {
        boundary_vertex_set.insert(candidate_vertex_id);
      }
    }
  }

  boundary_vertices->insert(
      boundary_vertices->end(), boundary_vertex_set.begin(),
      boundary_vertex_set.end());
  if (boundary_vertices->empty()) {
    LOG(WARNING) << "No boundary vertices found!";
  }
}

void VIMapQueries::forEachWellConstrainedObservedLandmark(
    const vi_map::Vertex& vertex,
    const std::function<void(
        const size_t frame_idx, const vi_map::LandmarkId& landmark_id,
        const vi_map::Vertex& storing_vertex)>& action) const {
  const unsigned int num_frames = vertex.numFrames();
  for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
    if (!vertex.isVisualFrameSet(frame_idx)) {
      continue;
    }
    for (size_t i = 0u; i < vertex.observedLandmarkIdsSize(frame_idx); ++i) {
      const vi_map::LandmarkId landmark_id =
          vertex.getObservedLandmarkId(frame_idx, i);

      if (landmark_id.isValid() && vi_map::isLandmarkWellConstrained(
                                       map_, map_.getLandmark(landmark_id))) {
        const vi_map::Vertex& storing_vertex =
            map_.getLandmarkStoreVertex(landmark_id);
        action(frame_idx, landmark_id, storing_vertex);
      }
    }
  }
}

}  // namespace vi_map_helpers
