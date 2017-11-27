#ifndef VI_MAP_HELPERS_VI_MAP_QUERIES_INL_H_
#define VI_MAP_HELPERS_VI_MAP_QUERIES_INL_H_

#include <vector>

#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

template <typename VertexIdContainerType>
void VIMapQueries::getConsecutiveLandmarkObserverGroupsFromMission(
    const vi_map::LandmarkId& landmark_id,
    const vi_map::MissionId& mission_id,
    std::vector<VertexIdContainerType>* clustered_vertex_ids) const {
  CHECK_NOTNULL(clustered_vertex_ids)->clear();

  vi_map::MissionIdSet observer_mission_ids;
  map_.getLandmarkObserverMissions(landmark_id, &observer_mission_ids);
  CHECK_GT(observer_mission_ids.count(mission_id), 0u)
      << "Mission " << mission_id.hexString() << " does not seem to observe "
      << "the landmark with ID " << landmark_id.hexString();

  pose_graph::VertexIdSet observer_vertex_ids;
  map_.getLandmarkObserverVertices(landmark_id, &observer_vertex_ids);

  pose_graph::VertexIdSet mission_observer_vertex_ids;
  for (const pose_graph::VertexId& vertex_id : observer_vertex_ids) {
    if (map_.getVertex(vertex_id).getMissionId() == mission_id) {
      mission_observer_vertex_ids.emplace(vertex_id);
    }
  }

  clusterConsecutiveVerticesFromOneMission(
      mission_observer_vertex_ids, clustered_vertex_ids);
}

template <typename VertexIdContainerType>
void VIMapQueries::clusterConsecutiveVerticesFromOneMission(
    const pose_graph::VertexIdSet& vertices,
    std::vector<VertexIdContainerType>* clustered_vertex_ids) const {
  CHECK_NOTNULL(clustered_vertex_ids)->clear();

  vi_map::MissionId mission_id;
  for (const pose_graph::VertexId& vertex_id : vertices) {
    const vi_map::MissionId& current_mission_id =
        map_.getVertex(vertex_id).getMissionId();

    if (!mission_id.isValid()) {
      // mission_id not initialized yet, let's do it now.
      mission_id = current_mission_id;
    } else {
      // mission_id is valid so we should verify if all the other vertices
      // belong to the same mission.
      CHECK_EQ(current_mission_id, mission_id) << "All vertices must belong "
                                               << "to the same mission.";
    }
  }

  pose_graph::VertexIdList ordered_vertex_ids;
  map_.getAllVertexIdsInMissionAlongGraph(mission_id, &ordered_vertex_ids);

  for (size_t i = 0u; i < ordered_vertex_ids.size(); ++i) {
    if (vertices.count(ordered_vertex_ids[i]) > 0u) {
      VertexIdContainerType current_cluster;
      do {
        current_cluster.insert(current_cluster.end(), ordered_vertex_ids[i]);
        ++i;
      } while (vertices.count(ordered_vertex_ids[i]) > 0u &&
               i < ordered_vertex_ids.size());

      clustered_vertex_ids->push_back(current_cluster);
    }
  }
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_QUERIES_INL_H_
