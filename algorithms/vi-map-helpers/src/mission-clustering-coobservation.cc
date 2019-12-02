#include "vi-map-helpers/mission-clustering-coobservation.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <maplab-common/accessors.h>
#include <vi-map/vi-map.h>

#include "vi-map-helpers/vi-map-queries.h"

namespace vi_map_helpers {
namespace {

template <typename IdType>
bool haveCommonElements(
    const std::unordered_set<IdType>& set_a,
    const std::unordered_set<IdType>& set_b) {
  // Make sure we loop over the smaller set.
  if (set_b.size() < set_a.size()) {
    return haveCommonElements<IdType>(set_b, set_a);
  }

  for (const IdType& id : set_a) {
    if (set_b.count(id) > 0u) {
      return true;
    }
  }
  return false;
}
}  // namespace

MissionCoobservationCachedQuery::MissionCoobservationCachedQuery(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdSet& mission_ids) {
  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map_helpers::VIMapQueries queries(vi_map);
    queries.getLandmarksObservedByMission(
        mission_id, &mission_landmark_map_[mission_id]);
  }

  for (const vi_map::MissionId& mission_id : mission_ids) {
    pose_graph::EdgeIdList mission_lc_edges;
    vi_map.getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kLoopClosure,
        &mission_lc_edges);
    pose_graph::EdgeIdSet mission_lc_edges_set;
    std::copy(
        mission_lc_edges.begin(), mission_lc_edges.end(),
        std::inserter(mission_lc_edges_set, mission_lc_edges_set.end()));
    mission_lc_edge_[mission_id] = mission_lc_edges_set;
  }
}

bool MissionCoobservationCachedQuery::hasCommonObservations(
    const vi_map::MissionId& mission_id,
    const vi_map::MissionIdSet& other_mission_ids) const {
  CHECK(mission_id.isValid());
  const vi_map::LandmarkIdSet& landmarks_of_mission =
      common::getChecked(mission_landmark_map_, mission_id);
  const pose_graph::EdgeIdSet& edge_ids_of_mission =
      common::getChecked(mission_lc_edge_, mission_id);

  for (const vi_map::MissionId& other_mission_id : other_mission_ids) {
    const vi_map::LandmarkIdSet& landmarks_of_other_mission =
        common::getChecked(mission_landmark_map_, other_mission_id);
    const pose_graph::EdgeIdSet& edge_ids_of_other_mission =
        common::getChecked(mission_lc_edge_, other_mission_id);

    if (haveCommonElements(landmarks_of_mission, landmarks_of_other_mission) ||
        haveCommonElements(edge_ids_of_mission, edge_ids_of_other_mission)) {
      return true;
    }
  }
  return false;
}

std::vector<vi_map::MissionIdSet> clusterMissionByLandmarkCoobservations(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdSet& mission_ids) {
  std::vector<vi_map::MissionIdSet> components;

  MissionCoobservationCachedQuery coobservations(vi_map, mission_ids);

  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map::MissionIdSet new_component;
    new_component.emplace(mission_id);
    for (std::vector<vi_map::MissionIdSet>::iterator it = components.begin();
         it != components.end();) {
      if (coobservations.hasCommonObservations(mission_id, *it)) {
        // Add the existing component items to the new component.
        new_component.insert(it->begin(), it->end());
        // Remove the old component from the components list.
        it = components.erase(it);
      } else {
        ++it;
      }
    }
    components.emplace_back(new_component);
  }
  return components;
}

size_t getNumAbsolute6DoFConstraintsForMissionCluster(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdSet& mission_ids) {
  CHECK(!mission_ids.empty());
  size_t num_constraints = 0u;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    num_constraints +=
        vi_map.getNumAbsolute6DoFMeasurementsInMission(mission_id);
  }
  return num_constraints;
}

bool hasLcEdgesInMissionCluster(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdSet& mission_ids) {
  CHECK(!mission_ids.empty());
  for (const vi_map::MissionId& mission_id : mission_ids) {
    pose_graph::EdgeIdList edge_ids;
    vi_map.getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kLoopClosure, &edge_ids);
    if (!edge_ids.empty()) {
      return true;
    }
  }
  return false;
}

bool hasInertialConstraintsInAllMissionsInCluster(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdSet& mission_ids) {
  CHECK(!mission_ids.empty());
  for (const vi_map::MissionId& mission_id : mission_ids) {
    pose_graph::EdgeIdList edge_ids;
    vi_map.getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kViwls, &edge_ids);
    if (edge_ids.empty()) {
      return false;
    }
  }
  return true;
}

bool hasVisualConstraintsInAllMissionsInCluster(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdSet& mission_ids) {
  CHECK(!mission_ids.empty());
  for (const vi_map::MissionId& mission_id : mission_ids) {
    // This might be a bit cheap, as the presence of an ncamera doesn't mean
    // there are actual visual constraints.
    if (!vi_map.getMission(mission_id).hasNCamera()) {
      return false;
    }
  }
  return true;
}

bool hasWheelOdometryConstraintsInAllMissionsInCluster(
    const vi_map::VIMap& vi_map, const vi_map::MissionIdSet& mission_ids) {
  CHECK(!mission_ids.empty());
  for (const vi_map::MissionId& mission_id : mission_ids) {
    pose_graph::EdgeIdList edge_ids;
    vi_map.getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kWheelOdometry, &edge_ids);
    if (edge_ids.empty()) {
      return false;
    }
  }
  return true;
}

}  // namespace vi_map_helpers
