#include "vi-map-helpers/mission-clustering-coobservation.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <maplab-common/accessors.h>
#include <vi-map/vi-map.h>

#include "vi-map-helpers/vi-map-queries.h"

namespace vi_map_helpers {
namespace {
bool haveCommonElements(
    const vi_map::LandmarkIdSet& set_a, const vi_map::LandmarkIdSet& set_b) {
  // Make sure we loop over the smaller set.
  if (set_b.size() < set_a.size()) {
    return haveCommonElements(set_b, set_a);
  }

  for (const vi_map::LandmarkId& landmark_id : set_a) {
    if (set_b.count(landmark_id) > 0u) {
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
}

bool MissionCoobservationCachedQuery::hasCommonObservations(
    const vi_map::MissionId& mission_id,
    const vi_map::MissionIdSet& other_mission_ids) const {
  CHECK(mission_id.isValid());
  const vi_map::LandmarkIdSet& landmarks_of_mission =
      common::getChecked(mission_landmark_map_, mission_id);

  for (const vi_map::MissionId& other_mission_id : other_mission_ids) {
    const vi_map::LandmarkIdSet& landmarks_of_other_mission =
        common::getChecked(mission_landmark_map_, other_mission_id);

    if (haveCommonElements(landmarks_of_mission, landmarks_of_other_mission)) {
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

}  // namespace vi_map_helpers
