#include "map-anchoring/map-anchoring.h"

#include <loop-closure-handler/loop-detector-node.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

DEFINE_bool(
    add_anchored_missions_to_database, true,
    "Add missions that got anchored to the loop detector database too.");

namespace map_anchoring {

void setMissionBaseframeKnownState(
    const vi_map::MissionId& mission_id, const bool baseframe_known_state,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  const vi_map::MissionBaseFrameId& mission_baseframe_id =
      map->getMission(mission_id).getBaseFrameId();
  map->getMissionBaseFrame(mission_baseframe_id)
      .set_is_T_G_M_known(baseframe_known_state);
}

bool anchorMission(const vi_map::MissionId& mission_id, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(map->hasMission(mission_id));

  vi_map::VIMission& mission = map->getMission(mission_id);
  vi_map::MissionBaseFrame& mission_baseframe =
      map->getMissionBaseFrame(mission.getBaseFrameId());
  if (mission_baseframe.is_T_G_M_known()) {
    LOG(ERROR) << "The T_G_M transformation of the baseframe of mission "
               << mission_id << " is already marked as known. The mission "
               << "can't be anchored again.";
    return false;
  }

  loop_detector_node::LoopDetectorNode loop_detector;
  addAllMissionsWithKnownBaseFrameToProvidedLoopDetector(*map, &loop_detector);

  return anchorMissionUsingProvidedLoopDetector(mission_id, loop_detector, map);
}

bool anchorAllMissions(vi_map::VIMap* map) {
  constexpr std::nullptr_t kPlotter = nullptr;
  return anchorAllMissions(map, kPlotter);
}

bool anchorAllMissions(
    vi_map::VIMap* map, const visualization::ViwlsGraphRvizPlotter* plotter) {
  CHECK_NOTNULL(map);
  // Build a list of all unknown baseframes.
  std::queue<vi_map::MissionId> missions_with_unknown_baseframe;
  vi_map::MissionIdList all_missions;
  map->getAllMissionIds(&all_missions);
  for (const vi_map::MissionId& mission_id : all_missions) {
    const vi_map::Mission& mission = map->getMission(mission_id);
    const vi_map::MissionBaseFrame& base_frame =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    if (!base_frame.is_T_G_M_known()) {
      missions_with_unknown_baseframe.push(mission_id);
    }
  }

  if (missions_with_unknown_baseframe.size() == all_missions.size()) {
    LOG(ERROR) << "At least one mission needs to have a known baseframe.";
    return false;
  }

  if (missions_with_unknown_baseframe.empty()) {
    LOG(INFO) << "All baseframes are known -- nothing to do.";
    return true;
  }

  // A loop-detector to which we add all the missions we have already anchored.
  // This allows us to avoid building the index over and over as we anchor new
  // missions.
  loop_detector_node::LoopDetectorNode loop_detector;

  // Add the known missions to the database.
  bool initial_mission_added = false;

  // How often do we try to anchor. Prevent locking forever.
  constexpr int kTrialsPerMission = 2;
  int remaining_trials_for_mission = kTrialsPerMission;
  while (!missions_with_unknown_baseframe.empty()) {
    if (FLAGS_add_anchored_missions_to_database || !initial_mission_added) {
      VLOG(1) << "Adding known missions to loop-detector.";
      // Add all missions that have a known base-frame (if any).
      addAllMissionsWithKnownBaseFrameToProvidedLoopDetector(
          *map, &loop_detector);
      initial_mission_added = true;
    }

    vi_map::MissionId mission_id = missions_with_unknown_baseframe.front();
    missions_with_unknown_baseframe.pop();

    // Assemble the current working set.
    vi_map::MissionIdSet selected_missions;
    selected_missions.insert(mission_id);
    vi_map::MissionIdList all_missions;
    map->getAllMissionIds(&all_missions);
    for (const vi_map::MissionId& mission_id : all_missions) {
      const vi_map::MissionBaseFrame& base_frame =
          map->getMissionBaseFrameForMission(mission_id);
      if (base_frame.is_T_G_M_known()) {
        selected_missions.insert(mission_id);
      }
    }
    map->selectMissions(selected_missions);

    VLOG(1) << "Trying to anchor mission " << mission_id << ".";
    bool success =
        anchorMissionUsingProvidedLoopDetector(mission_id, loop_detector, map);

    if (!success) {
      LOG(WARNING) << "Failed to anchor mission " << mission_id
                   << ", pushing it back to the work-list.";
      --remaining_trials_for_mission;
      if (remaining_trials_for_mission > 0) {
        missions_with_unknown_baseframe.push(mission_id);
        continue;
      }

      // Continuing to next mission, reset number of remaining trials.
      remaining_trials_for_mission = kTrialsPerMission;
      continue;
    }

    if (plotter != nullptr) {
      plotter->visualizeMap(*map);
    }
  }

  if (!missions_with_unknown_baseframe.empty()) {
    LOG(ERROR) << "Could not anchor all missions. Still have the following "
               << "unanchored:";
    while (!missions_with_unknown_baseframe.empty()) {
      vi_map::MissionId mission_id = missions_with_unknown_baseframe.front();
      missions_with_unknown_baseframe.pop();
      LOG(ERROR) << "\t" << mission_id;
    }
    map->resetMissionSelection();
    return false;
  }
  map->resetMissionSelection();
  VLOG(3) << "All missions anchored.";
  return true;
}

bool addAllMissionsWithKnownBaseFrameToProvidedLoopDetector(
    const vi_map::VIMap& map,
    loop_detector_node::LoopDetectorNode* loop_detector) {
  CHECK_NOTNULL(loop_detector);

  // Already anchored map mission IDs. They will be kept fixed during the
  // optimization phase.
  vi_map::MissionIdList base_mission_ids;
  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);

  for (const vi_map::MissionId& map_mission_id : all_mission_ids) {
    const vi_map::VIMission& map_mission = map.getMission(map_mission_id);
    if (map.getMissionBaseFrame(map_mission.getBaseFrameId())
            .is_T_G_M_known()) {
      base_mission_ids.emplace_back(map_mission_id);

      // Only add the missions which we don't already have in the database.
      if (!loop_detector->hasMissionInDatabase(map_mission_id)) {
        loop_detector->addMissionToDatabase(map_mission_id, map);
      }
    }
  }

  if (base_mission_ids.empty()) {
    LOG(ERROR) << "At least one mission should have a known T_G_M baseframe "
               << "transformation.";
    return false;
  }

  VLOG(1) << loop_detector->printStatus();
  return true;
}

bool anchorMissionUsingProvidedLoopDetector(
    const vi_map::MissionId& mission_id,
    const loop_detector_node::LoopDetectorNode& loop_detector,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(map->hasMission(mission_id));

  VLOG(1) << "Matching mission " << mission_id << " to database.";
  // Probe.
  ProbeResult probe_result;
  probeMissionAnchoring(mission_id, loop_detector, map, &probe_result);

  if (probe_result.wasSuccessful()) {
    CHECK(!probe_result.matching_missions.empty());
    VLOG(1) << "Probe successful, will anchor mission " << mission_id;

    vi_map::VIMission& mission = map->getMission(mission_id);
    vi_map::MissionBaseFrame& mission_baseframe =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    // Prealign.
    mission_baseframe.set_T_G_M(probe_result.T_G_M);

    // Mark as anchored.
    constexpr bool kIsMissionAnchored = true;
    map->getMissionBaseFrame(mission.getBaseFrameId())
        .set_is_T_G_M_known(kIsMissionAnchored);

    return true;
  }
  LOG(WARNING) << "Probe did not meet criteria, not merging mission "
               << mission_id;
  return false;
}

ProbeResult::ProbeResult()
    : num_vertex_candidate_links(0), average_landmark_match_inlier_ratio(0.) {}

bool ProbeResult::wasSuccessful() const {
  return num_vertex_candidate_links >= kMinMergingNumLinks ||
         average_landmark_match_inlier_ratio >= kMinMergingInlierRatioThreshold;
}

void probeMissionAnchoring(
    const vi_map::MissionId& mission_id,
    const loop_detector_node::LoopDetectorNode& loop_detector,
    vi_map::VIMap* map, ProbeResult* result) {
  CHECK_NOTNULL(map);
  CHECK_NOTNULL(result);

  static constexpr bool kMergeLandmarksOnProbe = false;
  static constexpr bool kAddLoopclosureEdgesOnProbe = false;

  vi_map::LoopClosureConstraintVector inlier_constraints;
  loop_detector.detectLoopClosuresMissionToDatabase(
      mission_id, kMergeLandmarksOnProbe, kAddLoopclosureEdgesOnProbe,
      &result->num_vertex_candidate_links,
      &result->average_landmark_match_inlier_ratio, map, &result->T_G_M,
      &inlier_constraints);

  // Create a list of the missions that received matches.
  std::unordered_map<vi_map::MissionId, int> mission_matches;
  for (const vi_map::LoopClosureConstraint& constraint : inlier_constraints) {
    for (const vi_map::VertexKeyPointToStructureMatch& structure_match :
         constraint.structure_matches) {
      const vi_map::Vertex& storing_vertex =
          map->getLandmarkStoreVertex(structure_match.landmark_result);
      mission_matches[storing_vertex.getMissionId()] += 1;
    }
  }
  std::vector<std::pair<vi_map::MissionId, int>> sorted_matches;
  for (const std::pair<const vi_map::MissionId, int>& match : mission_matches) {
    sorted_matches.emplace_back(match);
  }
  std::sort(
      sorted_matches.begin(), sorted_matches.end(),
      [](const std::pair<vi_map::MissionId, int>& lhs,
         const std::pair<vi_map::MissionId, int>& rhs) {
        return lhs.second > rhs.second;
      });

  // Only take the two best matching missions.
  sorted_matches.resize(std::min<size_t>(sorted_matches.size(), 2u));
  for (const std::pair<vi_map::MissionId, int>& match : sorted_matches) {
    if (match.first != mission_id) {
      VLOG(1) << "Selecting mission " << match.first << " with score "
              << match.second;
      result->matching_missions.push_back(match.first);
    }
  }

  VLOG(2) << "num_vertex_candidate_links "
          << result->num_vertex_candidate_links;
  VLOG(2) << "average_landmark_match_inlier_ratio "
          << result->average_landmark_match_inlier_ratio;
}

}  // namespace map_anchoring
