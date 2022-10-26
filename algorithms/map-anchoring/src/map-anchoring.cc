#include "map-anchoring/map-anchoring.h"

#include <loop-closure-handler/loop-detector-node.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>

DEFINE_bool(
    add_anchored_missions_to_database, true,
    "Add missions that got anchored to the loop detector database too.");

DEFINE_double(
    abs_constraints_baseframe_min_number_of_constraints, 3,
    "Outlier rejection in absolute constraints: Sets the minimum number of "
    "constraints required to perform outlier rejection. If less constraints "
    "are present in the map, the outlier rejection will not do anything.");
DEFINE_double(
    abs_constraints_baseframe_min_inlier_ratio, 0.5,
    "Outlier rejection in absolute constraints: Sets the minimum required "
    "inlier ratio for mission baseframe RANSAC.");
DEFINE_double(
    abs_constraints_baseframe_ransac_max_orientation_error_rad, 0.0872,
    "Outlier rejection in absolute constraints: Sets the maximum orientation "
    "error for inliers for mission baseframe RANSAC.");
DEFINE_double(
    abs_constraints_baseframe_ransac_max_position_error_m, 0.5,
    "Outlier rejection in absolute constraints: Sets the maximum position "
    "error for inliers for mission baseframe RANSAC. ");
DEFINE_int32(
    abs_constraints_baseframe_ransac_num_interations, 2000,
    "Outlier rejection in absolute constraints: Sets the maximum number of "
    "iterations for mission baseframe RANSAC.");

namespace map_anchoring {

void setMissionBaseframeToKnownIfHasAbs6DoFConstraints(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  for (const vi_map::MissionId& mission_id : mission_ids) {
    const bool has_absolute_6dof_constraints =
        map->getNumAbsolute6DoFMeasurementsInMission(mission_id) > 0u;

    if (has_absolute_6dof_constraints) {
      const vi_map::MissionBaseFrameId& mission_baseframe_id =
          map->getMission(mission_id).getBaseFrameId();
      map->getMissionBaseFrame(mission_baseframe_id).set_is_T_G_M_known(true);
    }
  }
}

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

  // How often do we try to anchor. Prevent locking forever.
  constexpr int kTrialsPerMission = 3;

  // Build a list of all unknown baseframes.
  std::queue<vi_map::MissionId> missions_to_anchor;
  std::vector<vi_map::MissionId> abandoned_missions;
  std::unordered_map<vi_map::MissionId, int> remaining_trials_for_mission_map;
  vi_map::MissionIdList all_missions;
  map->getAllMissionIds(&all_missions);
  std::stringstream ss;
  ss << "\n";
  ss << "----------------------------------------------------------------\n";
  ss << "                         MapAnchoring                           \n";
  ss << "----------------------------------------------------------------\n";
  ss << " Current State:\n";
  for (const vi_map::MissionId& mission_id : all_missions) {
    const vi_map::Mission& mission = map->getMission(mission_id);
    const vi_map::MissionBaseFrame& base_frame =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    const bool baseframe_is_known = base_frame.is_T_G_M_known();
    if (!baseframe_is_known) {
      missions_to_anchor.push(mission_id);
      remaining_trials_for_mission_map[mission_id] = kTrialsPerMission;
    }
    ss << "\t- " << mission_id << ":\t"
       << ((baseframe_is_known) ? "anchored\n" : "not anchored\n");
  }
  ss << "----------------------------------------------------------------\n";
  ss << " Result:\n";
  if (missions_to_anchor.size() == all_missions.size()) {
    ss << "\tFailed: At least one mission needs to have a known baseframe. \n";
    ss << "----------------------------------------------------------------\n";
    VLOG(1) << ss.str();
    return false;
  }

  if (missions_to_anchor.empty()) {
    ss << "\tAll baseframes are known -- nothing to do.\n";
    ss << "----------------------------------------------------------------\n";
    VLOG(1) << ss.str();
    return true;
  }

  // A loop-detector to which we add all the missions we have already anchored.
  // This allows us to avoid building the index over and over as we anchor new
  // missions.
  loop_detector_node::LoopDetectorNode loop_detector;
  if (plotter != nullptr) {
    loop_detector.instantiateVisualizer();
  }

  // Add the known missions to the database.
  bool initial_mission_added = false;
  while (!missions_to_anchor.empty()) {
    if (FLAGS_add_anchored_missions_to_database || !initial_mission_added) {
      VLOG(1) << "Adding known missions to loop-detector.";
      // Add all missions that have a known base-frame (if any).
      addAllMissionsWithKnownBaseFrameToProvidedLoopDetector(
          *map, &loop_detector);
      initial_mission_added = true;
    }

    vi_map::MissionId mission_id = missions_to_anchor.front();
    missions_to_anchor.pop();

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
      --(remaining_trials_for_mission_map[mission_id]);

      if (missions_to_anchor.empty()) {
        VLOG(1)
            << "Failed to anchor mission " << mission_id
            << ", since this is the last mission to be anchored, retrying is "
            << "very unlikely to succeed. Anchoring failed!";
        abandoned_missions.push_back(mission_id);
        continue;
      }

      if (remaining_trials_for_mission_map[mission_id] > 0) {
        VLOG(1) << "Failed to anchor mission " << mission_id
                << ", pushing it back to the end of the work-list.";
        missions_to_anchor.push(mission_id);
        continue;
      }

      VLOG(1) << "Used up all (" << kTrialsPerMission << ") trials for mission "
              << mission_id << ". Anchoring failed!";
      abandoned_missions.push_back(mission_id);
      continue;
    }

    if (plotter != nullptr && success) {
      plotter->visualizeMap(*map);
    }
  }
  // There should be nothing left to anchor, either we succeeded or we have run
  // out of trials for each mission.
  CHECK(missions_to_anchor.empty());

  // Reset the selected missions.
  map->resetMissionSelection();

  if (!abandoned_missions.empty()) {
    ss << "\tCould not anchor all missions, still unknown:\n";
    for (const vi_map::MissionId& abandoned_mission_id : abandoned_missions) {
      ss << "\t- " << abandoned_mission_id << "\n";
    }
    ss << "----------------------------------------------------------------\n";
    VLOG(1) << ss.str();
    return false;
  }
  ss << "\tAll missions anchored.\n";
  ss << "----------------------------------------------------------------\n";
  VLOG(1) << ss.str();
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

  // Probe aligning mission.
  pose::Transformation T_G_M;
  static constexpr bool kMergeLandmarks = false;
  static constexpr bool kAddLoopclosureEdges = false;

  vi_map::LoopClosureConstraintVector inlier_constraints;
  const bool success = loop_detector.detectLoopClosuresMissionToDatabase(
      mission_id, kMergeLandmarks, kAddLoopclosureEdges, map, &T_G_M,
      &inlier_constraints);

  if (success) {
    VLOG(1) << "Probe successful, will anchor mission " << mission_id;

    vi_map::VIMission& mission = map->getMission(mission_id);
    vi_map::MissionBaseFrame& mission_baseframe =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    // Prealign.
    mission_baseframe.set_T_G_M(T_G_M);

    // Mark as anchored.
    constexpr bool kIsMissionAnchored = true;
    map->getMissionBaseFrame(mission.getBaseFrameId())
        .set_is_T_G_M_known(kIsMissionAnchored);

    return true;
  }
  VLOG(1) << "Probe did not meet criteria, will not anchor mission "
          << mission_id;
  return false;
}

void removeOutliersInAbsolute6DoFConstraints(vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  LOG(INFO) << "Removing outliers from absolute pose constraints";

  std::stringstream ss;
  ss << "\n";
  ss << "----------------------------------------------------------------\n";
  ss << "           Absolute Pose Constraints  - Outlier Removal         \n";
  ss << "----------------------------------------------------------------\n";

  vi_map::MissionIdList missions_to_optimize;
  map->getAllMissionIds(&missions_to_optimize);
  for (const vi_map::MissionId& mission_id : missions_to_optimize) {
    const vi_map::VIMission& mission = map->getMission(mission_id);
    if (!mission.hasAbsolute6DoFSensor()) {
      continue;
    }

    // Get vertices.
    pose_graph::VertexIdList vertices;
    map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertices);

    // Get absolute pose sensor.
    const aslam::SensorId& sensor_id = mission.getAbsolute6DoFSensor();
    const aslam::Transformation T_S_B =
        map->getSensorManager().getSensor_T_B_S(sensor_id).inverse();

    aslam::TransformationVector T_G_M_vector;
    std::vector<std::pair<pose_graph::VertexId, uint32_t>>
        vertex_id_and_abs_constraint_idx;

    for (const pose_graph::VertexId& vertex_id : vertices) {
      const vi_map::Vertex& vertex = map->getVertex(vertex_id);
      const std::vector<vi_map::Absolute6DoFMeasurement>&
          abs_6dof_measurements = vertex.getAbsolute6DoFMeasurements();
      const aslam::Transformation& T_B_M = vertex.get_T_M_I().inverse();

      uint32_t abs_constraint_idx = 0u;
      for (const vi_map::Absolute6DoFMeasurement& abs_6dof_measurement :
           abs_6dof_measurements) {
        const aslam::Transformation& T_G_S = abs_6dof_measurement.get_T_G_S();
        const aslam::Transformation T_G_M = T_G_S * T_S_B * T_B_M;

        T_G_M_vector.push_back(T_G_M);
        vertex_id_and_abs_constraint_idx.emplace_back(
            vertex_id, abs_constraint_idx);

        ++abs_constraint_idx;
      }
    }

    const uint32_t num_abs_constraints = T_G_M_vector.size();
    ss << "\t- " << mission_id << " has " << num_abs_constraints
       << " absolute pose constraints\n";

    // Check flags.
    CHECK_GE(FLAGS_abs_constraints_baseframe_min_number_of_constraints, 3);
    CHECK_GE(FLAGS_abs_constraints_baseframe_min_inlier_ratio, 0.0);
    CHECK_GE(FLAGS_abs_constraints_baseframe_ransac_num_interations, 0);
    CHECK_GE(
        FLAGS_abs_constraints_baseframe_ransac_max_orientation_error_rad, 0.0);
    CHECK_GE(FLAGS_abs_constraints_baseframe_ransac_max_position_error_m, 0.0);

    if (num_abs_constraints <
        FLAGS_abs_constraints_baseframe_min_number_of_constraints) {
      ss << "\t  "
         << "-> Failed: not enough constraints (< "
         << FLAGS_abs_constraints_baseframe_min_number_of_constraints << ")\n";
      ss << "----------------------------------------------------------------"
            "\n";
      continue;
    }

    // RANSAC and LSQ estimate of the mission baseframe transformation.
    const int kNumInliersThreshold =
        num_abs_constraints * FLAGS_abs_constraints_baseframe_min_inlier_ratio;
    aslam::Transformation T_G_M_LS;
    int num_inliers = 0;
    std::random_device device;
    const int ransac_seed = device();
    std::unordered_set<int> inlier_indices;

    common::transformationRansac(
        T_G_M_vector, FLAGS_abs_constraints_baseframe_ransac_num_interations,
        FLAGS_abs_constraints_baseframe_ransac_max_orientation_error_rad,
        FLAGS_abs_constraints_baseframe_ransac_max_position_error_m,
        ransac_seed, &T_G_M_LS, &num_inliers, &inlier_indices);

    ss << "\t  -> Inliers " << num_inliers << "/" << num_abs_constraints
       << "\n";
    if (num_inliers < kNumInliersThreshold) {
      ss << "\t  -> Failed: Not enough inliers (" << kNumInliersThreshold
         << "<)\n";
      ss << "----------------------------------------------------------------"
            "\n";

      continue;
    }

    // Remove outliers.
    CHECK_EQ(vertex_id_and_abs_constraint_idx.size(), num_abs_constraints);
    for (uint32_t global_idx = 0u; global_idx < num_abs_constraints;
         ++global_idx) {
      if (inlier_indices.count(global_idx) > 0u) {
        continue;
      }
      const std::pair<pose_graph::VertexId, uint32_t>
          vertex_id_with_constraint_idx =
              vertex_id_and_abs_constraint_idx[global_idx];
      const pose_graph::VertexId vertex_id =
          vertex_id_with_constraint_idx.first;
      const uint32_t constraint_idx = vertex_id_with_constraint_idx.second;

      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      std::vector<vi_map::Absolute6DoFMeasurement>& abs_6dof_measurements =
          vertex.getAbsolute6DoFMeasurements();

      abs_6dof_measurements.erase(
          abs_6dof_measurements.begin() + constraint_idx);
    }
    const uint32_t num_outliers = num_abs_constraints - num_inliers;
    ss << "\t  -> Outliers " << num_outliers << "/" << num_abs_constraints
       << " -> REMOVED\n";
    ss << "----------------------------------------------------------------\n";
  }
  LOG(INFO) << ss.str();
}

}  // namespace map_anchoring
