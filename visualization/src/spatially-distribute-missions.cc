#include "visualization/spatially-distribute-missions.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_double(
    spatially_distribute_missions_meters, 20,
    "Amount to shift missions when distributing them spatially.");
DEFINE_int32(
    spatially_distribute_missions_dimension, 0,
    "Dimension to shift along [x, y, z] = [0, 1, 2].");
DEFINE_bool(
    spatially_distribute_missions_around_circle, false,
    "Should sdm distribute the missions around a circle.");

namespace visualization {
bool spatiallyDistributeMissions(vi_map::VIMap* map) {
  typedef std::pair<double, vi_map::MissionId> LengthAndMission;
  std::vector<LengthAndMission> lengths_and_missions;

  vi_map::MissionIdList all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);

  vi_map::MissionIdList missions_with_unknown_baseframe;
  vi_map::MissionIdList missions_with_known_baseframe;
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    const vi_map::VIMission& mission = map->getMission(mission_id);
    vi_map::MissionBaseFrame& baseframe =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    if (baseframe.is_T_G_M_known()) {
      missions_with_known_baseframe.emplace_back(mission_id);
    } else {
      missions_with_unknown_baseframe.emplace_back(mission_id);
    }
  }

  if (missions_with_unknown_baseframe.empty()) {
    return false;
  }

  // Compute the average vertex position of the vertices from missions with
  // known base-frames.
  Eigen::Vector3d mean_vertex_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_baseframe_position = Eigen::Vector3d::Zero();
  size_t num_samples = 0u;
  for (const vi_map::MissionId& mission_id : missions_with_known_baseframe) {
    pose_graph::VertexIdList mission_vertices;
    map->getAllVertexIdsInMission(mission_id, &mission_vertices);
    for (const pose_graph::VertexId& vertex_id : mission_vertices) {
      mean_vertex_position += map->getVertex_G_p_I(vertex_id);
      ++num_samples;
    }
    mean_baseframe_position +=
        map->getMissionBaseFrameForMission(mission_id).get_p_G_M();
  }
  if (num_samples > 0u) {
    mean_vertex_position /= num_samples;
    mean_baseframe_position /= num_samples;
  }

  if (FLAGS_spatially_distribute_missions_around_circle) {
    VLOG(1) << "Will distribute the missions around a circle.";
    VLOG(2) << "Mean position of known mission vertices "
            << mean_vertex_position.transpose();
  } else {
    VLOG(1) << "Will distribute missions along one dimension.";
    VLOG(2) << "Mean position of known mission baseframes "
            << mean_baseframe_position.transpose();
  }

  CHECK_GE(FLAGS_spatially_distribute_missions_dimension, 0);
  CHECK_LE(FLAGS_spatially_distribute_missions_dimension, 2);

  const double radians_per_mission =
      2. * M_PI / missions_with_unknown_baseframe.size();
  for (size_t i = 0u; i < missions_with_unknown_baseframe.size(); ++i) {
    const vi_map::MissionId& mission_id = missions_with_unknown_baseframe[i];
    const vi_map::VIMission& mission = map->getMission(mission_id);
    vi_map::MissionBaseFrame& baseframe =
        map->getMissionBaseFrame(mission.getBaseFrameId());
    Eigen::Vector3d p_GM = baseframe.get_p_G_M();

    if (FLAGS_spatially_distribute_missions_around_circle) {
      p_GM(0) =
          mean_vertex_position(0) + FLAGS_spatially_distribute_missions_meters *
                                        cos(i * radians_per_mission);
      p_GM(1) =
          mean_vertex_position(1) + FLAGS_spatially_distribute_missions_meters *
                                        sin(i * radians_per_mission);
    } else {
      p_GM(FLAGS_spatially_distribute_missions_dimension) =
          mean_baseframe_position(
              FLAGS_spatially_distribute_missions_dimension) +
          FLAGS_spatially_distribute_missions_meters * (i + 1);
    }
    baseframe.set_p_G_M(p_GM);
  }

  return true;
}
}  // namespace visualization
