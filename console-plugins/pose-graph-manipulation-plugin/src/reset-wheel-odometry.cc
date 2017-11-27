#include "pose-graph-manipulation-plugin/reset-wheel-odometry.h"

#include <console-common/command-registerer.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/sensor-manager.h>
#include <vi-map/vi-map.h>

namespace pose_graph_manipulation {

int resetVertexPosesToWheelOdometryTrajectory(vi_map::VIMap *map) {
  CHECK_NOTNULL(map);
  const vi_map::SensorManager& sensor_manager = map->getSensorManager();
  vi_map::SensorIdSet wheel_odometry_sensor_ids;
  sensor_manager.getAllSensorIdsOfType(
      vi_map::SensorType::kRelative6DoFPose, &wheel_odometry_sensor_ids);

  if (wheel_odometry_sensor_ids.empty()) {
    LOG(ERROR) << "The given map does not contain any wheel-odometry sensors.";
    return common::kStupidUserError;
  }

  AlignedUnorderedMap<vi_map::SensorId, aslam::Transformation>
      wheel_odometry_sensor_id_to_T_S_I_map;
  for (const vi_map::SensorId& wheel_odometry_sensor_id :
       wheel_odometry_sensor_ids) {
    CHECK(wheel_odometry_sensor_id.isValid());
    aslam::Transformation T_R_S;
    CHECK(sensor_manager.getSensor_T_R_S(wheel_odometry_sensor_id, &T_R_S));
    // S signifies "Sensor" frame.
    // R signifies the sensor extrinsics reference frame. In our case: The
    // IMU frame I.
    const aslam::Transformation T_S_I = T_R_S.inverse();
    CHECK(
        wheel_odometry_sensor_id_to_T_S_I_map
            .emplace(wheel_odometry_sensor_id, T_S_I)
            .second);
  }

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  VLOG(1) << "Found wheel-odometry in map with " << mission_ids.size()
          << " missions.";

  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());

    pose_graph::EdgeIdList wheel_odometry_edges;
    map->getAllEdgeIdsInMissionAlongGraph(
        mission_id, pose_graph::Edge::EdgeType::kOdometry,
        &wheel_odometry_edges);

    VLOG(2) << "Have " << wheel_odometry_edges.size() << " wheel-odometry "
            << "edges for mission " << mission_id.hexString();
    common::ProgressBar progress_bar(wheel_odometry_edges.size());

    aslam::Transformation T_O_Sk;
    bool first = true;
    for (const pose_graph::EdgeId& edge_id : wheel_odometry_edges) {
      CHECK(edge_id.isValid());
      const vi_map::TransformationEdge& wheel_odometry_edge =
          map->getEdgeAs<vi_map::TransformationEdge>(edge_id);

      const vi_map::SensorId& sensor_id = wheel_odometry_edge.getSensorId();
      CHECK(sensor_id.isValid());
      AlignedUnorderedMap<vi_map::SensorId,
                          aslam::Transformation>::const_iterator
          extrinsics_iterator =
              wheel_odometry_sensor_id_to_T_S_I_map.find(sensor_id);
      CHECK(extrinsics_iterator != wheel_odometry_sensor_id_to_T_S_I_map.end())
          << "Could not find an extrinsics transformation for sensor with id: "
          << sensor_id.hexString() << '.';
      const aslam::Transformation& T_S_I = extrinsics_iterator->second;
      if (first) {
        map->getVertex(wheel_odometry_edge.from()).set_T_M_I(T_O_Sk * T_S_I);
        first = false;
      }
      const aslam::Transformation& T_Sk_Skp1 = wheel_odometry_edge.getT_A_B();
      map->getVertex(wheel_odometry_edge.to())
          .set_T_M_I(T_O_Sk * T_Sk_Skp1 * T_S_I);

      T_O_Sk = T_O_Sk * T_Sk_Skp1;
      progress_bar.increment();
    }
  }
  return common::kSuccess;
}

}  // namespace pose_graph_manipulation
