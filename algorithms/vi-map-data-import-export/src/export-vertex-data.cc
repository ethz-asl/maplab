#include "vi-map-data-import-export/export-vertex-data.h"

#include <aslam/common/unique-id.h>
#include <console-common/command-registerer.h>
#include <glog/logging.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>
#include <sensors/sensor-types.h>
#include <vi-map/vi-map.h>

namespace data_import_export {

char convertSensorTypeToFrameIdentifier(const vi_map::SensorType sensor_type) {
  switch (sensor_type) {
    case vi_map::SensorType::kNCamera:
    case vi_map::SensorType::kCamera:
      return 'C';
      break;
    case vi_map::SensorType::kImu:
      return 'I';
      break;
    case vi_map::SensorType::kLoopClosureSensor:
    case vi_map::SensorType::kGpsUtm:
    case vi_map::SensorType::kGpsWgs:
    case vi_map::SensorType::kWheelOdometry:
    case vi_map::SensorType::kAbsolute6DoF:
      return 'B';
      break;
    default:
      LOG(FATAL) << "Unknown sensor type: " << static_cast<int>(sensor_type);
      break;
  }

  return '\0';
}

int exportPosesVelocitiesAndBiasesToCsv(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    const aslam::Transformation& T_I_S, const char sensor_frame_identifier,
    const std::string& pose_export_file,
    const std::string& format /* = "asl" */) {
  pose_graph::VertexIdList vertex_ids;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());
    pose_graph::VertexIdList vertex_ids_along_mission_graph;
    map.getAllVertexIdsInMissionAlongGraph(
        mission_id, &vertex_ids_along_mission_graph);
    vertex_ids.insert(
        vertex_ids.end(), vertex_ids_along_mission_graph.begin(),
        vertex_ids_along_mission_graph.end());
  }

  aslam::TransformationVector T_M_B_vector;
  pose_graph::VertexIdList all_mission_vertices;
  std::vector<Eigen::Matrix<int64_t, 1, Eigen::Dynamic>> timestamps;
  std::vector<aslam::TransformationVector> T_M_Bs;
  const landmark_triangulation::PoseInterpolator pose_interpolator;
  map.getAllVertexIdsInMissionAlongGraph(mission_ids[0], &all_mission_vertices);
  for (const pose_graph::VertexId& vertex_id : all_mission_vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    pose_graph::EdgeIdSet outgoing_edges;
    vertex.getOutgoingEdges(&outgoing_edges);
    pose_graph::EdgeId outgoing_imu_edge_id;
    for (const pose_graph::EdgeId& edge_id : outgoing_edges) {
      if (map.getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kViwls) {
        outgoing_imu_edge_id = edge_id;
        break;
      }
    }
    // We must have reached the end of the graph.
    if (!outgoing_imu_edge_id.isValid()) {
      break;
    }

    const vi_map::ViwlsEdge& imu_edge =
        map.getEdgeAs<vi_map::ViwlsEdge>(outgoing_imu_edge_id);
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps =
        imu_edge.getImuTimestamps();
    timestamps.push_back(imu_timestamps);
    aslam::TransformationVector imu_poses_vector;
    const bool interpolation_succeeded = pose_interpolator.getPosesAtTime(
        map, mission_ids[0], imu_timestamps, &imu_poses_vector);
    T_M_Bs.push_back(imu_poses_vector);
  }

  if (pose_export_file.empty()) {
    LOG(ERROR) << "Need to specify file path with the flag -pose_export_file.";
    return common::kUnknownError;
  }

  common::FileLogger csv_file(pose_export_file);
  if (!csv_file.isOpen()) {
    LOG(ERROR) << "Failed to open logger to file: '" << pose_export_file << "'";
    return common::kUnknownError;
  }
  LOG(INFO) << "Exporting poses, velocities and biases in " << format
            << " format to: " << pose_export_file;
  std::string kDelimiter;
  if (format == "asl") {
    kDelimiter = ", ";
    const std::string kSensorFrameIdentifier =
        std::string(1, sensor_frame_identifier);
    csv_file.writeDataWithDelimiterAndNewLine(
        kDelimiter, "# timestamp [ns]", "vertex-id", "mission-id",
        "p_G_" + kSensorFrameIdentifier + "x [m]",
        "p_G_" + kSensorFrameIdentifier + "y [m]",
        "p_G_" + kSensorFrameIdentifier + "z [m]",
        "q_G_" + kSensorFrameIdentifier + "w",
        "q_G_" + kSensorFrameIdentifier + "x",
        "q_G_" + kSensorFrameIdentifier + "y",
        "q_G_" + kSensorFrameIdentifier + "z",
        "p_M_" + kSensorFrameIdentifier + "x [m]",
        "p_M_" + kSensorFrameIdentifier + "y [m]",
        "p_M_" + kSensorFrameIdentifier + "z [m]",
        "q_M_" + kSensorFrameIdentifier + "w",
        "q_M_" + kSensorFrameIdentifier + "x",
        "q_M_" + kSensorFrameIdentifier + "y",
        "q_M_" + kSensorFrameIdentifier + "z", "v_Mx [m/s]", "v_My [m/s]",
        "v_Mz [m/s]", "bgx [rad/s]", "bgy [rad/s]", "bgz [rad/s]",
        "bax [m/s^2]", "bay [m/s^2]", "baz [m/s^2]");
  } else if (format == "rpg") {
    kDelimiter = " ";
    const std::string kSensorFrameIdentifier =
        std::string(1, sensor_frame_identifier);
    csv_file.writeDataWithDelimiterAndNewLine(
        kDelimiter, "timestamp_ns", "p_G_" + kSensorFrameIdentifier + "x",
        "p_G_" + kSensorFrameIdentifier + "y",
        "p_G_" + kSensorFrameIdentifier + "z",
        "q_G_" + kSensorFrameIdentifier + "x",
        "q_G_" + kSensorFrameIdentifier + "y",
        "q_G_" + kSensorFrameIdentifier + "z",
        "q_G_" + kSensorFrameIdentifier + "w");
  } else {
    LOG(ERROR) << "Invalid format " << format;
    return common::kStupidUserError;
  }

  size_t counter = 0u;
  for (size_t v_idx = 0u; v_idx < T_M_Bs.size(); ++v_idx) {
    for (size_t i_idx = 0u; i_idx < T_M_Bs[v_idx].size(); ++i_idx) {
      // LOG(WARNING) << timestamps[v_idx](0,i_idx);
      LOG(WARNING) << T_M_Bs[v_idx][i_idx].getPosition();
      ++counter;
      const auto x = T_M_Bs[v_idx][i_idx].getPosition().x();
      const auto y = T_M_Bs[v_idx][i_idx].getPosition().y();
      const auto z = T_M_Bs[v_idx][i_idx].getPosition().z();
      const auto qx = T_M_Bs[v_idx][i_idx].getRotation().x();
      const auto qy = T_M_Bs[v_idx][i_idx].getRotation().y();
      const auto qz = T_M_Bs[v_idx][i_idx].getRotation().z();
      const auto qw = T_M_Bs[v_idx][i_idx].getRotation().w();
      csv_file.writeDataWithDelimiterAndNewLine(
          kDelimiter, timestamps[v_idx](0, i_idx), 0, "abcdef", x, y, z, qw, qx,
          qy, qz, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  }
  LOG(WARNING) << "imu ts: " << counter;

  // for (const pose_graph::VertexId& vertex_id : vertex_ids) {
  //   CHECK(vertex_id.isValid());
  //   const vi_map::Vertex& vertex = map.getVertex(vertex_id);
  //   const vi_map::MissionId& mission_id = vertex.getMissionId();
  //   CHECK(mission_id.isValid());
  //
  //   const int64_t timestamp_nanoseconds =
  //   vertex.getMinTimestampNanoseconds(); const aslam::Transformation T_G_I =
  //   map.getVertex_T_G_I(vertex_id); const aslam::Transformation T_G_S = T_G_I
  //   * T_I_S; const Eigen::Vector3d& p_G_S = T_G_S.getPosition(); const
  //   kindr::minimal::RotationQuaternion& q_G_S = T_G_S.getRotation(); const
  //   aslam::Transformation& T_M_I = vertex.get_T_M_I(); const
  //   aslam::Transformation T_M_S = T_M_I * T_I_S; const Eigen::Vector3d& p_M_S
  //   = T_M_S.getPosition(); const kindr::minimal::RotationQuaternion& q_M_S =
  //   T_M_S.getRotation(); const Eigen::Vector3d& v_M = vertex.get_v_M(); const
  //   Eigen::Vector3d& gyro_bias = vertex.getGyroBias(); const Eigen::Vector3d&
  //   acc_bias = vertex.getAccelBias(); if (format == "asl") {
  //     csv_file.writeDataWithDelimiterAndNewLine(
  //         kDelimiter, timestamp_nanoseconds, vertex_id.hexString(),
  //         mission_id.hexString(), p_G_S[0], p_G_S[1], p_G_S[2], q_G_S.w(),
  //         q_G_S.x(), q_G_S.y(), q_G_S.z(), p_M_S[0], p_M_S[1], p_M_S[2],
  //         q_M_S.w(), q_M_S.x(), q_M_S.y(), q_M_S.z(), v_M[0], v_M[1], v_M[2],
  //         gyro_bias[0], gyro_bias[1], gyro_bias[2], acc_bias[0], acc_bias[1],
  //         acc_bias[2]);
  //   } else if (format == "rpg") {
  //     csv_file.writeDataWithDelimiterAndNewLine(
  //         kDelimiter, timestamp_nanoseconds, p_G_S[0], p_G_S[1], p_G_S[2],
  //         q_G_S.x(), q_G_S.y(), q_G_S.z(), q_G_S.w());
  //   } else {
  //     LOG(ERROR) << "Invalid format " << format;
  //     return common::kStupidUserError;
  //   }
  // }

  return common::kSuccess;
}

int exportPosesVelocitiesAndBiasesToCsv(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    const aslam::SensorId& reference_sensor_id,
    const std::string& pose_export_file,
    const std::string& format /* = "asl" */) {
  CHECK(reference_sensor_id.isValid());

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  const vi_map::SensorType sensor_type =
      sensor_manager.getSensorType(reference_sensor_id);
  CHECK(isValidSensorType(sensor_type));

  const aslam::Transformation& T_I_S =
      sensor_manager.getSensor_T_B_S(reference_sensor_id);

  const char sensor_frame_identifier =
      convertSensorTypeToFrameIdentifier(sensor_type);

  return exportPosesVelocitiesAndBiasesToCsv(
      map, mission_ids, T_I_S, sensor_frame_identifier, pose_export_file,
      format);
}

}  // namespace data_import_export
