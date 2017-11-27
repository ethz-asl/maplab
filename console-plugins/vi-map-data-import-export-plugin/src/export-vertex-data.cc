#include "vi-map-data-import-export-plugin/export-vertex-data.h"

#include <console-common/command-registerer.h>
#include <glog/logging.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/vi-map.h>

namespace data_import_export {

int exportPosesVelocitiesAndBiasesToCsv(
    const vi_map::VIMap& map, const vi_map::MissionIdList& mission_ids,
    const std::string& pose_export_file) {
  pose_graph::VertexIdList vertex_ids;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());
    pose_graph::VertexIdList vertex_ids_along_mission_graph;
    map.getAllVertexIdsInMissionAlongGraph(mission_id,
                                           &vertex_ids_along_mission_graph);
    vertex_ids.insert(vertex_ids.end(), vertex_ids_along_mission_graph.begin(),
                      vertex_ids_along_mission_graph.end());
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
  LOG(INFO) << "Exporting poses, velocities and biases to: "
            << pose_export_file;

  const char kDelimiter = ',';

  csv_file << std::string("# timestamp [ns]") << kDelimiter << " vertex-id"
           << kDelimiter << " p_G_Ix [m]" << kDelimiter << " p_G_Iy [m]"
           << kDelimiter << " p_G_Iz [m]" << kDelimiter << " q_G_Iw"
           << kDelimiter << " q_G_Ix" << kDelimiter << " q_G_Iy" << kDelimiter
           << " q_G_Iz" << kDelimiter << " p_M_Ix [m]" << kDelimiter
           << " p_M_Iy [m]" << kDelimiter << " p_M_Iz [m]" << kDelimiter
           << " q_M_Iw" << kDelimiter << " q_M_Ix" << kDelimiter << " q_M_Iy"
           << kDelimiter << " q_M_Iz" << kDelimiter << " v_M_Ix [m/s]"
           << kDelimiter << " v_M_Iy [m/s]" << kDelimiter << " v_M_Iz [m/s]"
           << kDelimiter << " bgx [rad/s]" << kDelimiter << " bgy [rad/s]"
           << kDelimiter << " bgz [rad/s]" << kDelimiter << " bax [m/s^2]"
           << kDelimiter << " bay [m/s^2]" << kDelimiter << " baz [m/s^2]\n";
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    CHECK(vertex_id.isValid());
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    const int64_t timestamp_nanoseconds = vertex.getMinTimestampNanoseconds();
    const aslam::Transformation T_G_I = map.getVertex_T_G_I(vertex_id);
    const Eigen::Vector3d& p_G_I = T_G_I.getPosition();
    const kindr::minimal::RotationQuaternion& q_G_I = T_G_I.getRotation();
    const aslam::Transformation& T_M_I = vertex.get_T_M_I();
    const Eigen::Vector3d& p_M_I = T_M_I.getPosition();
    const kindr::minimal::RotationQuaternion& q_M_I = T_M_I.getRotation();
    const Eigen::Vector3d& v_M = vertex.get_v_M();
    const Eigen::Vector3d& gyro_bias = vertex.getGyroBias();
    const Eigen::Vector3d& acc_bias = vertex.getAccelBias();

    csv_file << timestamp_nanoseconds << kDelimiter << vertex_id.hexString()
             << kDelimiter << p_G_I[0] << kDelimiter << p_G_I[1] << kDelimiter
             << p_G_I[2] << kDelimiter << q_G_I.w() << kDelimiter << q_G_I.x()
             << kDelimiter << q_G_I.y() << kDelimiter << q_G_I.z() << kDelimiter
             << p_M_I[0] << kDelimiter << p_M_I[1] << kDelimiter << p_M_I[2]
             << kDelimiter << q_M_I.w() << kDelimiter << q_M_I.x() << kDelimiter
             << q_M_I.y() << kDelimiter << q_M_I.z() << kDelimiter << v_M[0]
             << kDelimiter << v_M[1] << kDelimiter << v_M[2] << kDelimiter
             << gyro_bias[0] << kDelimiter << gyro_bias[1] << kDelimiter
             << gyro_bias[2] << kDelimiter << acc_bias[0] << kDelimiter
             << acc_bias[1] << kDelimiter << acc_bias[2] << std::endl;
  }
  return common::kSuccess;
}

}  // namespace data_import_export
