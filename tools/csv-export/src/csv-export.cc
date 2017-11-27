#include "csv-export/csv-export.h"

#include <fstream>  // NOLINT
#include <gflags/gflags.h>

#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>
#include <maplab-common/binary-serialization.h>
#include <maplab-common/conversions.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/landmark-quality-metrics.h>

DEFINE_bool(
    only_export_high_quality_landmarks, false,
    "If true export only "
    "landmarks where the quality metrics return true.");
DEFINE_bool(
    export_absolute_time, false,
    "Export absolute time instead of time relative to the first "
    "frame.");
DEFINE_bool(
    export_projected_descriptor, false,
    "If true export projected "
    "descriptor, otherwise export raw descriptor.");

namespace csv_export {
namespace {

static constexpr char kDelimiter[] = ", ";

typedef std::unordered_map<pose_graph::VertexId, size_t> VertexIdToIndexMap;

void exportVerticesAndTracksToCsv(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const std::string& base_path, size_t* vertex_index,
    VertexIdToIndexMap* vertex_id_to_index_map) {
  CHECK_NOTNULL(vertex_index);
  CHECK_NOTNULL(vertex_id_to_index_map);

  const std::string path_vertices =
      common::concatenateFolderAndFileName(base_path, "vertices.csv");
  common::FileLogger logger_vertices(path_vertices);
  logger_vertices.writeDataWithDelimiterAndNewLine(
      kDelimiter, "vertex index", "timestamp [ns]", "position x [m]",
      "position y [m]", "position z [m]", "quaternion x", "quaternion y",
      "quaternion z", "quaternion w", "velocity x [m/s]", "velocity y [m/s]",
      "velocity z [m/s]", "acc bias x [m/s^2]", "acc bias y [m/s^2]",
      "acc bias z [m/s^2]", "gyro bias x [rad/s]", "gyro bias y [rad/s]",
      "gyro bias z [rad/s]");

  const std::string path_tracks =
      common::concatenateFolderAndFileName(base_path, "tracks.csv");
  common::FileLogger logger_tracks(path_tracks);
  logger_tracks.writeDataWithDelimiterAndNewLine(
      kDelimiter, "timestamp [ns]", "vertex index", "frame index",
      "keypoint index", "keypoint measurement 0 [px]",
      "keypoint measurement 1 [px]", "keypoint measurement uncertainty",
      "keypoint scale", "keypoint track id");

  const std::string path_descriptor =
      common::concatenateFolderAndFileName(base_path, "descriptor.csv");
  common::FileLogger logger_descriptor(path_descriptor);
  logger_tracks.writeDataWithDelimiterAndNewLine(
      kDelimiter, "Descriptor byte as integer 1-N");

  pose_graph::VertexIdList vertex_ids_in_mission;
  map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids_in_mission);

  common::ProgressBar progress_bar(vertex_ids_in_mission.size());
  for (const pose_graph::VertexId& vertex_id : vertex_ids_in_mission) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);

    // Write vertex data itself.
    const aslam::Transformation T_G_I = map.getVertex_T_G_I(vertex_id);
    const Eigen::Vector3d& v_M = vertex.get_v_M();
    const Eigen::Vector3d& acc_bias = vertex.getAccelBias();
    const Eigen::Vector3d& gyro_bias = vertex.getGyroBias();

    logger_vertices.writeDataWithDelimiterAndNewLine(
        kDelimiter, *vertex_index, vertex.getMinTimestampNanoseconds(),
        T_G_I.getPosition(), T_G_I.getEigenQuaternion(), v_M, acc_bias,
        gyro_bias);

    vertex_id_to_index_map->emplace(vertex_id, *vertex_index);

    // Frame data.
    vertex.forEachFrame(
        [&](const unsigned int frame_index, const aslam::VisualFrame& frame) {
          const size_t num_keypoints = frame.getNumKeypointMeasurements();
          const size_t num_bytes_per_descriptor =
              frame.getDescriptorSizeBytes();

          const int64_t timestamp_ns = frame.getTimestampNanoseconds();
          const Eigen::Matrix2Xd& keypoint_measurements =
              frame.getKeypointMeasurements();
          const Eigen::VectorXd& keypoint_measurment_uncertainties =
              frame.getKeypointMeasurementUncertainties();
          const Eigen::VectorXd& keypoint_scales = frame.getKeypointScales();
          const Eigen::VectorXi& track_ids = frame.getTrackIds();
          for (size_t keypoint_idx = 0u; keypoint_idx < num_keypoints;
               ++keypoint_idx) {
            logger_tracks.writeDataWithDelimiterAndNewLine(
                kDelimiter, timestamp_ns, *vertex_index, frame_index,
                keypoint_idx, keypoint_measurements(0, keypoint_idx),
                keypoint_measurements(1, keypoint_idx),
                keypoint_measurment_uncertainties(keypoint_idx),
                keypoint_scales(keypoint_idx), track_ids(keypoint_idx));

            const unsigned char* descriptor =
                CHECK_NOTNULL(frame.getDescriptor(keypoint_idx));
            for (size_t descriptor_idx = 0u;
                 descriptor_idx < num_bytes_per_descriptor; ++descriptor_idx) {
              if (descriptor_idx != 0u) {
                logger_descriptor << kDelimiter;
              }
              logger_descriptor
                  << static_cast<size_t>(descriptor[descriptor_idx]);
            }
            logger_descriptor << std::endl;
          }
        });

    ++(*vertex_index);
    progress_bar.increment();
  }
}

void exportLandmarksAndObservationsToCsv(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const std::string& base_path,
    const VertexIdToIndexMap& vertex_id_to_index_map, size_t* landmark_index) {
  CHECK_NOTNULL(landmark_index);

  const std::string path_landmarks =
      common::concatenateFolderAndFileName(base_path, "landmarks.csv");
  common::FileLogger logger_landmarks(path_landmarks);
  logger_landmarks.writeDataWithDelimiterAndNewLine(
      kDelimiter, "landmark index", "landmark position x [m]",
      "landmark position y [m]", "landmark position z [m]");

  const std::string path_observations =
      common::concatenateFolderAndFileName(base_path, "observations.csv");
  common::FileLogger logger_observations(path_observations);
  logger_observations.writeDataWithDelimiterAndNewLine(
      kDelimiter, "vertex index", "frame index", "keypoint index",
      "landmark index");

  vi_map::LandmarkIdList all_landmarks_in_mission;
  map.getAllLandmarkIdsInMission(mission_id, &all_landmarks_in_mission);
  for (const vi_map::LandmarkId& landmark_id : all_landmarks_in_mission) {
    if (!landmark_id.isValid()) {
      continue;
    }
    if (FLAGS_only_export_high_quality_landmarks) {
      if (!vi_map::isLandmarkWellConstrained(
              map, map.getLandmark(landmark_id))) {
        continue;
      }
    }

    const vi_map::Landmark& landmark = map.getLandmark(landmark_id);
    const Eigen::Vector3d landmark_position =
        map.getLandmark_G_p_fi(landmark_id);

    logger_landmarks.writeDataWithDelimiterAndNewLine(
        kDelimiter, *landmark_index, landmark_position);

    // Write observations.
    const vi_map::KeypointIdentifierList& keypoint_identifier_list =
        landmark.getObservations();
    for (const vi_map::KeypointIdentifier& keypoint_identifier :
         keypoint_identifier_list) {
      const pose_graph::VertexId& vertex_id =
          keypoint_identifier.frame_id.vertex_id;
      const VertexIdToIndexMap::const_iterator it_vertex_id_to_index =
          vertex_id_to_index_map.find(vertex_id);
      CHECK(it_vertex_id_to_index != vertex_id_to_index_map.end());
      logger_observations.writeDataWithDelimiterAndNewLine(
          kDelimiter, it_vertex_id_to_index->second,
          keypoint_identifier.frame_id.frame_index,
          keypoint_identifier.keypoint_index, *landmark_index);
    }

    ++(*landmark_index);
  }
}

void exportImuDataToCsv(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const std::string& base_path) {
  const std::string path_imu =
      common::concatenateFolderAndFileName(base_path, "imu.csv");
  common::FileLogger logger_imu(path_imu);
  logger_imu.writeDataWithDelimiterAndNewLine(
      kDelimiter, "timestamp [ns]", "acc x [m/s^2]", "acc y [m/s^2]",
      "acc z [m/s^2]", "gyro x [rad/s]", "gyro y [rad/s]", "gyro z [rad/s]");
  pose_graph::EdgeIdList all_edges_in_mission;
  map.getAllEdgeIdsInMissionAlongGraph(mission_id, &all_edges_in_mission);
  for (const pose_graph::EdgeId& edge_id : all_edges_in_mission) {
    if (map.getEdgeType(edge_id) == vi_map::Edge::EdgeType::kViwls) {
      const vi_map::ViwlsEdge& viwls_edge =
          map.getEdgeAs<vi_map::ViwlsEdge>(edge_id);
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps =
          viwls_edge.getImuTimestamps();
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data =
          viwls_edge.getImuData();

      const int num_measurements = imu_timestamps.cols();
      CHECK_EQ(num_measurements, imu_data.cols());
      for (int i = 0; i < num_measurements; ++i) {
        logger_imu.writeDataWithDelimiterAndNewLine(
            kDelimiter, imu_timestamps(i), imu_data.col(i));
      }
    }
  }
}

}  // namespace

void exportMapToCsv(const vi_map::VIMap& map, const std::string& base_path) {
  CHECK(!base_path.empty());
  CHECK(common::createPath(base_path));

  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);

  VertexIdToIndexMap vertex_id_to_index_map;
  size_t vertex_index = 0u;
  size_t landmark_index = 0u;

  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    const std::string base_path_for_mission =
        common::concatenateFolderAndFileName(base_path, mission_id.hexString());
    CHECK(common::createPath(base_path_for_mission));

    LOG(INFO) << "Exporting data from mission " << mission_id << ".";

    exportVerticesAndTracksToCsv(
        map, mission_id, base_path_for_mission, &vertex_index,
        &vertex_id_to_index_map);

    exportLandmarksAndObservationsToCsv(
        map, mission_id, base_path_for_mission, vertex_id_to_index_map,
        &landmark_index);

    exportImuDataToCsv(map, mission_id, base_path_for_mission);
  }
}

}  // namespace csv_export
