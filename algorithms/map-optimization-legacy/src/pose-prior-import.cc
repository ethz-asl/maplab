#include <fstream>   // NOLINT
#include <iostream>  // NOLINT

#include <aslam/common/time.h>

#include "map-optimization-legacy/ba-optimization-options.h"
#include "map-optimization-legacy/graph-ba-optimizer.h"
#include "map-optimization-legacy/pose-prior-import.h"

namespace map_optimization_legacy {

void PosePriorImport::importPosesFromCSV(
    const std::string& import_filename, const vi_map::MissionId& mission_id,
    vi_map::VIMap* map) {
  CHECK_NOTNULL(map);
  CHECK(map->hasMission(mission_id)) << "Given mission not present in map!";
  // Try to load the file.
  CHECK(!import_filename.empty());
  std::ifstream import_file(import_filename, std::ios::in);
  CHECK(import_file.is_open()) << "Failed to open file " << import_filename;

  // Store everything into an Eigen matrix, corresponding to the structure in
  // the CSV/MATLAB.
  // This will set the positions to these values in the mission frame.
  // Order: time [s], x [m], y [m], z [m], q_w, q_x, q_y, q_z [active]
  constexpr size_t kNumColumns = 8;

  // Count how many lines are in the file, then rewind the iterator. Don't want
  // to resize the eigen matrix every time, since the ground truth file is
  // usually pretty huge.
  std::streampos file_beginning = import_file.tellg();
  const size_t num_lines = std::count(
      std::istreambuf_iterator<char>(import_file),
      std::istreambuf_iterator<char>(), '\n');
  import_file.seekg(file_beginning);

  Eigen::Matrix<double, Eigen::Dynamic, kNumColumns> data_matrix(
      num_lines, kNumColumns);

  std::string line;
  size_t line_index = 0;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);
    const bool comma_separated = (line.find(',') != std::string::npos);
    for (size_t i = 0; i < kNumColumns; ++i) {
      CHECK(!line_stream.eof()) << "Not enough elements in line:\n" << line;
      std::string element;

      std::getline(line_stream, element, (comma_separated ? ',' : ' '));
      CHECK(!element.empty());
      try {  // NOLINT
        data_matrix(line_index, i) = std::stod(element);
      } catch (const std::exception& exception) {  // NOLINT
        LOG(FATAL) << "Could not parse prior file: " << exception.what();
      }
    }
    ++line_index;
  }

  // Match up each vertex to closest timestamp.
  // Don't allow more than N sec deviation. If there is no match within the
  // tolerance, then don't add a pose prior to that vertex.
  constexpr static const double kMaxTimestampDeviationSec = 0.01;

  // Iterate over the given mission.
  pose_graph::VertexId vertex_id =
      map->getMission(mission_id).getRootVertexId();

  constexpr double kNormTolerance = 0.01;

  size_t matrix_index = 0;
  // Assume both vertex timestamps and CSV timestamps are monotonically
  // increasing.
  do {
    double vertex_timestamp = aslam::time::nanoSecondsToSeconds(
        map->getVertex(vertex_id)
            .getVisualNFrame()
            .getMinTimestampNanoseconds());
    // Find the first timestamp that's greater than the vertex timestamp.
    while (matrix_index < num_lines &&
           data_matrix(matrix_index, 0) < vertex_timestamp) {
      ++matrix_index;
    }
    if (data_matrix(matrix_index, 0) >= vertex_timestamp &&
        fabs(vertex_timestamp - data_matrix(matrix_index, 0)) <
            kMaxTimestampDeviationSec) {
      // Also check the previous vertex to see if maybe it's closer.
      if (matrix_index > 0 &&
          fabs(vertex_timestamp - data_matrix(matrix_index - 1, 0)) <
              fabs(vertex_timestamp - data_matrix(matrix_index, 0))) {
        --matrix_index;
      }

      // Set the vertex pose from the GT data.
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      Eigen::Quaterniond q_M_I(
          data_matrix(matrix_index, 4), data_matrix(matrix_index, 5),
          data_matrix(matrix_index, 6), data_matrix(matrix_index, 7));
      if (fabs(q_M_I.norm() - 1.0) > kNormTolerance) {
        LOG(ERROR) << "Quaternion norm outside of tolerance: " << q_M_I.norm()
                   << ", skipping pose prior.";
        continue;
      }
      q_M_I.normalize();
      vertex.set_p_M_I(data_matrix.block<1, 3>(matrix_index, 1));
      vertex.set_q_M_I(q_M_I);

      vertices_with_priors_.emplace(vertex_id);
    }
    // If we're at the end, don't fix the vertices, but give them reasonable
    // priors from the last pose.
    if (matrix_index >= num_lines - 1) {
      matrix_index = num_lines - 1;
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      Eigen::Quaterniond q_M_I(
          data_matrix(matrix_index, 4), data_matrix(matrix_index, 5),
          data_matrix(matrix_index, 6), data_matrix(matrix_index, 7));
      if (fabs(q_M_I.norm() - 1.0) > kNormTolerance) {
        LOG(ERROR) << "Quaternion norm outside of tolerance: " << q_M_I.norm()
                   << ", skipping.";
      } else {
        q_M_I.normalize();
        vertex.set_q_M_I(q_M_I);
        vertex.set_p_M_I(data_matrix.block<1, 3>(matrix_index, 1));
      }
    }
  } while (map->getNextVertex(
      vertex_id, map->getGraphTraversalEdgeType(mission_id), &vertex_id));

  LOG(INFO) << "Added pose priors to " << vertices_with_priors_.size()
            << " out of " << map->numVerticesInMission(mission_id)
            << " vertices.";
}

void PosePriorImport::runBAWithPosePriors(vi_map::VIMap* map) const {
  CHECK_NOTNULL(map);
  GraphBaOptimizer optimizer(map);
  BaOptimizationOptions options;
  options.num_iterations = 40;
  options.add_pose_prior_for_fixed_vertices = false;
  options.fix_ncamera_intrinsics = true;
  options.fix_landmark_positions = false;
  options.fix_accel_bias = false;
  options.fix_gyro_bias = false;
  options.fix_landmark_positions_of_fixed_vertices = false;
  options.remove_behind_camera_landmarks = false;
  options.include_wheel_odometry = false;
  options.include_gps = false;
  options.position_only_gps = false;
  vi_map::MissionBaseFrameIdSet fixed_baseframes;
  pose_graph::VertexIdSet velocity_prior_for_vertices;
  ceres::Solver::Summary summary;
  optimizer.visualInertialBaOptimizationWithCallback(
      fixed_baseframes, vertices_with_priors_, velocity_prior_for_vertices,
      options, nullptr, &summary);
}

}  // namespace map_optimization_legacy
