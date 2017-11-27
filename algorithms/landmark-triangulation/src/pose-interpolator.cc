#include "landmark-triangulation/pose-interpolator.h"

#include <aslam/common/time.h>
#include <glog/logging.h>
#include <imu-integrator/imu-integrator.h>
#include <maplab-common/macros.h>

namespace landmark_triangulation {
void PoseInterpolator::buildListOfAllRequiredIMUMeasurements(
    const vi_map::VIMap& map, const std::vector<int64_t>& timestamps,
    const pose_graph::EdgeId& imu_edge_id, int start_index, int end_index,
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* imu_timestamps,
    Eigen::Matrix<double, 6, Eigen::Dynamic>* imu_data) const {
  CHECK_NOTNULL(imu_timestamps);
  CHECK_NOTNULL(imu_data);
  CHECK_LT(start_index, static_cast<int>(timestamps.size()));
  CHECK_LT(end_index, static_cast<int>(timestamps.size()));
  CHECK_GE(start_index, 0);
  CHECK_GE(end_index, 0);

  // First add all imu measurements from this vertex to the buffer.
  typedef std::pair<int64_t, IMUMeasurement> buffer_value_type;
  using common::TemporalBuffer;
  typedef TemporalBuffer<IMUMeasurement,
                         Eigen::aligned_allocator<buffer_value_type> >
      ImuMeasurementBuffer;
  ImuMeasurementBuffer imu_buffer;
  {
    const vi_map::ViwlsEdge& imu_edge =
        map.getEdgeAs<vi_map::ViwlsEdge>(imu_edge_id);
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps =
        imu_edge.getImuTimestamps();
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data =
        imu_edge.getImuData();  // 3x1 accel, 3x1 gyro.
    CHECK_EQ(imu_timestamps.cols(), imu_data.cols());
    for (int i = 0; i < imu_data.cols(); ++i) {
      IMUMeasurement measurement;
      measurement.imu_measurement = imu_data.col(i);
      measurement.timestamp = imu_timestamps(0, i);
      imu_buffer.addValue(measurement.timestamp, measurement);
    }
  }

  // Now compute the interpolated values for the requested items in case
  // we don't have IMU measurements at the particular time.
  for (int i = start_index; i <= end_index; ++i) {
    int64_t requested_time = timestamps[i];
    IMUMeasurement measurement;
    bool have_value_at_time =
        imu_buffer.getValueAtTime(requested_time, &measurement);
    if (have_value_at_time) {
      // No need for interpolation of IMU measurements.
      continue;
    }
    // Get the IMU measurement closest before and after the requested
    // time.
    IMUMeasurement measurement_before;
    int64_t timestamp_before = 0;
    CHECK(
        imu_buffer.getValueAtOrBeforeTime(
            requested_time, &timestamp_before, &measurement_before));
    IMUMeasurement measurement_after;
    int64_t timestamp_after = 0;
    CHECK(
        imu_buffer.getValueAtOrAfterTime(
            requested_time, &timestamp_after, &measurement_after));

    CHECK_NE(timestamp_after, timestamp_before);

    // Interpolate the IMU measurement.
    IMUMeasurement interpolated_measurement;
    interpolated_measurement.timestamp = requested_time;
    double alpha = static_cast<double>(requested_time - timestamp_before) /
                   static_cast<double>(timestamp_after - timestamp_before);
    CHECK_GT(alpha, 0.0);
    CHECK_LT(alpha, 1.0);
    interpolated_measurement.imu_measurement =
        (1 - alpha) * measurement_before.imu_measurement +
        alpha * measurement_after.imu_measurement;
    imu_buffer.addValue(
        interpolated_measurement.timestamp, interpolated_measurement);
  }

  imu_buffer.lockContainer();
  const ImuMeasurementBuffer::BufferType& buffered_values =
      imu_buffer.buffered_values();

  imu_timestamps->resize(Eigen::NoChange, buffered_values.size());
  imu_data->resize(Eigen::NoChange, buffered_values.size());
  int index = 0;
  for (const buffer_value_type& value : buffered_values) {
    (*imu_timestamps)(0, index) = value.second.timestamp;
    imu_data->col(index) = value.second.imu_measurement;
    ++index;
  }
  imu_buffer.unlockContainer();
}

void PoseInterpolator::computeRequestedPosesInRange(
    const vi_map::VIMap& map, const vi_map::VIMission& mission,
    const pose_graph::VertexId& vertex_begin_id,
    const pose_graph::EdgeId& /*imu_edge_id*/,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
    StateBuffer* state_buffer) const {
  CHECK_NOTNULL(state_buffer);
  CHECK_EQ(imu_timestamps.cols(), imu_data.cols());
  if (imu_data.cols() == 0) {
    return;
  }

  using imu_integrator::ImuIntegratorRK4;
  const vi_map::MissionId& mission_id = mission.id();
  CHECK(mission_id.isValid());
  const vi_map::Imu& imu_sensor =
      map.getSensorManager().getSensorForMission<vi_map::Imu>(mission_id);
  const vi_map::ImuSigmas& imu_sigmas = imu_sensor.getImuSigmas();

  ImuIntegratorRK4 integrator(
      imu_sigmas.gyro_noise_density,
      imu_sigmas.gyro_bias_random_walk_noise_density,
      imu_sigmas.acc_noise_density,
      imu_sigmas.acc_bias_random_walk_noise_density,
      imu_sensor.getGravityMagnitudeMps2());

  using imu_integrator::kAccelBiasBlockSize;
  using imu_integrator::kAccelReadingOffset;
  using imu_integrator::kErrorStateSize;
  using imu_integrator::kGyroBiasBlockSize;
  using imu_integrator::kGyroReadingOffset;
  using imu_integrator::kImuReadingSize;
  using imu_integrator::kNanoSecondsToSeconds;
  using imu_integrator::kPositionBlockSize;
  using imu_integrator::kStateAccelBiasOffset;
  using imu_integrator::kStateGyroBiasOffset;
  using imu_integrator::kStateOrientationBlockSize;
  using imu_integrator::kStatePositionOffset;
  using imu_integrator::kStateSize;

  Eigen::Matrix<double, 2 * kImuReadingSize, 1> debiased_imu_readings;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize> phi;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize> new_phi_accum;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize> Q;
  Eigen::Matrix<double, kErrorStateSize, kErrorStateSize> new_Q_accum;
  Eigen::Matrix<double, kStateSize, 1> current_state;
  Eigen::Matrix<double, kStateSize, 1> next_state;

  const vi_map::Vertex& vertex_from = map.getVertex(vertex_begin_id);
  const aslam::Transformation T_M_I = vertex_from.get_T_M_I();

  // Active to passive and direction switch, so no inversion.
  const Eigen::Matrix<double, 4, 1>& q_I_M_from =
      T_M_I.getRotation().toImplementation().coeffs();
  const Eigen::Matrix<double, 3, 1>& p_M_I_from = T_M_I.getPosition();
  const Eigen::Matrix<double, 3, 1>& v_M_I_from = vertex_from.get_v_M();
  const Eigen::Matrix<double, 3, 1>& b_g_from = vertex_from.getGyroBias();
  const Eigen::Matrix<double, 3, 1>& b_a_from = vertex_from.getAccelBias();

  current_state << q_I_M_from, b_g_from, v_M_I_from, b_a_from, p_M_I_from;

  // Store the value where we start integration.
  StateLinearizationPoint state_linearization_point_begin;
  state_linearization_point_begin.timestamp = imu_timestamps(0, 0);
  state_linearization_point_begin.q_M_I.coeffs() =
      current_state.head<kStateOrientationBlockSize>();
  state_linearization_point_begin.p_M_I =
      current_state.segment<kPositionBlockSize>(kStatePositionOffset);
  constexpr bool kEmitWarningIfValuesOverwritten = false;
  state_buffer->addValue(
      state_linearization_point_begin.timestamp,
      state_linearization_point_begin, kEmitWarningIfValuesOverwritten);

  // Now compute all the integrated values.
  for (int i = 0; i < imu_data.cols() - 1; ++i) {
    CHECK_GE(imu_timestamps(0, i + 1), imu_timestamps(0, i))
        << "IMU measurements not properly ordered";

    Eigen::Vector3d current_gyro_bias =
        current_state.segment<kGyroBiasBlockSize>(kStateGyroBiasOffset);
    Eigen::Vector3d current_accel_bias =
        current_state.segment<kAccelBiasBlockSize>(kStateAccelBiasOffset);

    debiased_imu_readings << imu_data.col(i).segment<3>(kAccelReadingOffset) -
                                 current_accel_bias,
        imu_data.col(i).segment<3>(kGyroReadingOffset) - current_gyro_bias,
        imu_data.col(i + 1).segment<3>(kAccelReadingOffset) -
            current_accel_bias,
        imu_data.col(i + 1).segment<3>(kGyroReadingOffset) - current_gyro_bias;

    double delta_time_seconds =
        (imu_timestamps(0, i + 1) - imu_timestamps(0, i)) *
        kNanoSecondsToSeconds;
    integrator.integrate(
        current_state, debiased_imu_readings, delta_time_seconds, &next_state,
        &phi, &Q);

    StateLinearizationPoint state_linearization_point;
    state_linearization_point.timestamp = imu_timestamps(0, i + 1);
    state_linearization_point.q_M_I.coeffs() =
        next_state.head<kStateOrientationBlockSize>();
    state_linearization_point.p_M_I =
        next_state.segment<kPositionBlockSize>(kStatePositionOffset);
    state_buffer->addValue(
        state_linearization_point.timestamp, state_linearization_point);

    current_state = next_state;
  }
}

void PoseInterpolator::getVertexToTimeStampMap(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    std::unordered_map<pose_graph::VertexId, int64_t>* vertex_to_time_map)
    const {
  CHECK_NOTNULL(vertex_to_time_map)->clear();
  // Get the outgoing edge of the vertex and its IMU data.
  pose_graph::VertexIdList all_mission_vertices;
  map.getAllVertexIdsInMissionAlongGraph(mission_id, &all_mission_vertices);

  vertex_to_time_map->reserve(all_mission_vertices.size());
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
    if (imu_timestamps.cols() > 0) {
      (*vertex_to_time_map)[vertex_id] = imu_timestamps(0, 0);
    }
  }
}

void PoseInterpolator::buildVertexToTimeList(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    std::vector<VertexInformation>* vertices_and_time) const {
  CHECK_NOTNULL(vertices_and_time)->clear();
  // Get the outgoing edge of the vertex and its IMU data.
  pose_graph::VertexIdList all_mission_vertices;
  map.getAllVertexIdsInMissionAlongGraph(mission_id, &all_mission_vertices);

  vertices_and_time->reserve(all_mission_vertices.size());
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
    if (imu_timestamps.cols() > 0) {
      VertexInformation vertex_information;
      vertex_information.timestamp_ns = imu_timestamps(0, 0);
      vertex_information.timestamp_ns_end =
          imu_timestamps(0, imu_timestamps.cols() - 1);
      vertex_information.vertex_id = vertex_id;
      vertex_information.outgoing_imu_edge_id = outgoing_imu_edge_id;
      vertices_and_time->push_back(vertex_information);
    }
  }
}

void PoseInterpolator::getImuDataInRange(
    const vi_map::VIMap& map, pose_graph::EdgeId imu_edge_id,
    const std::vector<int64_t>& sorted_timestamps, int64_t range_time_start,
    int64_t range_time_end,
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* imu_timestamps,
    Eigen::Matrix<double, 6, Eigen::Dynamic>* imu_data) const {
  CHECK_NOTNULL(imu_timestamps);
  CHECK_NOTNULL(imu_data);

  CHECK_LE(range_time_start, range_time_end);

  typedef std::vector<int64_t>::const_iterator TimestampIterator;

  // Search if we have timestamp requests in this range.
  TimestampIterator it_range_start = std::lower_bound(
      sorted_timestamps.begin(), sorted_timestamps.end(), range_time_start,
      std::less<int64_t>());
  TimestampIterator it_range_end = std::lower_bound(
      sorted_timestamps.begin(), sorted_timestamps.end(), range_time_end,
      std::less<int64_t>());

  int start_index = -1;
  int end_index = -1;
  if (it_range_start == it_range_end &&
      it_range_start != sorted_timestamps.end()) {
    // No request in range. Range is enclosed in the interval between two
    // consecutive timestamps.
  } else if (
      it_range_start == sorted_timestamps.begin() &&
      it_range_end != sorted_timestamps.end()) {
    // Some requests in range.
    start_index = 0;
    --it_range_end;
    end_index = std::distance(sorted_timestamps.cbegin(), it_range_end);
  } else if (
      it_range_end == sorted_timestamps.end() &&
      it_range_start != sorted_timestamps.end()) {
    // Some requests in range.
    start_index = std::distance(sorted_timestamps.cbegin(), it_range_start);
    end_index = sorted_timestamps.size() - 1;
  } else if (
      it_range_end != sorted_timestamps.end() &&
      it_range_start != sorted_timestamps.end()) {
    start_index = std::distance(sorted_timestamps.cbegin(), it_range_start);
    --it_range_end;
    end_index = std::distance(sorted_timestamps.cbegin(), it_range_end);
  }

  // Do we have pose requests in this range?
  if (start_index != -1 && end_index != -1) {
    CHECK_GE(start_index, 0);
    CHECK_GE(end_index, 0);
    CHECK_LT(start_index, static_cast<int>(sorted_timestamps.size()));
    CHECK_LT(end_index, static_cast<int>(sorted_timestamps.size()));

    CHECK_GE(sorted_timestamps[start_index], range_time_start)
        << start_index << "<->" << end_index;
    CHECK_LE(sorted_timestamps[end_index], range_time_end)
        << start_index << "<->" << end_index;

    buildListOfAllRequiredIMUMeasurements(
        map, sorted_timestamps, imu_edge_id, start_index, end_index,
        imu_timestamps, imu_data);
  } else {
    // Both indices should be set to -1.
    CHECK_EQ(start_index, end_index);
  }
}

void PoseInterpolator::getPosesAtTime(
    const vi_map::VIMap& map, vi_map::MissionId mission_id,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& pose_timestamps,
    aslam::TransformationVector* poses_M_I) const {
  CHECK_NOTNULL(poses_M_I)->clear();
  CHECK_GT(pose_timestamps.rows(), 0);

  CHECK(
      map.getGraphTraversalEdgeType(mission_id) ==
      pose_graph::Edge::EdgeType::kViwls);

  // Remember the initial ordering of the timestamps before we sort them.
  std::vector<int64_t> timestamps;
  timestamps.reserve(pose_timestamps.rows());
  for (int i = 0; i < pose_timestamps.size(); ++i) {
    timestamps.emplace_back(pose_timestamps(0, i));
  }

  std::sort(timestamps.begin(), timestamps.end(), std::less<int64_t>());

  // Build up a list of vertex-information and the timestamps of the first imu
  // measurement on a vertex' outgoing IMU edge.
  std::vector<VertexInformation> vertices_and_time;
  buildVertexToTimeList(map, mission_id, &vertices_and_time);
  CHECK_GT(vertices_and_time.size(), 1u)
      << "The Viwls edges of mission " << mission_id
      << " include none at all or only a single IMU "
      << "measurement. Interpolation is not possible!";

  int64_t smallest_time = vertices_and_time.front().timestamp_ns;
  int64_t largest_time = vertices_and_time.back().timestamp_ns_end;
  CHECK_GE(timestamps.front(), smallest_time)
      << "Requested sample out of bounds! First available time is "
      << smallest_time << " but " << timestamps.front() << " was requested.";
  CHECK_LE(timestamps.back(), largest_time)
      << "Requested sample out of bounds! Last available time is "
      << largest_time << " but " << timestamps.back() << " was requested.";
  VLOGF(4) << "Interpolation range is valid: (" << timestamps.front()
           << " >= " << smallest_time << ") and (" << timestamps.back()
           << " <= " << largest_time << ")";

  std::vector<VertexInformation>::const_iterator vertex_it =
      vertices_and_time.begin();

  const vi_map::VIMission& mission = map.getMission(mission_id);

  StateBuffer state_buffer;

  while (vertex_it != vertices_and_time.end()) {
    int64_t range_time_start = vertex_it->timestamp_ns;
    int64_t range_time_end = vertex_it->timestamp_ns_end;

    const pose_graph::EdgeId& imu_edge_id = vertex_it->outgoing_imu_edge_id;
    const pose_graph::VertexId& vertex_id = vertex_it->vertex_id;

    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data;

    getImuDataInRange(
        map, imu_edge_id, timestamps, range_time_start, range_time_end,
        &imu_timestamps, &imu_data);

    computeRequestedPosesInRange(
        map, mission, vertex_id, imu_edge_id, imu_timestamps, imu_data,
        &state_buffer);

    ++vertex_it;
  }

  // Copy the interpolated data to the output buffer.
  poses_M_I->clear();
  poses_M_I->reserve(pose_timestamps.cols());
  for (int i = 0; i < pose_timestamps.cols(); ++i) {
    StateLinearizationPoint state_linearization_point;
    const bool buffer_has_state = state_buffer.getValueAtTime(
        pose_timestamps(0, i), &state_linearization_point);
    CHECK(buffer_has_state) << ": No value in state_buffer at time: "
                            << pose_timestamps(0, i);
    poses_M_I->emplace_back(
        state_linearization_point.q_M_I, state_linearization_point.p_M_I);
  }
}

void PoseInterpolator::getMissionTimeRange(
    const vi_map::VIMap& vi_map, const vi_map::MissionId mission_id,
    int64_t* mission_start_ns_ptr, int64_t* mission_end_ns_ptr) const {
  *CHECK_NOTNULL(mission_start_ns_ptr) = -1;
  *CHECK_NOTNULL(mission_end_ns_ptr) = -1;
  pose_graph::VertexIdList vertex_ids;
  pose_graph::EdgeIdSet edge_ids;
  vi_map.getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids);
  CHECK(!vertex_ids.empty());

  // Interpolation is based on IMU measurements, so select the first IMU
  // timestamp along the viwls edge leaving the root vertex.
  const vi_map::Vertex& root_vertex = vi_map.getVertex(vertex_ids.front());
  VLOGF(2) << "Earliest timestamp on root vertex: "
           << root_vertex.getVisualNFrame().getMinTimestampNanoseconds();
  root_vertex.getOutgoingEdges(&edge_ids);
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    if (vi_map.getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kViwls) {
      *mission_start_ns_ptr =
          vi_map.getEdgeAs<vi_map::ViwlsEdge>(edge_id).getImuTimestamps()(0, 0);
      break;
    }
  }
  CHECK_GE(*mission_start_ns_ptr, 0)
      << "Did not find timestamp on viwls edge leaving root vertex.";
  VLOGF(2) << "First IMU timestamp after root: " << *mission_start_ns_ptr;

  // Likewise, use the last IMU measurement before the final vertex.
  const vi_map::Vertex& last_vertex = vi_map.getVertex(vertex_ids.back());
  VLOGF(2) << "Latest timestamp on final vertex: "
           << last_vertex.getVisualNFrame().getMaxTimestampNanoseconds();
  last_vertex.getIncomingEdges(&edge_ids);
  for (const pose_graph::EdgeId& edge_id : edge_ids) {
    if (vi_map.getEdgeType(edge_id) == pose_graph::Edge::EdgeType::kViwls) {
      *mission_end_ns_ptr = vi_map.getEdgeAs<vi_map::ViwlsEdge>(edge_id)
                                .getImuTimestamps()
                                .topRightCorner<1, 1>()(0, 0);
      break;
    }
  }
  CHECK_GE(*mission_end_ns_ptr, 0)
      << "Did not find timestamp on viwls edge entering final vertex.";
  CHECK_LT(*mission_start_ns_ptr, *mission_end_ns_ptr);
}

void PoseInterpolator::getPosesEveryNSeconds(
    const vi_map::VIMap& vi_map, const vi_map::MissionId mission_id,
    const double timestep_seconds,
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* pose_times,
    aslam::TransformationVector* poses) const {
  CHECK_GT(timestep_seconds, 0) << "Interpolation timestep must be positive.";
  CHECK_NOTNULL(pose_times);
  CHECK_NOTNULL(poses);
  int64_t mission_start_ns = -1;
  int64_t mission_end_ns = -1;
  getMissionTimeRange(vi_map, mission_id, &mission_start_ns, &mission_end_ns);

  const int64_t timestep_ns =
      aslam::time::secondsToNanoSeconds(timestep_seconds);
  CHECK_GT(timestep_ns, 0);
  const int64_t pose_count =
      ((mission_end_ns - mission_start_ns) / timestep_ns) + 1;
  VLOGF(1) << "Interpolating between " << mission_start_ns << " and "
           << mission_end_ns;
  VLOGF(1) << "Interpolating " << pose_count << " poses, last will be "
           << mission_start_ns + (pose_count - 1) * timestep_ns;

  pose_times->resize(Eigen::NoChange, pose_count);
  for (int64_t i = 0; i < pose_count; ++i) {
    (*pose_times)(0, i) = mission_start_ns + i * timestep_ns;
  }
  CHECK_LE((*pose_times)(0, pose_count - 1), mission_end_ns);
  return getPosesAtTime(vi_map, mission_id, *pose_times, poses);
}

}  // namespace landmark_triangulation
