#ifndef LANDMARK_TRIANGULATION_POSE_INTERPOLATOR_H_
#define LANDMARK_TRIANGULATION_POSE_INTERPOLATOR_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/temporal-buffer.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace landmark_triangulation {

struct VertexInformation {
  int64_t timestamp_ns;
  int64_t timestamp_ns_end;
  pose_graph::VertexId vertex_id;
  pose_graph::EdgeId outgoing_imu_edge_id;
};

struct IMUMeasurement {
  int64_t timestamp;                            // nanoseconds.
  Eigen::Matrix<double, 6, 1> imu_measurement;  // 3x1 accel, 3x1 gyro.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct StateLinearizationPoint {
  int64_t timestamp;  // nanoseconds.
  Eigen::Quaterniond q_M_I;
  Eigen::Matrix<double, 3, 1> p_M_I;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PoseInterpolator {
 public:
  // Returns interpolated poses for the mission specified by mission_id.
  // One pose is calculated and returned for each timestamp included in
  // imu_timestamps.  Timestamps do not need to be sorted, but must lie within
  // the minimum and maximum timestamp of the mission's IMU measurements.
  // Extrapolating outside this range is not supported.
  void getPosesAtTime(
      const vi_map::VIMap& map, vi_map::MissionId mission_id,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      aslam::TransformationVector* poses) const;

  // Returns interpolated poses and their associated timestamps across an entire
  // mission specified by emission_id.  Timestamps begin at the earliest IMU
  // measurement, then continue every timestep_seconds until the last possible
  // timestep in the mission is reached.
  void getPosesEveryNSeconds(
      const vi_map::VIMap& vi_map, const vi_map::MissionId mission_id,
      const double timestep_seconds,
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* pose_times,
      aslam::TransformationVector* poses) const;

  // Returns a map from vertex id to timestamp based on the IMU measurements.
  // Returns an empty map if there are no IMU measurements.
  void getVertexToTimeStampMap(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
      std::unordered_map<pose_graph::VertexId, int64_t>* vertex_to_time_map)
      const;

 private:
  typedef std::pair<int64_t, StateLinearizationPoint> state_buffer_value_type;
  typedef common::TemporalBuffer<
      StateLinearizationPoint,
      Eigen::aligned_allocator<state_buffer_value_type> >
      StateBuffer;

  // Determine the earliest and latest IMU measurements across an entire
  // mission.
  void getMissionTimeRange(
      const vi_map::VIMap& vi_map, const vi_map::MissionId mission_id,
      int64_t* mission_start_ns_ptr, int64_t* mission_end_ns_ptr) const;

  void getImuDataInRange(
      const vi_map::VIMap& map, pose_graph::EdgeId imu_edge_id,
      const std::vector<int64_t>& sorted_timestamps, int64_t range_time_start,
      int64_t range_time_end,
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* imu_timestamps,
      Eigen::Matrix<double, 6, Eigen::Dynamic>* imu_data) const;

  void buildVertexToTimeList(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
      std::vector<VertexInformation>* vertices_and_time) const;

  void computeRequestedPosesInRange(
      const vi_map::VIMap& map, const vi_map::VIMission& mission,
      const pose_graph::VertexId& vertex_begin_id,
      const pose_graph::EdgeId& imu_edge_id,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
      StateBuffer* state_buffer) const;

  void getImuTimeStampsInRange(
      const std::vector<int64_t>& timestamps, int64_t range_time_start,
      int64_t range_time_end) const;

  void buildListOfAllRequiredIMUMeasurements(
      const vi_map::VIMap& map, const std::vector<int64_t>& timestamps,
      const pose_graph::EdgeId& imu_edge_id, int start_index, int end_index,
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* imu_timestamps,
      Eigen::Matrix<double, 6, Eigen::Dynamic>* imu_data) const;
};
}  // namespace landmark_triangulation
#endif  // LANDMARK_TRIANGULATION_POSE_INTERPOLATOR_H_
