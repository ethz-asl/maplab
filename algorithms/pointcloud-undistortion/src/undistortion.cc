#include "pointcloud-undistortion/undistortion.h"

#include <gflags/gflags.h>
DEFINE_int32(
    pointcloud_undistortion_ts_precision_ns, 1e5,
    "Precision up to which point timestamps are rounded");

#include <landmark-triangulation/pose-interpolator.h>

namespace pointcloud_undistortion {

bool undistortPointCloud(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const aslam::SensorId& sensor_id, const int64_t& timestamp_ns,
    resources::PointCloud* cloud) {
  if (cloud->is_undistorted) {
    return true;
  }
  if (cloud->hasNormals() || !cloud->hasTimes()) {
    return false;
  }

  constexpr float kSecondsToNanoSeconds = 1e+9;
  std::unordered_map<int64_t, std::vector<size_t>> times_to_indices;
  const int64_t kRoundValueNanoSeconds =
      static_cast<int64_t>(FLAGS_pointcloud_undistortion_ts_precision_ns / 2);
  for (size_t idx = 0u; idx < cloud->size(); ++idx) {
    if (FLAGS_pointcloud_undistortion_ts_precision_ns > 0) {
      auto& indices_at_time = times_to_indices
          [(static_cast<int64_t>(
                kSecondsToNanoSeconds * cloud->times[idx] +
                kRoundValueNanoSeconds) /
            static_cast<int64_t>(
                FLAGS_pointcloud_undistortion_ts_precision_ns)) *
           static_cast<int64_t>(FLAGS_pointcloud_undistortion_ts_precision_ns)];
      indices_at_time.push_back(idx);
    } else {
      auto& indices_at_time = times_to_indices[(static_cast<int64_t>(
        kSecondsToNanoSeconds * cloud->times[idx])];
      indices_at_time.push_back(idx);
    }
  }

  // Check if there is IMU data to interpolate the sensor poses.
  const landmark_triangulation::PoseInterpolator pose_interpolator;
  landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
  int64_t min_timestamp_ns;
  int64_t max_timestamp_ns;
  pose_interpolator.getVertexToTimeStampMap(
      map, mission_id, &vertex_to_time_map, &min_timestamp_ns,
      &max_timestamp_ns);

  // Interpolate poses for every resource.
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> base_timestamp(1);
  base_timestamp[0] = timestamp_ns;
  aslam::TransformationVector T_base_vector;
  const bool based_succeeded = pose_interpolator.getPosesAtTime(
      map, mission_id, base_timestamp, &T_base_vector);
  const aslam::Transformation& T_M_B_t0 = T_base_vector[0];
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(
      times_to_indices.size());
  auto it = times_to_indices.begin();
  for (size_t idx = 0u; it != times_to_indices.end(); ++it, ++idx) {
    resource_timestamps[idx] = timestamp_ns + it->first;
    if (resource_timestamps[idx] > max_timestamp_ns ||
        resource_timestamps[idx] < min_timestamp_ns) {
      LOG(WARNING) << "The point cloud contains a point at "
                   << aslam::time::timeNanosecondsToString(
                          resource_timestamps[idx])
                   << " that is not within the time range of the pose-graph ["
                   << aslam::time::timeNanosecondsToString(min_timestamp_ns)
                   << ", "
                   << aslam::time::timeNanosecondsToString(max_timestamp_ns)
                   << "]";
      return false;
    }
  }
  aslam::TransformationVector T_M_B_vector;
  const bool interpolation_succeeded = pose_interpolator.getPosesAtTime(
      map, mission_id, resource_timestamps, &T_M_B_vector);
  if (!interpolation_succeeded) {
    return false;
  }

  const vi_map::SensorManager& sensor_manager = map.getSensorManager();
  const aslam::Transformation& T_B_S =
      sensor_manager.getSensor_T_B_S(sensor_id);

  it = times_to_indices.begin();
  for (size_t time_idx = 0u; it != times_to_indices.end(); ++it, ++time_idx) {
    const aslam::Transformation& T_M_B_t_point = T_M_B_vector[time_idx];
    for (const size_t& point_idx : it->second) {
      const size_t idx = point_idx * 3;
      const Eigen::Vector3d point(
          cloud->xyz[idx], cloud->xyz[idx + 1u], cloud->xyz[idx + 2u]);
      const aslam::Transformation T_S_S_point =
          (T_M_B_t0 * T_B_S).inverse() * T_M_B_t_point * T_B_S;
      const Eigen::Vector3f& transformed_point =
          (T_S_S_point * point).cast<float>();
      cloud->xyz[idx] = transformed_point.x();
      cloud->xyz[idx + 1u] = transformed_point.y();
      cloud->xyz[idx + 2u] = transformed_point.z();
    }
  }

  cloud->is_undistorted = true;
  return true;
}

}  //  namespace pointcloud_undistortion
