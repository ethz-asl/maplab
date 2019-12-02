#ifndef ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_
#define ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/resource-conversion.h>
#include <posegraph/unique-id.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-mission.h>
#include <vio-common/vio-types.h>

DECLARE_bool(map_builder_save_point_clouds_as_resources);
DECLARE_bool(map_builder_save_point_cloud_maps_as_resources);
DECLARE_bool(map_builder_save_image_as_resources);

namespace aslam {
class NCamera;
class VisualNFrame;
}  // namespace aslam
namespace vio {
class MapUpdate;
class ViNodeState;
}  // namespace vio
namespace vi_map {
class VIMap;
}

namespace online_map_builders {
class StreamMapBuilder {
 public:
  StreamMapBuilder(
      const vi_map::SensorManager& sensor_manager, vi_map::VIMap* map);

  // Deep copies the nframe.
  void apply(const vio::MapUpdate& update);
  void apply(const vio::MapUpdate& update, bool deep_copy_nframe);

  vi_map::MissionId getMissionId() const {
    return mission_id_;
  }

  pose_graph::VertexId getRootVertexId() const;
  pose_graph::VertexId getLastVertexId() const;

  void removeAllVerticesAfterVertexId(
      const pose_graph::VertexId& vertiex_id_from,
      pose_graph::VertexIdList* removed_vertex_ids);

  bool checkConsistency() const;

  template <typename PointCloudType>
  void attachLidarMeasurement(
      const vi_map::LidarMeasurement<PointCloudType>& lidar_measurement);

  void bufferAbsolute6DoFConstraint(
      const vi_map::Absolute6DoFMeasurement::Ptr& absolute_6dof_constraint);

  void bufferLoopClosureConstraint(
      const vi_map::LoopClosureMeasurement::ConstPtr& loop_closure_constraint);

  void bufferWheelOdometryConstraint(
      const vi_map::WheelOdometryMeasurement::ConstPtr&
          wheel_odometry_constraint);

  template <typename PointCloudType>
  void attachPointCloudMap(
      const vi_map::PointCloudMapSensorMeasurement<PointCloudType>&
          pointcloud_map_measurement);

  void updateMapDependentData();

 private:
  inline void getMissionTimeLimitsNs(
      int64_t* start_time_ns, int64_t* end_time_ns) const {
    CHECK_NOTNULL(start_time_ns);
    CHECK_NOTNULL(end_time_ns);
    CHECK_NOTNULL(map_);
    CHECK(mission_id_.isValid());

    pose_graph::VertexIdList vertex_ids;
    map_->getAllVertexIdsInMissionAlongGraph(mission_id_, &vertex_ids);
    *start_time_ns =
        map_->getVertex(vertex_ids.front()).getMinTimestampNanoseconds();
    *end_time_ns =
        map_->getVertex(vertex_ids.back()).getMinTimestampNanoseconds();

    CHECK_GE(*end_time_ns, *start_time_ns);
  }

  void notifyBuffers();

  void notifyAbsolute6DoFConstraintBuffer();
  void notifyLoopClosureConstraintBuffer();
  void notifyWheelOdometryConstraintBuffer();

  void addRootViwlsVertex(
      const std::shared_ptr<aslam::VisualNFrame>& nframe,
      const vio::ViNodeState& vinode_state);

  void addViwlsVertexAndEdge(
      const std::shared_ptr<aslam::VisualNFrame>& nframe,
      const vio::ViNodeState& vinode_state, const aslam::Transformation& T_G_M,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data);

  pose_graph::VertexId addViwlsVertex(
      const std::shared_ptr<aslam::VisualNFrame>& nframe,
      const vio::ViNodeState& vinode_state, const aslam::Transformation& T_G_M);

  void addImuEdge(
      pose_graph::VertexId target_vertex_id,
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements);

  void addWheelOdometryEdge(
      const pose_graph::VertexId& source_vertex_id,
      const pose_graph::VertexId& target_vertex_id,
      const aslam::Transformation& wheel_transformation);

  inline const vi_map::VIMap* constMap() const;

  vi_map::VIMap* const map_;
  landmark_triangulation::PoseInterpolator pose_interpolator_;

  // TODO(mfehr): The thread safety shouldn't be necessary but needs to be
  // checked.
  std::atomic<int64_t> oldest_vertex_timestamp_ns_;
  std::atomic<int64_t> newest_vertex_timestamp_ns_;

  vi_map_helpers::VIMapQueries queries_;
  vi_map_helpers::VIMapManipulation manipulation_;
  const vi_map::MissionId mission_id_;
  pose_graph::VertexId last_vertex_;

  typedef std::unordered_map<
      std::pair<pose_graph::VertexId, pose_graph::VertexId>, pose_graph::EdgeId>
      LoopClosureEdgeLookup;

  LoopClosureEdgeLookup attached_loop_closures_lookup_;
  vi_map::LoopClosureTemporalMap loop_closure_measurement_buffer_;

  common::TemporalBuffer<vi_map::WheelOdometryMeasurement::ConstPtr>
      wheel_odometry_measurement_temporal_buffer_;
  // Id of the current vertex being processed by the wheel odometry logic.
  pose_graph::VertexId vertex_processing_wheel_odometry_id_;
  // Id of the last vertex that was deemed processed by the wheel odometry
  // logic (either an edge was added or it was determined that no edge can ever
  // be added).
  pose_graph::VertexId last_vertex_done_wheel_odometry_id_;

  // Transform between wheel odometry origin to previously processed vertex
  aslam::Transformation T_Ow_Btm1_;
  bool found_wheel_odometry_origin_;

  common::TemporalBuffer<vi_map::Absolute6DoFMeasurement::Ptr>
      absolute_6dof_measurment_buffer_;

  aslam::Transformation T_M0_G_;
  bool is_first_baseframe_estimate_processed_;

  static constexpr size_t kKeepNMostRecentImages = 10u;
};

template <typename PointCloudType>
void StreamMapBuilder::attachLidarMeasurement(
    const vi_map::LidarMeasurement<PointCloudType>& lidar_measurement) {
  CHECK_NOTNULL(map_);
  CHECK(lidar_measurement.isValid())
      << "[StreamMapBuilder] Lidar measurement is invalid!";

  if (backend::getPointCloudSize(lidar_measurement.getPointCloud()) == 0u) {
    LOG(WARNING) << "[StreamMapBuilder] Received empty point cloud!";
    return;
  }

  const aslam::SensorId& lidar_sensor_id = lidar_measurement.getSensorId();

  CHECK(map_->getSensorManager().hasSensor(lidar_sensor_id))
      << "[StreamMapBuilder] The lidar sensor of this lidar measurement is "
         "not yet present in the map's sensor manager!";

  resources::PointCloud point_cloud;
  backend::convertPointCloudType<PointCloudType, resources::PointCloud>(
      lidar_measurement.getPointCloud(), &point_cloud);

  backend::ResourceType point_cloud_type =
      backend::getResourceTypeForPointCloud(point_cloud);
  vi_map::VIMission& mission = map_->getMission(mission_id_);
  map_->addSensorResource(
      point_cloud_type, lidar_sensor_id,
      lidar_measurement.getTimestampNanoseconds(), point_cloud, &mission);
}

template <typename PointCloudType>
void StreamMapBuilder::attachPointCloudMap(
    const vi_map::PointCloudMapSensorMeasurement<PointCloudType>&
        pointcloud_map_measurement) {
  CHECK_NOTNULL(map_);
  CHECK(pointcloud_map_measurement.isValid())
      << "[StreamMapBuilder] Point cloud map measurement is invalid!";

  if (backend::getPointCloudSize(pointcloud_map_measurement.getPointCloud()) ==
      0u) {
    LOG(WARNING) << "[StreamMapBuilder] Received empty point cloud map!";
    return;
  }

  const aslam::SensorId& point_cloud_map_sensor_id =
      pointcloud_map_measurement.getSensorId();

  CHECK(map_->getSensorManager().hasSensor(point_cloud_map_sensor_id))
      << "[StreamMapBuilder] The point cloud map sensor of this point cloud "
         "map measurement is "
         "not yet present in the map's sensor manager!";

  resources::PointCloud point_cloud;
  backend::convertPointCloudType<PointCloudType, resources::PointCloud>(
      pointcloud_map_measurement.getPointCloud(), &point_cloud);

  backend::ResourceType point_cloud_type =
      backend::getResourceTypeForPointCloud(point_cloud);
  vi_map::VIMission& mission = map_->getMission(mission_id_);

  if (map_->hasSensorResource(
          mission, point_cloud_type, point_cloud_map_sensor_id,
          pointcloud_map_measurement.getTimestampNanoseconds())) {
    LOG(WARNING)
        << "[StreamMapBuilder] There is already a point cloud map resource at "
        << pointcloud_map_measurement.getTimestampNanoseconds()
        << " replacing...";
    map_->deleteSensorResource<resources::PointCloud>(
        point_cloud_type, point_cloud_map_sensor_id,
        pointcloud_map_measurement.getTimestampNanoseconds(),
        false /*keep sensor resurce file*/, &mission);
  }
  map_->addSensorResource(
      point_cloud_type, point_cloud_map_sensor_id,
      pointcloud_map_measurement.getTimestampNanoseconds(), point_cloud,
      &mission);
}

}  // namespace online_map_builders

#endif  // ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_
