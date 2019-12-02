#ifndef ONLINE_MAP_BUILDERS_STREAM_VIO_MAP_BUILDER_H_
#define ONLINE_MAP_BUILDERS_STREAM_VIO_MAP_BUILDER_H_

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <map-resources/resource-conversion.h>
#include <posegraph/unique-id.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-mission.h>

DECLARE_bool(map_builder_save_point_clouds_as_resources);
DECLARE_bool(map_builder_save_image_as_resources);

namespace aslam {
class NCamera;
class VisualNFrame;
}  // namespace aslam
namespace vio {
class VioUpdate;
class ViNodeState;
}  // namespace vio
namespace vi_map {
class VIMap;
}

namespace online_map_builders {
class StreamVioMapBuilder {
 public:
  StreamVioMapBuilder(
      const vi_map::SensorManager& sensor_manager, vi_map::VIMap* map);

  // Deep copies the nframe.
  void apply(const vio::VioUpdate& update);
  void apply(const vio::VioUpdate& update, bool deep_copy_nframe);

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

 private:
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

  inline const vi_map::VIMap* constMap() const;

  vi_map::VIMap* const map_;
  vi_map_helpers::VIMapManipulation manipulation_;
  const vi_map::MissionId mission_id_;
  pose_graph::VertexId last_vertex_;

  aslam::Transformation T_M0_G_;
  bool is_first_baseframe_estimate_processed_;

  static constexpr size_t kKeepNMostRecentImages = 10u;
};

template <typename PointCloudType>
void StreamVioMapBuilder::attachLidarMeasurement(
    const vi_map::LidarMeasurement<PointCloudType>& lidar_measurement) {
  CHECK_NOTNULL(map_);
  CHECK(lidar_measurement.isValid())
      << "[StreamVioMapBuilder] Lidar measurement is invalid!";

  if (backend::getPointCloudSize(lidar_measurement.getPointCloud()) == 0u) {
    LOG(WARNING) << "[StreamVioMapBuilder] Received empty point cloud!";
    return;
  }

  const aslam::SensorId& lidar_sensor_id = lidar_measurement.getSensorId();

  CHECK(map_->getSensorManager().hasSensor(lidar_sensor_id))
      << "[StreamVioMapBuilder] The lidar sensor of this lidar measurement is "
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

}  // namespace online_map_builders

#endif  // ONLINE_MAP_BUILDERS_STREAM_VIO_MAP_BUILDER_H_
