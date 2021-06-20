#ifndef ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_
#define ONLINE_MAP_BUILDERS_STREAM_MAP_BUILDER_H_

#include <Eigen/Dense>
#include <algorithm>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/resource-conversion.h>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <posegraph/unique-id.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <unordered_map>
#include <utility>
#include <vector>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-mission.h>
#include <vio-common/vio-types.h>

DECLARE_bool(map_builder_save_point_clouds_as_resources);
DECLARE_bool(map_builder_save_point_cloud_maps_as_resources);
DECLARE_bool(
    map_builder_save_point_clouds_as_range_image_including_intensity_image);
DECLARE_bool(map_builder_save_image_as_resources);
DECLARE_bool(map_builder_visualize_lidar_depth_maps_in_ocv_window);

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

  void bufferExternalFeaturesMeasurement(
      const vi_map::ExternalFeaturesMeasurement::ConstPtr&
          external_features_measurement);

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
  void notifyExternalFeaturesMeasurementBuffer();

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
  // TODO(smauq): find more elegant solution than this by modifying the tracker
  // code in aslam to deal with the multiple feature types. Also not dealing
  // nicely with submapping at the moment!!!
  std::atomic<int64_t> second_newest_vertex_timestamp_ns_;

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
  // Flag for the current vertex being processed to determine if we are done
  // with it (either an edge was added or it was determined that no edge can
  // ever be added).
  bool done_current_vertex_wheel_odometry_;

  // Transform between wheel odometry origin and the last vertex for which
  // we added wheel odometry
  aslam::Transformation T_Ow_Btm1_;
  // Id of last vertex we added a wheel odometry edge to
  pose_graph::VertexId Btm1_vertex_id_;
  // Convinience bool flag to check if we have initialized the relative wheel
  // odometry calculation
  bool found_wheel_odometry_origin_;

  common::TemporalBuffer<vi_map::Absolute6DoFMeasurement::Ptr>
      absolute_6dof_measurement_buffer_;

  aslam::Transformation T_M0_G_;
  bool is_first_baseframe_estimate_processed_;

  // Store lidar depth camera for lidar point cloud projection.
  aslam::SensorId lidar_depth_camera_id_;
  aslam::Camera::Ptr lidar_depth_camera_sensor_;
  // Save transformation between lidar sensor frame and lidar camera sensor
  // frame.
  aslam::Transformation T_C_lidar_S_lidar_;

  common::TemporalBuffer<vi_map::ExternalFeaturesMeasurement::ConstPtr>
      external_features_measurement_temporal_buffer_;

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

  vi_map::VIMission& mission = map_->getMission(mission_id_);

  if (lidar_depth_camera_id_.isValid()) {
    CHECK(lidar_depth_camera_sensor_);

    // Transform into camera frame.
    point_cloud.applyTransformation(T_C_lidar_S_lidar_);

    cv::Mat range_image, image;
    cv::Mat* image_ptr =
        (FLAGS_map_builder_save_point_clouds_as_range_image_including_intensity_image)  // NOLINT
            ? &image
            : nullptr;

    backend::convertPointCloudToDepthMap(
        point_cloud, *lidar_depth_camera_sensor_, true /*use_openni_format*/,
        true /*create_range_image*/, &range_image, image_ptr);

    if (FLAGS_map_builder_visualize_lidar_depth_maps_in_ocv_window) {
      cv::namedWindow("depth");
      cv::namedWindow("intensity");
      double min;
      double max;
      cv::minMaxIdx(range_image, &min, &max, 0, 0, range_image > 1e-6);
      max = std::min(10000., max);

      cv::Mat scaled_range_image;
      float scale = 255 / (max - min);
      range_image.convertTo(scaled_range_image, CV_8UC1, scale, -min * scale);
      cv::Mat color_image;
      cv::applyColorMap(scaled_range_image, color_image, cv::COLORMAP_JET);
      color_image.setTo(cv::Scalar(0u, 0u, 0u), range_image < 1e-6);
      cv::imshow("depth", color_image);
      if (image_ptr != nullptr) {
        image.setTo(cv::Scalar(0u), range_image < 1e-6);
        cv::imshow("intensity", image);
      }
      cv::waitKey(1);
    }

    CHECK_EQ(CV_MAT_TYPE(range_image.type()), CV_16UC1);
    map_->addSensorResource(
        backend::ResourceType::kRawDepthMap, lidar_depth_camera_id_,
        lidar_measurement.getTimestampNanoseconds(), range_image, &mission);

    if (!image.empty()) {
      if (image.type() == CV_8UC1) {
        map_->addSensorResource(
            backend::ResourceType::kImageForDepthMap, lidar_depth_camera_id_,
            lidar_measurement.getTimestampNanoseconds(), image, &mission);
      } else if (image.type() == CV_8UC3) {
        map_->addSensorResource(
            backend::ResourceType::kColorImageForDepthMap,
            lidar_depth_camera_id_, lidar_measurement.getTimestampNanoseconds(),
            image, &mission);
      }
    }
  } else {
    backend::ResourceType point_cloud_type =
        backend::getResourceTypeForPointCloud(point_cloud);
    map_->addSensorResource(
        point_cloud_type, lidar_sensor_id,
        lidar_measurement.getTimestampNanoseconds(), point_cloud, &mission);
  }
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
