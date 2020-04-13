#ifndef MAPLAB_NODE_DATASOURCE_ROSTOPIC_H_
#define MAPLAB_NODE_DATASOURCE_ROSTOPIC_H_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <aslam/common/time.h>
#include <maplab_msgs/OdometryWithImuBiases.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/lidar.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <vio-common/rostopic-settings.h>
#include <vio-common/vio-types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop

#ifdef VOXGRAPH
#include <voxgraph_msgs/LoopClosureEdge.h>
#include <voxgraph_msgs/LoopClosureEdgeList.h>
#include <voxgraph_msgs/MapSurface.h>
#endif  // VOXGRAPH

#include "maplab-node/datasource.h"
#include "maplab-node/odometry-estimate.h"

DECLARE_int64(imu_to_camera_time_offset_ns);

namespace maplab {

class DataSourceRostopic : public DataSource {
 public:
  explicit DataSourceRostopic(
      const vio_common::RosTopicSettings& settings,
      const vi_map::SensorManager& sensor_manager);
  virtual ~DataSourceRostopic();

  virtual void startStreaming();
  virtual void shutdown();

  virtual bool allDataStreamed() const {
    // Workers streaming live data never run out of data.
    return !ros::ok();
  }

  virtual std::string getDatasetName() const {
    return "live-rostopic";
  }

 private:
  void registerSubscribers(const vio_common::RosTopicSettings& ros_topics);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg, size_t camera_idx);
  void imuMeasurementCallback(const sensor_msgs::ImuConstPtr& msg);
  void lidarMeasurementCallback(
      const sensor_msgs::PointCloud2ConstPtr& msg,
      const aslam::SensorId& sensor_id);
  void odometryEstimateCallback(
      const maplab_msgs::OdometryWithImuBiasesConstPtr& msg,
      const aslam::SensorId& sensor_id);
  void absolute6DoFConstraintCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg,
      const aslam::SensorId& sensor_id);
  void wheelOdometryConstraintCallback(
      const nav_msgs::OdometryConstPtr& msg, const aslam::SensorId& sensor_id);

#ifdef VOXGRAPH
  // Voxgraph specific subscribers.
  void voxgraphLoopClosureConstraintCallback(
      const voxgraph_msgs::LoopClosureEdgeListConstPtr& msg,
      const aslam::SensorId& sensor_id);
#endif  // VOXGRAPH

#ifdef VOXGRAPH
  void voxgraphPointCloudMapCallback(
      const voxgraph_msgs::MapSurfaceConstPtr& msg,
      const aslam::SensorId& sensor_id);
#else
  void pointCloudMapCallback(
      const sensor_msgs::PointCloud2ConstPtr& msg,
      const aslam::SensorId& sensor_id);
#endif  // VOXGRAPH

  std::atomic<bool> shutdown_requested_;
  const vio_common::RosTopicSettings ros_topics_;
  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;
  std::vector<image_transport::Subscriber> sub_images_;
  ros::Subscriber sub_imu_;
  std::vector<ros::Subscriber> sub_lidars_;
  ros::Subscriber sub_odom_;

  ros::Subscriber sub_loop_closure_;
  ros::Subscriber sub_absolute_6dof_;
  ros::Subscriber sub_wheel_odometry_;
  ros::Subscriber sub_pointcloud_map_;

  int64_t last_imu_timestamp_ns_;
  int64_t last_imu_dispatch_timestamp_ns_;
  const int64_t imu_batch_period_ns_;
  vio::BatchedImuMeasurements::Ptr current_imu_batch_;

  std::vector<int64_t> last_image_timestamp_ns_;
  int64_t last_wheel_odometry_timestamp_ns_;
  int64_t last_odometry_timestamp_ns_;
  const int64_t odometry_min_period_ns_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_DATASOURCE_ROSTOPIC_H_
