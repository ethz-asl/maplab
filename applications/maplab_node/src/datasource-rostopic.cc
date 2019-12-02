#include "maplab-node/datasource-rostopic.h"

#include <string>

#include <aslam/common/time.h>
#include <boost/bind.hpp>
#include <map-resources/resource-conversion.h>
#include <maplab-common/accessors.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensors/lidar.h>
#include <vio-common/rostopic-settings.h>
#include <vi-map/sensor-utils.h>

#include "maplab-node/ros-helpers.h"

DECLARE_bool(zero_initial_timestamps);

namespace maplab {

DataSourceRostopic::DataSourceRostopic(
    const vio_common::RosTopicSettings& ros_topics,
    const vi_map::SensorManager& sensor_manager)
    : DataSource(sensor_manager),
      shutdown_requested_(false),
      ros_topics_(ros_topics),
      image_transport_(node_handle_) {} 

DataSourceRostopic::~DataSourceRostopic() {}

void DataSourceRostopic::startStreaming() {
  registerSubscribers(ros_topics_);
}

void DataSourceRostopic::shutdown() {
  shutdown_requested_ = true;
}

void DataSourceRostopic::registerSubscribers(
    const vio_common::RosTopicSettings& ros_topics) {
  // Camera subscriber.
  const size_t num_cameras = ros_topics.camera_topic_cam_index_map.size();
  sub_images_.reserve(num_cameras);

  for (const std::pair<const std::string, size_t>& topic_camidx :
       ros_topics.camera_topic_cam_index_map) {
    CHECK(!topic_camidx.first.empty()) << "Camera " << topic_camidx.second
                                       << " is subscribed to an empty topic!";

    boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
      boost::bind(
        &DataSourceRostopic::imageCallback, this, _1, topic_camidx.second);

    constexpr size_t kRosSubscriberQueueSizeImage = 20u;
    image_transport::Subscriber image_sub = image_transport_.subscribe(
      topic_camidx.first, kRosSubscriberQueueSizeImage, image_callback);
    sub_images_.push_back(image_sub);
    VLOG(1) << "[MaplabNode-DataSource] Camera " << topic_camidx.second
            << " is subscribed to topic: '" << topic_camidx.first << "'";
  }

  // IMU subscriber.
  CHECK(!ros_topics.imu_topic.empty())
      << "IMU is subscribed to an empty topic!";
  constexpr size_t kRosSubscriberQueueSizeImu = 1000u;
  boost::function<void(const sensor_msgs::ImuConstPtr&)> imu_callback =
      boost::bind(&DataSourceRostopic::imuMeasurementCallback, this, _1);
  sub_imu_ = node_handle_.subscribe(
      ros_topics.imu_topic, kRosSubscriberQueueSizeImu, imu_callback);

  VLOG(1) << "[MaplabNode-DataSource] IMU is subscribed to topic: '"
          << ros_topics.imu_topic << "'";

  // Lidar subscriber.
  for (const std::pair<const std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.lidar_topic_sensor_id_map) {
    CHECK(topic_sensorid.second.isValid())
        << "The ROS-topic to Lidar sensor id association contains an invalid "
        << "sensor id! topic: " << topic_sensorid.first;
    CHECK(!topic_sensorid.first.empty())
        << "Lidar(" << topic_sensorid.second
        << ") is subscribed to an empty topic!";
    boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>
        lidar_callback = boost::bind(
            &DataSourceRostopic::lidarMeasurementCallback, this, _1,
            topic_sensorid.second);
    constexpr size_t kRosSubscriberQueueSizeLidar = 20u;

    sub_lidars_.emplace_back(node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeLidar, lidar_callback));

    VLOG(1) << "[MaplabNode-DataSource] Lidar(" << topic_sensorid.second
            << ") is subscribed to topic: '" << topic_sensorid.first << "'";
  }

  // Odometry subscriber.
  const std::pair<std::string, aslam::SensorId>& topic_sensorid =
      ros_topics.odometry_6dof_topic;
  if (!topic_sensorid.first.empty()) {
    CHECK(topic_sensorid.second.isValid());
    constexpr size_t kRosSubscriberQueueSizeWheelOdometry = 1000u;
    boost::function<void(const maplab_msgs::OdometryWithImuBiasesConstPtr&)>
        odometry_callback = boost::bind(
            &DataSourceRostopic::odometryEstimateCallback, this, _1,
            topic_sensorid.second);
    sub_odom_ = node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeWheelOdometry,
        odometry_callback);

    VLOG(1) << "[MaplabNode-DataSource] External odometry sensor with id "
            << topic_sensorid.second << " is "
            << "subscribed to topic: '" << topic_sensorid.first << "'";
  } else {
    LOG(FATAL) << "[MaplabNode-DataSource] Subscribing to the odometry sensor "
               << "failed, because the topic is empty!";
  }

  // Absolute pose constraint subscribers.
  for (const std::pair<const std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.absolute_6dof_topic_map) {
    constexpr size_t kRosSubscriberQueueSizeAbsoluteConstraints = 1000u;
    boost::function<void(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr&)>
        absolute_callback = boost::bind(
            &DataSourceRostopic::absolute6DoFConstraintCallback, this, _1,
            topic_sensorid.second);
    sub_absolute_6dof_ = node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeAbsoluteConstraints,
        absolute_callback);

    VLOG(1)
        << "[MaplabNode-DataSource] External absolute 6DoF pose sensor with id "
        << topic_sensorid.second << " is "
        << "subscribed to topic: '" << topic_sensorid.first << "'";
  }

  // Wheel odometry constraint subscribers.
  for (const std::pair<const std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.wheel_odometry_topic_map) {
    constexpr size_t kRosSubscriberQueueSizeWheelOdometryConstraints = 1000u;
    boost::function<void(const nav_msgs::OdometryConstPtr&)>
        wheel_odometry_callback = boost::bind(
            &DataSourceRostopic::wheelOdometryConstraintCallback, this, _1,
            topic_sensorid.second);
    sub_wheel_odometry_ = node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeWheelOdometryConstraints,
        wheel_odometry_callback);

    VLOG(1) << "[MaplabNode-DataSource] External wheel odometry "
            << "sensor with id " << topic_sensorid.second
            << " is subscribed to topic: '" << topic_sensorid.first << "'";
  }

#ifdef VOXGRAPH
  // Loop closure constraint subscribers.
  for (const std::pair<const std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.loop_closure_topic_map) {
    constexpr size_t kRosSubscriberQueueSizeLoopClosureConstraints = 1000u;
    boost::function<void(const voxgraph_msgs::LoopClosureEdgeListConstPtr&)>
        loop_closure_callback = boost::bind(
            &DataSourceRostopic::voxgraphLoopClosureConstraintCallback, this,
            _1, topic_sensorid.second);
    sub_loop_closure_ = node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeLoopClosureConstraints,
        loop_closure_callback);

    VLOG(1) << "[MaplabNode-DataSource] External loop closure constraint "
            << "sensor with id " << topic_sensorid.second
            << " is subscribed to topic: '" << topic_sensorid.first << "'";
  }
#endif  // VOXGRAPH

#ifdef VOXGRAPH
  // Point cloud submap subscriber, based on voxgraph types.
  for (const std::pair<const std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.pointcloud_map_topic_map) {
    constexpr size_t kRosSubscriberQueueSizeRelativeConstraints = 1000u;
    boost::function<void(const voxgraph_msgs::MapSurfaceConstPtr&)>
        submap_callback = boost::bind(
            &DataSourceRostopic::voxgraphPointCloudMapCallback, this, _1,
            topic_sensorid.second);
    sub_pointcloud_map_ = node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeRelativeConstraints,
        submap_callback);

    VLOG(1) << "[MaplabNode-DataSource] External point cloud (sub-)map "
            << "sensor with id " << topic_sensorid.second
            << " is subscribed to topic: '" << topic_sensorid.first << "'";
  }
#else
  // Point cloud submap subscriber, based on PointCloud2 types.
  for (const std::pair<const std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.pointcloud_map_topic_map) {
    constexpr size_t kRosSubscriberQueueSizeRelativeConstraints = 1000u;
    boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>
        submap_callback = boost::bind(
            &DataSourceRostopic::pointCloudMapCallback, this, _1,
            topic_sensorid.second);
    sub_pointcloud_map_ = node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeRelativeConstraints,
        submap_callback);

    VLOG(1) << "[MaplabNode-DataSource] External point cloud (sub-)map "
            << "sensor with id " << topic_sensorid.second
            << " is subscribed to topic: '" << topic_sensorid.first << "'";
  }
#endif  // VOXGRAPH
}

void DataSourceRostopic::imageCallback(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx) {
  if (shutdown_requested_) {
    return;
  }

  vio::ImageMeasurement::Ptr image_measurement =
      convertRosImageToMaplabImage(image_message, camera_idx);  
  CHECK(image_measurement);

  // Apply the IMU to camera time shift.
  if (FLAGS_imu_to_camera_time_offset_ns != 0) {
    image_measurement->timestamp += FLAGS_imu_to_camera_time_offset_ns;
  }

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(&(image_measurement->timestamp))) {
    invokeImageCallbacks(image_measurement);
  }
}

void DataSourceRostopic::imuMeasurementCallback(
    const sensor_msgs::ImuConstPtr& msg) {
  if (shutdown_requested_) {
    return;
  }

  vio::ImuMeasurement::Ptr imu_measurement = convertRosImuToMaplabImu(msg);
  CHECK(imu_measurement);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(&(imu_measurement->timestamp))) {
    invokeImuCallbacks(imu_measurement);
  }
}

void DataSourceRostopic::lidarMeasurementCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  vi_map::RosLidarMeasurement::Ptr lidar_measurement =
      convertRosCloudToMaplabCloud(msg, sensor_id);
  CHECK(lidar_measurement);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(
          lidar_measurement->getTimestampNanosecondsMutable())) {
    invokeLidarCallbacks(lidar_measurement);
  }
}

void DataSourceRostopic::odometryEstimateCallback(
    const maplab_msgs::OdometryWithImuBiasesConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  const vi_map::Odometry6DoF& sensor =
      sensor_manager_.getSensor<vi_map::Odometry6DoF>(sensor_id);
  const aslam::Transformation& T_B_S =
      sensor_manager_.getSensor_T_B_S(sensor_id);

  maplab::OdometryEstimate::Ptr odometry_measurement =
      convertRosOdometryMsgToOdometryEstimate(msg, T_B_S, sensor);
  CHECK(odometry_measurement);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(&(odometry_measurement->timestamp_ns))) {
    invokeOdometryCallbacks(odometry_measurement);
  }
}

void DataSourceRostopic::absolute6DoFConstraintCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  const vi_map::Absolute6DoF& sensor =
      sensor_manager_.getSensor<vi_map::Absolute6DoF>(sensor_id);
  vi_map::Absolute6DoFMeasurement::Ptr absolute_constraint =
      convertPoseWithCovarianceToAbsolute6DoFConstraint(msg, sensor);
  if (!absolute_constraint) {
    LOG(ERROR) << "Received invalid Absolute6DoF constraint!";
    return;
  }

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(
          absolute_constraint->getTimestampNanosecondsMutable())) {
    invokeAbsolute6DoFConstraintCallbacks(absolute_constraint);
  }
}

#ifdef VOXGRAPH
void DataSourceRostopic::voxgraphLoopClosureConstraintCallback(
    const voxgraph_msgs::LoopClosureEdgeListConstPtr& lc_edges_msg,
    const aslam::SensorId& sensor_id) {
  CHECK(lc_edges_msg);
  if (shutdown_requested_) {
    return;
  }

  const vi_map::LoopClosureSensor& sensor =
      sensor_manager_.getSensor<vi_map::LoopClosureSensor>(sensor_id);

  std::vector<vi_map::LoopClosureMeasurement::Ptr> lc_edges;
  convertVoxgraphEdgeListToLoopClosureConstraint(
      lc_edges_msg, sensor, &lc_edges);

  VLOG(3) << "[DataSourceRostopic] Received a list of " << lc_edges.size()
          << " loop closure constraints.";

  for (const vi_map::LoopClosureMeasurement::Ptr& lc_edge : lc_edges) {
    CHECK(lc_edge);
    // Shift timestamps to start at 0.
    if (!FLAGS_zero_initial_timestamps ||
        (shiftByFirstTimestamp(lc_edge->getTimestampNanosecondsAMutable()) &&
         shiftByFirstTimestamp(lc_edge->getTimestampNanosecondsBMutable()))) {
      invokeLoopClosureConstraintCallbacks(lc_edge);
    }
  }
}
#endif  // VOXGRAPH

#ifdef VOXGRAPH
void DataSourceRostopic::voxgraphPointCloudMapCallback(
    const voxgraph_msgs::MapSurfaceConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  vi_map::RosPointCloudMapSensorMeasurement::Ptr pointcloud_map =
      convertVoxgraphMapToPointCloudMap(msg, sensor_id);
  CHECK(pointcloud_map);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(pointcloud_map->getTimestampNanosecondsMutable())) {
    invokePointCloudMapCallbacks(pointcloud_map);
  }
}
#else
void DataSourceRostopic::pointCloudMapCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  vi_map::RosPointCloudMapSensorMeasurement::Ptr pointcloud_map =
      convertRosPointCloudToPointCloudMap(msg, sensor_id);
  CHECK(pointcloud_map);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(pointcloud_map->getTimestampNanosecondsMutable())) {
    invokePointCloudMapCallbacks(pointcloud_map);
  }
}
#endif  // VOXGRAPH

void DataSourceRostopic::wheelOdometryConstraintCallback(
    const nav_msgs::OdometryConstPtr& wheel_odometry_msg,
    const aslam::SensorId& sensor_id) {
  CHECK(wheel_odometry_msg);
  if (shutdown_requested_) {
    return;
  }

  vi_map::WheelOdometryMeasurement::Ptr wheel_odometry_measurement;
  wheel_odometry_measurement =
      convertRosOdometryToMaplabWheelOdometry(wheel_odometry_msg, sensor_id);
  CHECK(wheel_odometry_measurement);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      (shiftByFirstTimestamp(
          wheel_odometry_measurement->getTimestampNanosecondsMutable()))) {
    invokeWheelOdometryConstraintCallbacks(wheel_odometry_measurement);
  }
}

}  // namespace maplab
