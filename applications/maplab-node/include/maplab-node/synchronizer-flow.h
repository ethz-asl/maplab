#ifndef MAPLAB_NODE_SYNCHRONIZER_FLOW_H_
#define MAPLAB_NODE_SYNCHRONIZER_FLOW_H_

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <string>
#include <vi-map/sensor-utils.h>
#include <vio-common/vio-types.h>
#include "maplab-node/flow-topics.h"
#include "maplab-node/synchronizer.h"

static constexpr char kSubscriberNodeName[] = "Synchronizer";

namespace maplab {

class SynchronizerFlow {
 public:
  explicit SynchronizerFlow(const vi_map::SensorManager& sensor_manager)
      : sensor_manager_(sensor_manager), synchronizer_(sensor_manager) {
    delivery_method_.exclusivity_group_id =
        kExclusivityGroupIdRawSensorDataSubscribers;
  }

  ~SynchronizerFlow() {
    shutdown();
  }

  void start() {
    synchronizer_.start();
  }

  void initializeInertialData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectImuData();

    message_flow_->registerSubscriber<message_flow_topics::IMU_MEASUREMENTS>(
        kSubscriberNodeName, delivery_method_,
        [this](const vio::BatchedImuMeasurements::ConstPtr& imu) {
          CHECK(imu);
          const size_t num_measurements = imu->batch.size();
          Eigen::Matrix<int64_t, 1, Eigen::Dynamic> timestamps_nanoseconds(
              1, num_measurements);
          Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements(
              6, num_measurements);
          for (size_t idx = 0u; idx < num_measurements; ++idx) {
            const vio::ImuMeasurement& measurement = imu->batch[idx];
            timestamps_nanoseconds(idx) = measurement.timestamp;
            imu_measurements.col(idx) = measurement.imu_data;
          }

          this->synchronizer_.processImuMeasurements(
              timestamps_nanoseconds, imu_measurements);
        });
  }

  void initializeOdometryData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectOdometryData();

    message_flow_->registerSubscriber<message_flow_topics::ODOMETRY_ESTIMATES>(
        kSubscriberNodeName, delivery_method_,
        [this](const OdometryEstimate::ConstPtr& odometry_estimate) {
          CHECK(odometry_estimate);
          this->synchronizer_.processOdometryMeasurement(
              odometry_estimate->vinode);
        });
  }

  void initializeLidarData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectLidarData();

    message_flow_->registerSubscriber<message_flow_topics::LIDAR_MEASUREMENTS>(
        kSubscriberNodeName, delivery_method_,
        [this](const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement) {
          CHECK(lidar_measurement);
          this->synchronizer_.processLidarMeasurement(lidar_measurement);
        });
    synchronizer_.registerLidarMeasurementCallback(
        message_flow_->registerPublisher<
            message_flow_topics::SYNCED_LIDAR_MEASUREMENTS>());
  }

  void initializeAbsolute6DoFData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectAbsolute6DoFData();

    message_flow_
        ->registerSubscriber<message_flow_topics::ABSOLUTE_6DOF_CONSTRAINTS>(
            kSubscriberNodeName, delivery_method_,
            [this](const vi_map::Absolute6DoFMeasurement::Ptr&
                       absolute_6dof_measurement) {
              CHECK(absolute_6dof_measurement);
              this->synchronizer_.processAbsolute6DoFMeasurement(
                  absolute_6dof_measurement);
            });
    synchronizer_.registerAbsolute6DoFMeasurementCallback(
        message_flow_
            ->registerPublisher<message_flow_topics::SYNCED_ABSOLUTE_6DOF>());
  }

  void initializeWheelOdometryData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectWheelOdometryData();

    message_flow_
        ->registerSubscriber<message_flow_topics::WHEEL_ODOMETRY_CONSTRAINTS>(
            kSubscriberNodeName, delivery_method_,
            [this](const vi_map::WheelOdometryMeasurement::ConstPtr&
                       wheel_odometry_measurement) {
              CHECK(wheel_odometry_measurement);
              this->synchronizer_.processWheelOdometryMeasurement(
                  wheel_odometry_measurement);
            });
    synchronizer_.registerWheelOdometryMeasurementCallback(
        message_flow_
            ->registerPublisher<message_flow_topics::SYNCED_WHEEL_ODOMETRY>());
  }

  void initializeLoopClosureData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectLoopClosureData();

    message_flow_
        ->registerSubscriber<message_flow_topics::LOOP_CLOSURE_CONSTRAINTS>(
            kSubscriberNodeName, delivery_method_,
            [this](const vi_map::LoopClosureMeasurement::ConstPtr&
                       loop_closure_measurement) {
              CHECK(loop_closure_measurement);
              this->synchronizer_.processLoopClosureMeasurement(
                  loop_closure_measurement);
            });
    synchronizer_.registerLoopClosureMeasurementCallback(
        message_flow_
            ->registerPublisher<message_flow_topics::SYNCED_LOOP_CLOSURE>());
  }

  void initializePointCloudMapData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectPointCloudMapData();

    message_flow_->registerSubscriber<message_flow_topics::POINTCLOUD_MAP>(
        kSubscriberNodeName, delivery_method_,
        [this](const vi_map::RosPointCloudMapSensorMeasurement::ConstPtr&
                   pointcloud_map_measurement) {
          CHECK(pointcloud_map_measurement);
          this->synchronizer_.processPointCloudMapMeasurement(
              pointcloud_map_measurement);
        });
    synchronizer_.registerPointCloudMapSensorMeasurementCallback(
        message_flow_
            ->registerPublisher<message_flow_topics::SYNCED_POINTCLOUD_MAP>());
  }

  void initializeVisualData(aslam::NCamera::Ptr mapping_ncamera) {
    CHECK_NOTNULL(message_flow_);
    CHECK(mapping_ncamera);

    synchronizer_.initializeNCameraSynchronization(mapping_ncamera);
    synchronizer_.expectVisualData();

    message_flow_->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
        kSubscriberNodeName, delivery_method_,
        [this](const vio::ImageMeasurement::ConstPtr& image) {
          CHECK(image);
          this->synchronizer_.processCameraImage(image);
        });
    synchronizer_.registerSynchronizedNFrameCallback(
        message_flow_
            ->registerPublisher<message_flow_topics::SYNCED_NFRAMES>());
  }

  void initializeExternalFeaturesData(
      const aslam::SensorIdSet& external_feature_sensor_ids) {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.initializeExternalFeaturesSynchronization(
        external_feature_sensor_ids);
    synchronizer_.expectExternalFeaturesData();

    message_flow_->registerSubscriber<message_flow_topics::EXTERNAL_FEATURES>(
        kSubscriberNodeName, delivery_method_,
        [this](const vi_map::ExternalFeaturesMeasurement::ConstPtr&
                   external_features_measurement) {
          CHECK(external_features_measurement);
          this->synchronizer_.processExternalFeatureMeasurement(
              external_features_measurement);
        });
    synchronizer_.registerExternalFeaturesMeasurementCallback(
        message_flow_->registerPublisher<
            message_flow_topics::SYNCED_EXTERNAL_FEATURES>());
  }

  void attachToMessageFlow(message_flow::MessageFlow* const flow) {
    CHECK_NOTNULL(flow);
    message_flow_ = flow;
  }

  void shutdown() {
    synchronizer_.shutdown();
  }

  const vio_common::PoseLookupBuffer& T_M_B_buffer() {
    return this->synchronizer_.T_M_B_buffer();
  }

  void registerEndOfDataCallback(const std::function<void()>& cb) {
    CHECK(cb);
    synchronizer_.registerEndOfDataCallback(cb);
  }

 private:
  const vi_map::SensorManager& sensor_manager_;

  message_flow::DeliveryOptions delivery_method_;
  message_flow::MessageFlow* message_flow_;

  Synchronizer synchronizer_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_SYNCHRONIZER_FLOW_H_
