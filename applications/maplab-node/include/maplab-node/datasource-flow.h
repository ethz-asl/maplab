#ifndef MAPLAB_NODE_DATASOURCE_FLOW_H_
#define MAPLAB_NODE_DATASOURCE_FLOW_H_
#include <memory>

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <vio-common/rostopic-settings.h>
#include <vio-common/vio-types.h>

#include "maplab-node/datasource-factory.h"
#include "maplab-node/datasource.h"
#include "maplab-node/flow-topics.h"

namespace maplab {

class DataSourceFlow {
 public:
  explicit DataSourceFlow(const vi_map::SensorManager& sensor_manager) {
    vio_common::RosTopicSettings rostopics_settings(sensor_manager);
    datasource_.reset(createAndConfigureDataSourcefromGflagsAndTopics(
        rostopics_settings, sensor_manager));
    CHECK(datasource_);
  }

  ~DataSourceFlow() {
    shutdown();
  }

  void attachToMessageFlow(message_flow::MessageFlow* flow) {
    CHECK_NOTNULL(flow);
    datasource_->registerImageCallback(
        flow->registerPublisher<message_flow_topics::IMAGE_MEASUREMENTS>());
    datasource_->registerImuCallback(
        flow->registerPublisher<message_flow_topics::IMU_MEASUREMENTS>());
    datasource_->registerLidarCallback(
        flow->registerPublisher<message_flow_topics::LIDAR_MEASUREMENTS>());
    datasource_->registerOdometryCallback(
        flow->registerPublisher<message_flow_topics::ODOMETRY_ESTIMATES>());
    datasource_->registerAbsolute6DoFConstraintCallback(
        flow->registerPublisher<
            message_flow_topics::ABSOLUTE_6DOF_CONSTRAINTS>());
    datasource_->registerWheelOdometryConstraintCallback(
        flow->registerPublisher<
            message_flow_topics::WHEEL_ODOMETRY_CONSTRAINTS>());
    datasource_->registerLoopClosureConstraintCallback(
        flow->registerPublisher<
            message_flow_topics::LOOP_CLOSURE_CONSTRAINTS>());
    datasource_->registerPointCloudMapCallback(
        flow->registerPublisher<message_flow_topics::POINTCLOUD_MAP>());
  }

  void startStreaming() {
    LOG(INFO) << "Starting data source streaming...";
    datasource_->startStreaming();
  }

  void shutdown() {
    LOG(INFO) << "Shutting down data source streaming...";
    datasource_->shutdown();
  }

  void registerEndOfDataCallback(const std::function<void()>& cb) {
    CHECK(cb);
    datasource_->registerEndOfDataCallback(cb);
  }

 private:
  std::unique_ptr<DataSource> datasource_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_DATASOURCE_FLOW_H_
