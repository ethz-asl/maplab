#ifndef MAPLAB_NODE_DATASOURCE_H_
#define MAPLAB_NODE_DATASOURCE_H_

#include <functional>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <maplab-common/macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <vi-map/sensor-manager.h>
#include <vio-common/vio-types.h>

#include "maplab-node/odometry-estimate.h"

DECLARE_double(maplab_throttle_frequency_odometry);
DECLARE_double(maplab_batch_imu_measurements_at_frequency);

namespace maplab {

#define DECLARE_SENSOR_CALLBACK(SENSOR_NAME, MEASUREMENT_TYPE)                 \
 public: /* NOLINT */                                                          \
  typedef std::function<void(MEASUREMENT_TYPE)> SENSOR_NAME##Callback;         \
  /* NOLINT */ void register##SENSOR_NAME##Callback(                           \
      const SENSOR_NAME##Callback& cb) { /* NOLINT */                          \
    CHECK(cb);                                                                 \
    SENSOR_NAME##_callbacks_.emplace_back(cb);                                 \
  }                                                                            \
  void invoke##SENSOR_NAME##Callbacks(const MEASUREMENT_TYPE& measurement)     \
      const {                                                                  \
    if (SENSOR_NAME##_callbacks_.empty()) {                                    \
      LOG(WARNING) << "The data source is subscribed to a sensor providing '"  \
                   << #SENSOR_NAME                                             \
                   << "' but no callbacks are registered to pass the data on " \
                   << "to the message flow! Did you register a flow "          \
                   << "subscriber in DataSourceFlow::attachToMessageFlow?";    \
    }                                                                          \
    for (const SENSOR_NAME##Callback& callback /* NOLINT */ :                  \
         SENSOR_NAME##_callbacks_) {                                           \
      callback(measurement);                                                   \
    }                                                                          \
  }                                                                            \
                                                                               \
 private: /* NOLINT */                                                         \
  std::vector<SENSOR_NAME##Callback> SENSOR_NAME##_callbacks_;

class CallbackManager {
  DECLARE_SENSOR_CALLBACK(Image, vio::ImageMeasurement::ConstPtr);
  DECLARE_SENSOR_CALLBACK(Imu, vio::BatchedImuMeasurements::ConstPtr);
  DECLARE_SENSOR_CALLBACK(Lidar, vi_map::RosLidarMeasurement::ConstPtr);
  DECLARE_SENSOR_CALLBACK(Odometry, maplab::OdometryEstimate::ConstPtr);
  DECLARE_SENSOR_CALLBACK(
      Absolute6DoFConstraint, vi_map::Absolute6DoFMeasurement::Ptr);
  DECLARE_SENSOR_CALLBACK(
      LoopClosureConstraint, vi_map::LoopClosureMeasurement::ConstPtr);
  DECLARE_SENSOR_CALLBACK(
      WheelOdometryConstraint, vi_map::WheelOdometryMeasurement::Ptr);
  DECLARE_SENSOR_CALLBACK(
      PointCloudMap, vi_map::RosPointCloudMapSensorMeasurement::ConstPtr);
  DECLARE_SENSOR_CALLBACK(
      ExternalFeatures, vi_map::ExternalFeaturesMeasurement::ConstPtr);
};

class DataSource : public CallbackManager {
 public:
  MAPLAB_POINTER_TYPEDEFS(DataSource);
  MAPLAB_DISALLOW_EVIL_CONSTRUCTORS(DataSource);

  explicit DataSource(const vi_map::SensorManager& sensor_manager)
      : sensor_manager_(sensor_manager), timestamp_at_start_ns_(-1) {}

  virtual ~DataSource() {}

  // Has all data been released to the output queues.
  virtual void startStreaming() = 0;
  virtual void shutdown() = 0;

  virtual bool allDataStreamed() const = 0;
  virtual std::string getDatasetName() const = 0;

  virtual void registerEndOfDataCallback(const std::function<void()>& cb) {
    CHECK(cb);
    end_of_data_callbacks_.emplace_back(cb);
  }
  void invokeEndOfDataCallbacks() const {
    for (const std::function<void()>& cb : end_of_data_callbacks_) {
      cb();
    }
  }

  // If this is the first timestamp we receive, we store it and shift all
  // subsequent timestamps. Will return false for any timestamps that are
  // smaller than the first timestamp received.
  bool shiftByFirstTimestamp(int64_t* timestamp_ns) {
    CHECK_NOTNULL(timestamp_ns);
    CHECK_GE(*timestamp_ns, 0);

    // There is a slight race condition happening here, without a mutex. But
    // it shouldn't matter except for initialization which message actually
    // sets the initial timestamp. Definite gain in performance like this
    if (timestamp_at_start_ns_ == -1) {
      timestamp_at_start_ns_ = *timestamp_ns;
      *timestamp_ns = 0;
      LOG(WARNING)
          << "Set the first timestamp that was received to "
          << timestamp_at_start_ns_
          << "ns, all subsequent timestamp will be shifted by that amount. "
          << "Be aware that the published estimation results are also "
          << "expressed in this shifted time frame!";
    } else {
      if (*timestamp_ns < timestamp_at_start_ns_) {
        LOG(WARNING) << "Received timestamp that is earlier than the first "
                     << "timestamp of the data source! First timestamp: "
                     << timestamp_at_start_ns_
                     << "ns, received timestamp: " << *timestamp_ns << "ns.";
        return false;
      }
      *timestamp_ns = *timestamp_ns - timestamp_at_start_ns_;
    }

    CHECK_GE(timestamp_at_start_ns_, 0);
    CHECK_GE(*timestamp_ns, 0);
    return true;
  }

 protected:
  DataSource() = default;

  const vi_map::SensorManager& sensor_manager_;

 private:
  std::vector<std::function<void()>> end_of_data_callbacks_;

  std::atomic<int64_t> timestamp_at_start_ns_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_DATASOURCE_H_
