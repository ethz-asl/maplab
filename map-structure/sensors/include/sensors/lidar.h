#ifndef SENSORS_LIDAR_H_
#define SENSORS_LIDAR_H_

#include <memory>
#include <string>
#include <vector>

#include <aslam/common/pose-types.h>
#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <resources-common/point-cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {
class VIMap;
class MeasurementsTest_TestAccessorsLidar_Test;

class Lidar final : public aslam::Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Lidar);

  enum class TimestampUnit {
    kNanoSeconds = 0,
    kMicroSeconds = 1,
    kMilliSeconds = 2,
    kSeconds = 3,
  };

  Lidar();
  explicit Lidar(const aslam::SensorId& sensor_id);
  Lidar(const aslam::SensorId& sensor_id, const std::string& topic);

  void operator=(const Lidar& other) {
    aslam::Sensor::operator=(other);
    has_point_timestamps_ = other.has_point_timestamps_;
    has_relative_point_timestamps_ = other.has_relative_point_timestamps_;
    timestamp_unit_ = other.timestamp_unit_;
  }

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(aligned_shared<Lidar>(*this));
  }

  Lidar* cloneWithNewIds() const {
    Lidar* cloned_lidar = new Lidar();
    *cloned_lidar = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_lidar->setId(new_id);
    return cloned_lidar;
  }

  uint8_t getSensorType() const override {
    return SensorType::kLidar;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kLidarIdentifier);
  }

  // Whether the LiDAR publishes timing information for individual points.
  bool hasPointTimestamps() const {
    return has_point_timestamps_;
  }

  // Whether the timestamps for the points are relative to the message
  // timestamp, or in absolute times (e.g. Hesai LiDARs do this).
  bool hasRelativePointTimestamps() const {
    return has_relative_point_timestamps_;
  };

  // Get conversion factor between the LiDAR timestamps for points
  // and nanoseconds. This is LiDAR model and driver specific.
  int32_t getTimestampConversionToNanoseconds() const;

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    return true;
  }

  void setRandomImpl() override {}

  bool isEqualImpl(const Sensor& other, const bool verbose) const override {
    const Lidar* other_lidar = dynamic_cast<const Lidar*>(&other);
    if (other_lidar == nullptr) {
      LOG_IF(WARNING, verbose) << "Other sensor is not a LiDAR!";
      return false;
    }

    bool is_equal = true;
    is_equal &= has_point_timestamps_ == other_lidar->has_point_timestamps_;
    is_equal &= has_relative_point_timestamps_ ==
                other_lidar->has_relative_point_timestamps_;
    is_equal &= timestamp_unit_ == other_lidar->timestamp_unit_;

    if (!is_equal) {
      LOG_IF(WARNING, verbose)
          << "This LiDAR sensor " << id_
          << "\n has_point_timestamps_: " << has_point_timestamps_
          << "\n has_relative_point_timestamps_: "
          << has_relative_point_timestamps_ << "\n timestamp_unit_: "
          << static_cast<std::underlying_type<TimestampUnit>::type>(
                 timestamp_unit_);

      LOG_IF(WARNING, verbose)
          << "Other LiDAR sensor " << other_lidar->id_
          << "\n has_point_timestamps_: " << other_lidar->has_point_timestamps_
          << "\n has_relative_point_timestamps_: "
          << other_lidar->has_relative_point_timestamps_
          << "\n timestamp_unit_: "
          << static_cast<std::underlying_type<TimestampUnit>::type>(
                 other_lidar->timestamp_unit_);
      return false;
    }

    return true;
  }

  bool has_point_timestamps_;
  bool has_relative_point_timestamps_;
  TimestampUnit timestamp_unit_;
};

template <typename PointCloudType>
class LidarMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(LidarMeasurement<PointCloudType>);

  LidarMeasurement() = default;
  LidarMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const PointCloudType& point_cloud)
      : Measurement(sensor_id, timestamp_nanoseconds),
        point_cloud_(point_cloud) {
    CHECK(isValid());
  }
  LidarMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds)
      : Measurement(sensor_id, timestamp_nanoseconds) {
    CHECK(isValid());
  }

  ~LidarMeasurement() = default;

  const PointCloudType& getPointCloud() const {
    return point_cloud_;
  }

  PointCloudType* getPointCloudMutable() {
    return &point_cloud_;
  }

  bool operator==(const LidarMeasurement& other) const {
    return Measurement::operator==(other) &&
           point_cloud_ == other.getPointCloud();
  }

 private:
  explicit LidarMeasurement(const aslam::SensorId& sensor_id)
      : Measurement(sensor_id) {}
  bool isValidImpl() const override;

  void setRandomImpl() override;

  PointCloudType point_cloud_;
};

}  // namespace vi_map

namespace vi_map {

typedef LidarMeasurement<resources::PointCloud> MaplabLidarMeasurement;
typedef LidarMeasurement<sensor_msgs::PointCloud2> RosLidarMeasurement;

DEFINE_MEASUREMENT_CONTAINERS(MaplabLidarMeasurement);
DEFINE_MEASUREMENT_CONTAINERS(RosLidarMeasurement);

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(MaplabLidarMeasurement)
DEFINE_MEASUREMENT_HASH(RosLidarMeasurement)

#endif  // SENSORS_LIDAR_H_
