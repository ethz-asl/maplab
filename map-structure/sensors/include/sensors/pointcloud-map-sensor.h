#ifndef SENSORS_POINTCLOUD_MAP_SENSOR_H_
#define SENSORS_POINTCLOUD_MAP_SENSOR_H_

#include <string>

#include <aslam/common/sensor.h>
#include <aslam/common/time.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/lidar.h"
#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

// Represents an external source of point cloud maps that can be associated with
// a specific frame and time along the pose graph. The main difference to the
// lidar is that it does not represent a physical sensor that recorded these
// point clouds from the sensor frame, but the points are likely an accumulation
// of multiple depth sensor measurements, potentially even derived from other
// sensor data, such as images.
class PointCloudMapSensor final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(PointCloudMapSensor);

  PointCloudMapSensor() = default;
  explicit PointCloudMapSensor(const aslam::SensorId& sensor_id);
  PointCloudMapSensor(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<PointCloudMapSensor>(*this));
  }

  PointCloudMapSensor* cloneWithNewIds() const {
    PointCloudMapSensor* cloned_local_map_sensor = new PointCloudMapSensor();
    *cloned_local_map_sensor = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_local_map_sensor->setId(new_id);
    return cloned_local_map_sensor;
  }

  uint8_t getSensorType() const override {
    return SensorType::kPointCloudMapSensor;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kPointCloudMapSensorIdentifier);
  }

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    return true;
  }

  void setRandomImpl() override {}

  bool isEqualImpl(
      const Sensor& /*other*/, const bool /*verbose*/) const override {
    return true;
  }
};

// Stores a point cloud anchored in the sensor frame at a specific time stamp.
// In contrast to the lidar measurement this point cloud is an accumulation of
// many measurements and has not been directly observed from the sensor frame.
template <typename PointCloudType>
class PointCloudMapSensorMeasurement final
    : public LidarMeasurement<PointCloudType> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(PointCloudMapSensorMeasurement);

  PointCloudMapSensorMeasurement() = default;
  PointCloudMapSensorMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const PointCloudType& point_cloud)
      : LidarMeasurement<PointCloudType>(
            sensor_id, timestamp_nanoseconds, point_cloud) {}

  PointCloudMapSensorMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds)
      : LidarMeasurement<PointCloudType>(sensor_id, timestamp_nanoseconds) {}

  ~PointCloudMapSensorMeasurement() = default;
};

typedef PointCloudMapSensorMeasurement<resources::PointCloud>
    MaplabPointCloudMapSensorMeasurement;
typedef PointCloudMapSensorMeasurement<pcl::PointCloud<pcl::PointXYZI>>
    PclPointCloudMapSensorMeasurement;
typedef PointCloudMapSensorMeasurement<pcl::PointCloud<pcl::OusterPointType>>
    OusterPointCloudMapSensorMeasurement;
typedef PointCloudMapSensorMeasurement<sensor_msgs::PointCloud2>
    RosPointCloudMapSensorMeasurement;

DEFINE_MEASUREMENT_CONTAINERS(MaplabPointCloudMapSensorMeasurement);
DEFINE_MEASUREMENT_CONTAINERS(PclPointCloudMapSensorMeasurement);
DEFINE_MEASUREMENT_CONTAINERS(RosPointCloudMapSensorMeasurement);
DEFINE_MEASUREMENT_CONTAINERS(OusterPointCloudMapSensorMeasurement);

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(MaplabPointCloudMapSensorMeasurement)
DEFINE_MEASUREMENT_HASH(PclPointCloudMapSensorMeasurement)
DEFINE_MEASUREMENT_HASH(RosPointCloudMapSensorMeasurement)
DEFINE_MEASUREMENT_HASH(OusterPointCloudMapSensorMeasurement)

#endif  // SENSORS_POINTCLOUD_MAP_SENSOR_H_
