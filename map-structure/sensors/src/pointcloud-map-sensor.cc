#include "sensors/pointcloud-map-sensor.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {

constexpr char kDefaultPointCloudMapSensorTopic[] = "/submap";

PointCloudMapSensor::PointCloudMapSensor(const aslam::SensorId& sensor_id)
    : PointCloudMapSensor(
          sensor_id,
          static_cast<std::string>(kDefaultPointCloudMapSensorTopic)) {}

PointCloudMapSensor::PointCloudMapSensor(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {}

bool PointCloudMapSensor::loadFromYamlNodeImpl(
    const YAML::Node& /*sensor_node*/) {
  // Nothing todo, since the sensor does not have any additional members other
  // than the inherited ones.
  return true;
}

void PointCloudMapSensor::saveToYamlNodeImpl(
    YAML::Node* /*sensor_node*/) const {
  // Nothing todo, since the sensor does not have any additional members other
  // than the inherited ones.
}

}  // namespace vi_map
