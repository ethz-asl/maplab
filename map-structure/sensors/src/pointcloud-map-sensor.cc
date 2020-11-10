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

bool PointCloudMapSensor::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (sensor_node[kYamlFieldNameNumberOfBeams]) {
    CHECK(YAML::safeGet(
        sensor_node, static_cast<std::string>(kYamlFieldNameNumberOfBeams),
        &n_beams_));
    has_number_of_beams_ = true;
  } else {
    has_number_of_beams_ = false;
  }

  if (sensor_node[kYamlFieldNameFoVUpperAngle]) {
    CHECK(YAML::safeGet(
        sensor_node, static_cast<std::string>(kYamlFieldNameFoVUpperAngle),
        &fov_upper_angle_deg_));
    has_upper_fov_angle_ = true;
  } else {
    has_upper_fov_angle_ = false;
  }

  if (sensor_node[kYamlFieldNameFoVLowerAngle]) {
    CHECK(YAML::safeGet(
        sensor_node, static_cast<std::string>(kYamlFieldNameFoVLowerAngle),
        &fov_lower_angle_deg_));
    has_lower_fov_angle_ = true;
  } else {
    has_lower_fov_angle_ = false;
  }
  return true;
}

void PointCloudMapSensor::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  CHECK_NOTNULL(sensor_node);
  if (has_number_of_beams_) {
    (*sensor_node)[static_cast<std::string>(kYamlFieldNameNumberOfBeams)] =
        YAML::convert<uint16_t>::encode(n_beams_);
  }
  if (fov_upper_angle_deg_) {
    (*sensor_node)[static_cast<std::string>(kYamlFieldNameFoVUpperAngle)] =
        YAML::convert<uint16_t>::encode(fov_upper_angle_deg_);
  }
  if (fov_lower_angle_deg_) {
    (*sensor_node)[static_cast<std::string>(kYamlFieldNameFoVLowerAngle)] =
        YAML::convert<uint16_t>::encode(fov_lower_angle_deg_);
  }
}

}  // namespace vi_map
