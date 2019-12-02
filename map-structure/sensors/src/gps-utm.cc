#include "sensors/gps-utm.h"

namespace vi_map {

GpsUtm::GpsUtm() : Sensor() {}

GpsUtm::GpsUtm(const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {}

bool GpsUtm::loadFromYamlNodeImpl(const YAML::Node& /*sensor_node*/) {
  return true;
}

void GpsUtm::saveToYamlNodeImpl(YAML::Node* /*sensor_node*/) const {}

}  // namespace vi_map
