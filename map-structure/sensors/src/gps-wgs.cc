#include "sensors/gps-wgs.h"

namespace vi_map {

GpsWgs::GpsWgs() : Sensor() {}

GpsWgs::GpsWgs(const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {}

bool GpsWgs::loadFromYamlNodeImpl(const YAML::Node& /*sensor_node*/) {
  return true;
}

void GpsWgs::saveToYamlNodeImpl(YAML::Node* /*sensor_node*/) const {}

void GpsWgsMeasurement::setRandomImpl() {
  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_real_distribution<double> longitude_distribution(
      kMinLongitudeDeg, kMaxLongitudeDeg);
  longitude_deg_ = longitude_distribution(random_engine);

  std::uniform_real_distribution<double> latitude_distribution(
      kMinLatitudeDeg, kMaxLatitudeDeg);
  latitude_deg_ = latitude_distribution(random_engine);

  std::uniform_real_distribution<double> altitude_distribution(
      kMinAltitudeMeters, kMaxAltitudeMeters);
  altitude_meters_ = altitude_distribution(random_engine);
}

}  // namespace vi_map
