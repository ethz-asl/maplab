#include "sensors/gps-wgs.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {

template <>
SensorType sensorToType<GpsWgs>() {
  return SensorType::kGpsWgs;
}

template <>
SensorType measurementToSensorType<GpsWgsMeasurement>() {
  return SensorType::kGpsWgs;
}

constexpr char kDefaultGpsWgsHardwareId[] = "gps_wgs";

GpsWgs::GpsWgs()
    : Sensor(
          SensorType::kGpsWgs,
          static_cast<std::string>(kDefaultGpsWgsHardwareId)) {}

GpsWgs::GpsWgs(const SensorId& sensor_id, const std::string& hardware_id)
    : Sensor(sensor_id, SensorType::kGpsWgs, hardware_id) {}

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

void GpsWgsMeasurement::serialize(
    measurements::proto::GpsWgsMeasurement* proto_measurement) const {
  CHECK_NOTNULL(proto_measurement);
  Measurement::serialize(proto_measurement->mutable_measurement_base());
  proto_measurement->set_latitude_deg(latitude_deg_);
  proto_measurement->set_longitude_deg(longitude_deg_);
  proto_measurement->set_altitude_meters(altitude_meters_);
  CHECK(isValid());
}
void GpsWgsMeasurement::deserialize(
    const measurements::proto::GpsWgsMeasurement& proto_measurement) {
  Measurement::deserialize(proto_measurement.measurement_base());
  latitude_deg_ = proto_measurement.latitude_deg();
  longitude_deg_ = proto_measurement.longitude_deg();
  altitude_meters_ = proto_measurement.altitude_meters();
  CHECK(isValid());
}

}  // namespace vi_map
