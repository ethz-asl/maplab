#include "sensors/gps-utm.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {

template <>
SensorType sensorToType<GpsUtm>() {
  return SensorType::kGpsUtm;
}

template <>
SensorType measurementToSensorType<GpsUtmMeasurement>() {
  return SensorType::kGpsUtm;
}

constexpr char kDefaultGpsUtmHardwareId[] = "gps_utm";

GpsUtm::GpsUtm()
    : Sensor(
          SensorType::kGpsUtm,
          static_cast<std::string>(kDefaultGpsUtmHardwareId)) {}

GpsUtm::GpsUtm(const SensorId& sensor_id, const std::string& hardware_id)
    : Sensor(sensor_id, SensorType::kGpsUtm, hardware_id) {}

bool GpsUtm::loadFromYamlNodeImpl(const YAML::Node& /*sensor_node*/) {
  return true;
}

void GpsUtm::saveToYamlNodeImpl(YAML::Node* /*sensor_node*/) const {}

void GpsUtmMeasurement::serialize(
    measurements::proto::GpsUtmMeasurement* proto_measurement) const {
  CHECK_NOTNULL(proto_measurement);
  Measurement::serialize(proto_measurement->mutable_measurement_base());
  common::eigen_proto::serialize(
      T_UTM_S_, proto_measurement->mutable_t_utm_s());
}
void GpsUtmMeasurement::deserialize(
    const measurements::proto::GpsUtmMeasurement& proto_measurement) {
  Measurement::deserialize(proto_measurement.measurement_base());
  common::eigen_proto::deserialize(proto_measurement.t_utm_s(), &T_UTM_S_);
}

}  // namespace vi_map
