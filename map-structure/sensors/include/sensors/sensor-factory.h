#ifndef SENSORS_SENSOR_FACTORY_H_
#define SENSORS_SENSOR_FACTORY_H_

#include <string>

#include <glog/logging.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/sensor.h"

namespace vi_map {

template <class DerivedSensor>
typename DerivedSensor::UniquePtr createTestSensor() {
  ASSERT_DERIVED(DerivedSensor, Sensor);
  typename DerivedSensor::UniquePtr sensor(new DerivedSensor);
  CHECK(sensor->getSensorType() == sensorToType<DerivedSensor>());
  sensor->setRandom();
  CHECK(sensor->isValid());
  return sensor;
}

template <class DerivedSensor>
typename DerivedSensor::UniquePtr createFromYaml(
    const std::string& yaml_filepath) {
  ASSERT_DERIVED(DerivedSensor, Sensor);
  typename DerivedSensor::UniquePtr sensor(new DerivedSensor);
  if (!sensor->deserializeFromFile(yaml_filepath)) {
    return nullptr;
  }
  CHECK(sensor->isValid());
  return sensor;
}

namespace sensors {
namespace internal {
template <class DerivedSensor>
typename DerivedSensor::UniquePtr createFromYaml(const YAML::Node& yaml_node) {
  ASSERT_DERIVED(DerivedSensor, Sensor);
  typename DerivedSensor::UniquePtr sensor(new DerivedSensor);
  if (!sensor->deserialize(yaml_node)) {
    return nullptr;
  }
  CHECK(sensor->isValid());
  return sensor;
}
}  // namespace internal
}  // namespace sensors

Sensor::UniquePtr createSensorFromYaml(const YAML::Node& sensor_node);
Sensor::UniquePtr createSensorFromYaml(const std::string& yaml_filepath);

Sensor::UniquePtr createTestSensor(const SensorType sensor_type);
Sensor::UniquePtr createRandomTestSensor();

}  // namespace vi_map

#endif  // SENSORS_SENSOR_FACTORY_H_
