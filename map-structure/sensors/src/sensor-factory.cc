#include "sensors/sensor-factory.h"

#include "sensors/gps-utm.h"
#include "sensors/gps-wgs.h"
#include "sensors/imu.h"
#include "sensors/relative-6dof-pose.h"

namespace vi_map {

Sensor::UniquePtr createSensorFromYaml(const YAML::Node& sensor_node) {
  std::string sensor_type_as_string;
  CHECK(sensor_node.IsDefined() && sensor_node.IsMap());
  if (!YAML::safeGet(sensor_node, "sensor_type", &sensor_type_as_string)) {
    LOG(FATAL) << "Unable to retrieve the sensor type from the given "
               << "YAML node.";
  }

  const SensorType sensor_type = stringToSensorType(sensor_type_as_string);

  switch (sensor_type) {
    case SensorType::kImu:
      return sensors::internal::createFromYaml<Imu>(sensor_node);
      break;
    case SensorType::kRelative6DoFPose:
      return sensors::internal::createFromYaml<Relative6DoFPose>(sensor_node);
      break;
    case SensorType::kGpsUtm:
      return sensors::internal::createFromYaml<GpsUtm>(sensor_node);
      break;
    case SensorType::kGpsWgs:
      return sensors::internal::createFromYaml<GpsWgs>(sensor_node);
      break;
    default:
      LOG(ERROR) << "Unknown sensor type: " << static_cast<int>(sensor_type);
      break;
  }

  return nullptr;
}

Sensor::UniquePtr createSensorFromYaml(const std::string& yaml_filepath) {
  YAML::Node sensor_node;
  try {
    sensor_node = YAML::LoadFile(yaml_filepath.c_str());
  } catch (const std::exception& ex) {  // NOLINT
    LOG(ERROR) << "Failed to open and parse the sensor YAML file "
               << yaml_filepath << " with the error: " << ex.what();
    return nullptr;
  }

  return createSensorFromYaml(sensor_node);
}

Sensor::UniquePtr createTestSensor(const SensorType sensor_type) {
  switch (sensor_type) {
    case SensorType::kImu:
      return createTestSensor<Imu>();
      break;
    case SensorType::kRelative6DoFPose:
      return createTestSensor<Relative6DoFPose>();
      break;
    case SensorType::kGpsUtm:
      return createTestSensor<GpsUtm>();
      break;
    case SensorType::kGpsWgs:
      return createTestSensor<GpsWgs>();
      break;
    default:
      LOG(ERROR) << "Unknown sensor type: " << static_cast<int>(sensor_type);
      break;
  }
  return nullptr;
}

Sensor::UniquePtr createRandomTestSensor() {
  constexpr int kNumSensorTypes = static_cast<int>(SensorType::kInvalidSensor);
  CHECK_GT(kNumSensorTypes, 0);

  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_int_distribution<int> sensor_type_distribution(
      0, kNumSensorTypes - 1);
  const int sensor_type_int = sensor_type_distribution(random_engine);
  CHECK_GE(sensor_type_int, 0);
  CHECK_LT(sensor_type_int, kNumSensorTypes);

  const SensorType sensor_type = static_cast<SensorType>(sensor_type_int);

  return createTestSensor(sensor_type);
}

}  // namespace vi_map
