#include "sensors/sensor.h"

#include <fstream>  // NOLINT

#include <maplab-common/eigen-proto.h>

namespace vi_map {

constexpr char kYamlFieldNameId[] = "id";
constexpr char kYamlFieldNameSensorType[] = "sensor_type";
constexpr char kYamlFieldNameHardwareId[] = "hardware_id";

constexpr char kDefaultHardwareId[] = "hardware_id_not_set";

Sensor::Sensor()
    : sensor_type_(SensorType::kInvalidSensor),
      hardware_id_(static_cast<std::string>(kDefaultHardwareId)) {}

Sensor::Sensor(const SensorType sensor_type)
    : sensor_type_(sensor_type), hardware_id_("") {
  CHECK(sensor_type != SensorType::kInvalidSensor);
}

Sensor::Sensor(const SensorType sensor_type, const std::string& hardware_id)
    : sensor_type_(sensor_type), hardware_id_(hardware_id) {
  common::generateId(&id_);
  CHECK(!hardware_id_.empty()) << "A sensor needs a non-empty hardware "
                               << "identification label.";
}

Sensor::Sensor(
    const SensorId& sensor_id, SensorType sensor_type,
    const std::string& hardware_id)
    : id_(sensor_id), sensor_type_(sensor_type), hardware_id_(hardware_id) {
  CHECK(id_.isValid());
  CHECK(!hardware_id_.empty()) << "A sensor needs a non-empty hardware "
                               << "identification label.";
}

bool Sensor::deserialize(const YAML::Node& sensor_node) {
  CHECK(!sensor_node.IsNull());
  std::string id_as_string;
  if (YAML::safeGet(
          sensor_node, static_cast<std::string>(kYamlFieldNameId),
          &id_as_string)) {
    CHECK(!id_as_string.empty());
    CHECK(id_.fromHexString(id_as_string));
  } else {
    LOG(WARNING) << "Unable to find an ID field. Generating a new random id.";
    common::generateId(&id_);
  }
  CHECK(id_.isValid());

  std::string sensor_type_as_string;
  if (!YAML::safeGet(
          sensor_node, static_cast<std::string>(kYamlFieldNameSensorType),
          &sensor_type_as_string)) {
    LOG(ERROR) << "Unable to retrieve the sensor type from the given "
               << "YAML node.";
    return false;
  }
  sensor_type_ = stringToSensorType(sensor_type_as_string);

  if (!YAML::safeGet(
          sensor_node, static_cast<std::string>(kYamlFieldNameHardwareId),
          &hardware_id_)) {
    LOG(ERROR) << "Unable to retrieve the sensor hardware id from the given "
               << "YAML node.";
    return false;
  }
  CHECK(!hardware_id_.empty())
      << "A sensor needs a non-empty hardware "
      << "identification label with YAML key \"hardware_id\".";

  sensor_type_ = stringToSensorType(sensor_type_as_string);

  return loadFromYamlNodeImpl(sensor_node);
}

void Sensor::serialize(YAML::Node* sensor_node_ptr) const {
  YAML::Node& sensor_node = *CHECK_NOTNULL(sensor_node_ptr);

  CHECK(id_.isValid());
  sensor_node[static_cast<std::string>(kYamlFieldNameId)] = id_.hexString();
  sensor_node[static_cast<std::string>(kYamlFieldNameSensorType)] =
      sensorTypeToString(sensor_type_);
  CHECK(!hardware_id_.empty());
  sensor_node[static_cast<std::string>(kYamlFieldNameHardwareId)] =
      hardware_id_;

  saveToYamlNodeImpl(&sensor_node);
}

}  // namespace vi_map
