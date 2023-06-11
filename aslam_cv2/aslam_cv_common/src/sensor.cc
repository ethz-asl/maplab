#include <aslam/common/sensor.h>

namespace aslam {

Sensor::Sensor() : topic_(""), description_("") {
  generateId(&id_);
}

Sensor::Sensor(const SensorId& id) : id_(id), topic_(""), description_("") {
  CHECK(id.isValid());
}

Sensor::Sensor(const SensorId& id, const std::string& topic)
    : id_(id), topic_(topic), description_("") {
  CHECK(id.isValid());
}

Sensor::Sensor(
    const SensorId& id, const std::string& topic,
    const std::string& description)
    : id_(id), topic_(topic), description_(description) {
  CHECK(id.isValid());
}

bool Sensor::isValid() const {
  if (!id_.isValid()) {
    LOG(ERROR) << "Invalid sensor id.";
    return false;
  }
  return isValidImpl();
}

void Sensor::setRandom() {
  generateId(&id_);
  setRandomImpl();
}

bool Sensor::deserialize(const YAML::Node& sensor_node) {
  if (!sensor_node.IsDefined() || sensor_node.IsNull()) {
    LOG(ERROR) << "Invalid YAML node for sensor deserialization.";
    return false;
  }

  if (!sensor_node.IsMap()) {
    LOG(WARNING) << "Sensor YAML node must be a map.";
    return false;
  }

  std::string id_as_string;
  if (YAML::safeGet(
          sensor_node, static_cast<std::string>(kYamlFieldNameId),
          &id_as_string)) {
    CHECK(!id_as_string.empty());
    CHECK(id_.fromHexString(id_as_string));
  } else {
    LOG(WARNING) << "Unable to find an ID field. Generating a new random id.";
    generateId(&id_);
  }
  CHECK(id_.isValid());

  if (!YAML::safeGet(
          sensor_node, static_cast<std::string>(kYamlFieldNameTopic),
          &topic_)) {
    LOG(WARNING) << "Unable to retrieve the sensor topic for sensor " << id_;
  }

  if (!YAML::safeGet(
          sensor_node, static_cast<std::string>(kYamlFieldNameDescription),
          &description_)) {
    LOG(WARNING) << "Unable to retrieve the sensor description.";
  }

  return loadFromYamlNodeImpl(sensor_node);
}

void Sensor::serialize(YAML::Node* sensor_node_ptr) const {
  YAML::Node& sensor_node = *CHECK_NOTNULL(sensor_node_ptr);

  CHECK(id_.isValid());
  sensor_node[static_cast<std::string>(kYamlFieldNameId)] = id_.hexString();
  sensor_node[static_cast<std::string>(kYamlFieldNameSensorType)] =
      getSensorTypeString();
  sensor_node[static_cast<std::string>(kYamlFieldNameTopic)] = topic_;
  sensor_node[static_cast<std::string>(kYamlFieldNameDescription)] =
      description_;

  saveToYamlNodeImpl(&sensor_node);
}

bool Sensor::operator==(const Sensor& other) const {
  return isEqual(other, true /*verbose*/);
}

bool Sensor::operator!=(const Sensor& other) const {
  return !isEqual(other, true /*verbose*/);
}

bool Sensor::isEqual(const Sensor& other, const bool verbose) const {
  bool is_equal = true;
  is_equal &= id_ == other.id_;
  is_equal &= topic_ == other.topic_;
  is_equal &= description_ == other.description_;
  is_equal &= getSensorType() == other.getSensorType();

  if (!is_equal) {
    LOG_IF(WARNING, verbose) << "this sensor: "
                             << "\n id: " << id_ << "\n topic: " << topic_
                             << "\n description: " << description_
                             << "\n sensor_type: " << getSensorTypeString();
    LOG_IF(WARNING, verbose)
        << "other sensor: "
        << "\n id: " << id_ << "\n topic: " << other.topic_
        << "\n description: " << other.description_
        << "\n sensor_type: " << other.getSensorTypeString();
  }

  // optimize to avoid unncessary comparisons
  if (is_equal) {
    is_equal &= isEqualImpl(other, verbose);
  }

  return is_equal;
}

}  // namespace aslam
