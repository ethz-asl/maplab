#include "sensors/sensor-system.h"

#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>

#include "sensors/sensor-factory.h"

namespace vi_map {

constexpr char kYamlFieldNameSensorSysteId[] = "sensor_system_id";
constexpr char kYamlFieldNameReferenceSensorId[] = "reference_sensor_id";
constexpr char kYamlFieldNameSensorId[] = "sensor_id";
constexpr char kYamlFieldNameSensorIdWithExtrinsics[] =
    "sensor_id_with_extrinsics";
constexpr char kYamlFieldNameExtrinsics[] = "extrinsics";

SensorSystem::SensorSystem() {
  common::generateId(&id_);
  CHECK(id_.isValid());
}

SensorSystem::SensorSystem(const SensorId& reference_sensor_id)
    : reference_sensor_id_(reference_sensor_id) {
  CHECK(reference_sensor_id_.isValid());
  common::generateId(&id_);
  CHECK(id_.isValid());
}

SensorSystem::UniquePtr SensorSystem::createFromYaml(
    const std::string& yaml_filepath) {
  SensorSystem::UniquePtr sensor_system(new SensorSystem);
  if (!sensor_system->deserializeFromFile(yaml_filepath)) {
    sensor_system.reset();
  }
  return sensor_system;
}

SensorSystem::UniquePtr SensorSystem::createFromYaml(
    const YAML::Node& yaml_node) {
  SensorSystem::UniquePtr sensor_system(new SensorSystem);
  if (!sensor_system->deserialize(yaml_node)) {
    sensor_system.reset();
  }
  return sensor_system;
}

void SensorSystem::setSensorExtrinsics(
    const SensorId& sensor_id, const Extrinsics& extrinsics) {
  CHECK(sensor_id.isValid());
  sensor_id_to_extrinsics_map_.emplace(sensor_id, extrinsics);
}

const Extrinsics& SensorSystem::getSensorExtrinsics(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  ExtrinsicsMap::const_iterator extrinsics_iterator =
      sensor_id_to_extrinsics_map_.find(sensor_id);
  CHECK(extrinsics_iterator != sensor_id_to_extrinsics_map_.end());
  return extrinsics_iterator->second;
}

ExtrinsicsType SensorSystem::getSensorExtrinsicsType(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return getSensorExtrinsics(sensor_id).getExtrinsicsType();
}

const aslam::Transformation& SensorSystem::getSensor_T_R_S(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  const Extrinsics& extrinsics =
      common::getChecked(sensor_id_to_extrinsics_map_, sensor_id);
  CHECK(extrinsics.getExtrinsicsType() != ExtrinsicsType::kInvalid);
  return extrinsics.get_T_R_S();
}

const aslam::Position3D& SensorSystem::getSensor_p_R_S(
    const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  const Extrinsics& extrinsics =
      common::getChecked(sensor_id_to_extrinsics_map_, sensor_id);
  CHECK(extrinsics.getExtrinsicsType() != ExtrinsicsType::kInvalid);
  return extrinsics.get_p_R_S();
}

void SensorSystem::getAllSensorIds(SensorIdSet* sensor_ids) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  CHECK(reference_sensor_id_.isValid());
  sensor_ids->emplace(reference_sensor_id_);
  for (const ExtrinsicsMap::value_type& sensor_id_with_extrinsics :
       sensor_id_to_extrinsics_map_) {
    CHECK(sensor_id_with_extrinsics.first.isValid());
    sensor_ids->emplace(sensor_id_with_extrinsics.first);
  }
}

bool SensorSystem::hasSensor(const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return reference_sensor_id_ == sensor_id ||
         sensor_id_to_extrinsics_map_.count(sensor_id) > 0u;
}

void SensorSystem::serialize(YAML::Node* yaml_node_ptr) const {
  YAML::Node& sensor_node = *CHECK_NOTNULL(yaml_node_ptr);

  CHECK(id_.isValid());
  sensor_node[static_cast<std::string>(kYamlFieldNameSensorSysteId)] =
      id_.hexString();

  CHECK(reference_sensor_id_.isValid());
  sensor_node[static_cast<std::string>(kYamlFieldNameReferenceSensorId)] =
      reference_sensor_id_.hexString();

  YAML::Node sensors_extrinsics_node(YAML::NodeType::Sequence);
  for (const ExtrinsicsMap::value_type& sensor_with_extrinsics :
       sensor_id_to_extrinsics_map_) {
    YAML::Node sensor_extrinsics_node;
    const SensorId& sensor_id = sensor_with_extrinsics.first;
    CHECK(sensor_id.isValid());
    sensor_extrinsics_node[static_cast<std::string>(kYamlFieldNameSensorId)] =
        sensor_id.hexString();
    YAML::Node extrinsics_node;
    sensor_with_extrinsics.second.serialize(&extrinsics_node);
    sensor_extrinsics_node[static_cast<std::string>(kYamlFieldNameExtrinsics)] =
        extrinsics_node;
    sensors_extrinsics_node.push_back(sensor_extrinsics_node);
  }
  sensor_node[static_cast<std::string>(kYamlFieldNameSensorIdWithExtrinsics)] =
      sensors_extrinsics_node;
}

bool SensorSystem::deserialize(const YAML::Node& sensor_system_node) {
  CHECK(!sensor_system_node.IsNull());
  std::string id_as_string;
  if (YAML::safeGet(
          sensor_system_node,
          static_cast<std::string>(kYamlFieldNameSensorSysteId),
          &id_as_string)) {
    CHECK(!id_as_string.empty());
    CHECK(id_.fromHexString(id_as_string));
  } else {
    LOG(WARNING) << "Unable to find an ID field. Generating a new random id.";
    common::generateId(&id_);
  }
  CHECK(id_.isValid());

  std::string reference_sensor_id_as_string;
  if (YAML::safeGet(
          sensor_system_node,
          static_cast<std::string>(kYamlFieldNameReferenceSensorId),
          &reference_sensor_id_as_string)) {
    CHECK(!reference_sensor_id_as_string.empty());
    CHECK(reference_sensor_id_.fromHexString(reference_sensor_id_as_string));
  } else {
    LOG(ERROR) << "Unable to find the reference sensor ID.";
    return false;
  }
  CHECK(reference_sensor_id_.isValid());

  const YAML::Node& sensors_extrinsics_node =
      sensor_system_node[static_cast<std::string>(
          kYamlFieldNameSensorIdWithExtrinsics)];

  if (!sensors_extrinsics_node.IsSequence()) {
    LOG(ERROR) << "Unable to parse the sensor extrinsics because the"
               << " extrinsics node is not a sequence.";
    return false;
  }

  for (const YAML::Node& sensor_extrinsics_node : sensors_extrinsics_node) {
    CHECK(sensor_extrinsics_node.IsMap());
    SensorId sensor_id;
    std::string sensor_id_as_string;
    if (YAML::safeGet(
            sensor_extrinsics_node,
            static_cast<std::string>(kYamlFieldNameSensorId),
            &sensor_id_as_string)) {
      CHECK(!sensor_id_as_string.empty());
      CHECK(sensor_id.fromHexString(sensor_id_as_string));
    } else {
      LOG(ERROR) << "Unable to find the reference sensor ID.";
      return false;
    }
    CHECK(sensor_id.isValid());

    CHECK(sensor_extrinsics_node.IsMap());
    const YAML::Node& extrinsics_node =
        sensor_extrinsics_node[static_cast<std::string>(
            kYamlFieldNameExtrinsics)];

    CHECK(!extrinsics_node.IsNull());
    Extrinsics::UniquePtr extrinsics =
        Extrinsics::createFromYaml(extrinsics_node);
    CHECK(extrinsics);
    CHECK(sensor_id_to_extrinsics_map_.emplace(sensor_id, *extrinsics).second);
  }
  return true;
}

}  // namespace vi_map
