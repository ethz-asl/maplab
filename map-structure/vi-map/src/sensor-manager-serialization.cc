#include "vi-map/sensor-manager.h"

#include <aslam-serialization/camera-serialization.h>
#include <sensors/sensor-factory.h>

namespace vi_map {

constexpr char kYamlFieldNameSensors[] = "sensors";
constexpr char kYamlFieldNameNCameras[] = "ncameras";
constexpr char kYamlFieldNameSensorSystem[] = "sensor_system";
constexpr char kYamlFieldNameMissionSensorsAssociation[] =
    "mission_sensors_associations";
constexpr char kYamlFieldNameMissionNCameraAssociation[] =
    "mission_ncamera_associations";
constexpr char kYamlFieldNameSensorId[] = "sensor_id";
constexpr char kYamlFieldNameMissionId[] = "mission_id";
constexpr char kYamlFieldNameNCameraId[] = "ncamera_id";
constexpr char kYamlFieldNameSensorIds[] = "sensor_ids";

void SensorManager::serialize(YAML::Node* yaml_node) const {
  CHECK_NOTNULL(yaml_node);

  YAML::Node sensors_node;
  for (const AlignedUnorderedMap<SensorId, Sensor::UniquePtr>::value_type&
           sensor_with_id : sensors_) {
    CHECK(sensor_with_id.first.isValid());
    CHECK(sensor_with_id.second);
    CHECK_EQ(sensor_with_id.first, sensor_with_id.second->getId());
    YAML::Node sensor_node;
    sensor_with_id.second->serialize(&sensor_node);

    sensors_node.push_back(sensor_node);
  }
  (*yaml_node)[static_cast<std::string>(kYamlFieldNameSensors)] = sensors_node;

  YAML::Node ncameras_nodes;
  for (const std::pair<aslam::NCameraId, aslam::NCamera::Ptr>&
           ncameras_with_id : ncameras_) {
    CHECK(ncameras_with_id.first.isValid());
    CHECK(ncameras_with_id.second);
    CHECK_EQ(ncameras_with_id.first, ncameras_with_id.second->getId());

    YAML::Node ncamera_node;
    ncameras_with_id.second->serializeToYaml(&ncamera_node);

    ncameras_nodes.push_back(ncamera_node);
  }
  (*yaml_node)[static_cast<std::string>(kYamlFieldNameNCameras)] =
      ncameras_nodes;

  YAML::Node missions_sensors_node;
  for (const AlignedUnorderedMap<MissionId, SensorIdSet>::value_type&
           mission_id_sensor_ids : mission_id_to_sensors_map_) {
    const MissionId& mission_id = mission_id_sensor_ids.first;
    CHECK(mission_id.isValid());

    YAML::Node mission_node;
    mission_node[static_cast<std::string>(kYamlFieldNameMissionId)] =
        mission_id.hexString();

    YAML::Node mission_sensor_ids_node;
    for (const SensorId& sensor_id : mission_id_sensor_ids.second) {
      CHECK(sensor_id.isValid());
      YAML::Node mission_sensor_id_node;
      mission_sensor_id_node[kYamlFieldNameSensorId] = sensor_id.hexString();
      mission_sensor_ids_node.push_back(mission_sensor_id_node);
    }
    mission_node[static_cast<std::string>(kYamlFieldNameSensorIds)] =
        mission_sensor_ids_node;

    missions_sensors_node.push_back(mission_node);
  }
  (*yaml_node)[static_cast<std::string>(
      kYamlFieldNameMissionSensorsAssociation)] = missions_sensors_node;

  YAML::Node missions_ncameras_node;
  for (const AlignedUnorderedMap<MissionId, aslam::NCameraId>::value_type&
           mission_id_ncamera_id : mission_id_to_ncamera_map_) {
    const MissionId& mission_id = mission_id_ncamera_id.first;
    CHECK(mission_id.isValid());

    YAML::Node mission_node;
    mission_node[static_cast<std::string>(kYamlFieldNameMissionId)] =
        mission_id.hexString();

    const aslam::NCameraId& ncamera_id = mission_id_ncamera_id.second;
    CHECK(ncamera_id.isValid());
    mission_node[static_cast<std::string>(kYamlFieldNameNCameraId)] =
        ncamera_id.hexString();

    missions_ncameras_node.push_back(mission_node);
  }
  (*yaml_node)[static_cast<std::string>(
      kYamlFieldNameMissionNCameraAssociation)] = missions_ncameras_node;

  if (sensor_system_) {
    CHECK(sensor_system_->getId().isValid());
    YAML::Node sensor_system_node;
    sensor_system_->serialize(&sensor_system_node);
    (*yaml_node)[static_cast<std::string>(kYamlFieldNameSensorSystem)] =
        sensor_system_node;
  }
}

/*
void SensorManager::deserialize(const std::string& sensors_yaml_filepath) {
  CHECK(common::fileExists(sensors_yaml_filepath))
      << "Sensors YAML file " << sensors_yaml_filepath << " does not exist.";

  YAML::Node sensors_with_systems_node;
  try {
    sensors_with_systems_node = YAML::LoadFile(sensors_yaml_filepath.c_str());
  } catch (const std::exception& ex) {  // NOLINT
    LOG(FATAL) << "Failed to load sensors from file " << sensors_yaml_filepath
               << " with the error: " << ex.what();
    return;
  }

  deserialize(sensors_with_systems_node);
}*/

bool SensorManager::deserialize(const YAML::Node& sensors_with_systems_node) {
  checkIsConsistent();
  CHECK(sensors_with_systems_node.IsDefined());
  CHECK(sensors_with_systems_node.IsMap());

  const YAML::Node ncameras_node =
      sensors_with_systems_node[static_cast<std::string>(
          kYamlFieldNameNCameras)];

  if (ncameras_node.IsDefined() && !ncameras_node.IsNull()) {
    CHECK(ncameras_node.IsSequence());
    for (const YAML::Node& ncamera_node : ncameras_node) {
      aslam::NCamera::Ptr ncamera =
          aslam::NCamera::deserializeFromYaml(ncamera_node);
      CHECK(ncamera);
      const aslam::NCameraId& ncamera_id = ncamera->getId();
      CHECK(ncamera_id.isValid());
      CHECK(ncameras_.emplace(ncamera_id, ncamera).second);
    }
  }

  const YAML::Node sensors_node =
      sensors_with_systems_node[static_cast<std::string>(
          kYamlFieldNameSensors)];
  if (sensors_node.IsDefined() && !sensors_node.IsNull()) {
    CHECK(sensors_node.IsSequence());
    for (const YAML::Node& sensor_node : sensors_node) {
      CHECK(!sensor_node.IsNull());
      Sensor::UniquePtr sensor = createSensorFromYaml(sensor_node);
      CHECK(sensor);
      const SensorId sensor_id = sensor->getId();
      CHECK(sensor_id.isValid());
      CHECK(sensors_.emplace(sensor_id, std::move(sensor)).second);
    }
  }

  const YAML::Node missions_sensors_node =
      sensors_with_systems_node[static_cast<std::string>(
          kYamlFieldNameMissionSensorsAssociation)];
  if (missions_sensors_node.IsDefined() && !missions_sensors_node.IsNull()) {
    CHECK(missions_sensors_node.IsSequence());
    for (const YAML::Node& mission_with_sensor_ids_node :
         missions_sensors_node) {
      std::string mission_id_string;
      if (!YAML::safeGet(
              mission_with_sensor_ids_node,
              static_cast<std::string>(kYamlFieldNameMissionId),
              &mission_id_string)) {
        LOG(FATAL) << "Unable to parse the mission field.";
      }
      CHECK(!mission_id_string.empty());
      MissionId mission_id;
      mission_id.fromHexString(mission_id_string);
      CHECK(mission_id.isValid());

      const YAML::Node& mission_sensor_ids_node =
          mission_with_sensor_ids_node[static_cast<std::string>(
              kYamlFieldNameSensorIds)];
      CHECK(!mission_sensor_ids_node.IsNull());
      SensorIdSet sensor_ids;
      CHECK(mission_sensor_ids_node.IsSequence());
      for (const YAML::Node& sensor_id_node : mission_sensor_ids_node) {
        CHECK(!sensor_id_node.IsNull());
        std::string sensor_id_string;
        if (!YAML::safeGet(
                sensor_id_node,
                static_cast<std::string>(kYamlFieldNameSensorId),
                &sensor_id_string)) {
          LOG(FATAL) << "Unable to parse the sensor id field.";
        }
        CHECK(!sensor_id_string.empty());
        SensorId sensor_id;
        sensor_id.fromHexString(sensor_id_string);
        CHECK(sensor_id.isValid());
        CHECK(sensor_ids.emplace(sensor_id).second);
      }

      CHECK(mission_id_to_sensors_map_.emplace(mission_id, sensor_ids).second);
    }
  }

  const YAML::Node missions_ncamera_node =
      sensors_with_systems_node[static_cast<std::string>(
          kYamlFieldNameMissionNCameraAssociation)];
  if (missions_ncamera_node.IsDefined() && !missions_ncamera_node.IsNull()) {
    CHECK(missions_ncamera_node.IsSequence());
    for (const YAML::Node& mission_with_ncamea_id_node :
         missions_ncamera_node) {
      std::string mission_id_string;
      if (!YAML::safeGet(
              mission_with_ncamea_id_node,
              static_cast<std::string>(kYamlFieldNameMissionId),
              &mission_id_string)) {
        LOG(FATAL) << "Unable to parse the mission field.";
      }
      CHECK(!mission_id_string.empty());
      MissionId mission_id;
      mission_id.fromHexString(mission_id_string);
      CHECK(mission_id.isValid());

      std::string ncamera_id_string;
      if (!YAML::safeGet(
              mission_with_ncamea_id_node,
              static_cast<std::string>(kYamlFieldNameNCameraId),
              &ncamera_id_string)) {
        LOG(FATAL) << "Unable to parse the ncamera id field.";
      }
      CHECK(!ncamera_id_string.empty());
      aslam::NCameraId ncamera_id;
      ncamera_id.fromHexString(ncamera_id_string);
      CHECK(ncamera_id.isValid());

      CHECK(mission_id_to_ncamera_map_.emplace(mission_id, ncamera_id).second);
    }
  }

  const YAML::Node sensor_system_node =
      sensors_with_systems_node[static_cast<std::string>(
          kYamlFieldNameSensorSystem)];
  if (sensor_system_node.IsDefined() && !sensor_system_node.IsNull()) {
    SensorSystem::UniquePtr sensor_system =
        SensorSystem::createFromYaml(sensor_system_node);

    CHECK(sensor_system);
    const SensorSystemId& sensor_system_id = sensor_system->getId();
    CHECK(sensor_system_id.isValid());
    sensor_system_ = std::move(sensor_system);
  }

  checkIsConsistent();
  return true;
}
}  // namespace vi_map
