#ifndef VI_MAP_SENSOR_MANAGER_INL_H_
#define VI_MAP_SENSOR_MANAGER_INL_H_

#include <type_traits>

namespace vi_map {

template <class DerivedSensor>
const DerivedSensor& SensorManager::getSensor(const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  AlignedUnorderedMap<SensorId, Sensor::UniquePtr>::const_iterator
  sensor_iterator = sensors_.find(sensor_id);
  CHECK(sensor_iterator != sensors_.end());
  CHECK(sensor_iterator->second);
  DerivedSensor* derived_sensor =
      dynamic_cast<DerivedSensor*>(sensor_iterator->second.get());
  CHECK_NOTNULL(derived_sensor);
  return *derived_sensor;
}

template <class DerivedSensor>
const DerivedSensor& SensorManager::getSensorForMission(
    const MissionId& mission_id) const {
  ASSERT_DERIVED(DerivedSensor, Sensor);
  CHECK(mission_id.isValid());
  SensorIdSet mission_sensors;
  getAllSensorIdsOfTypeAssociatedWithMission(
      sensorToType<DerivedSensor>(), mission_id, &mission_sensors);
  CHECK_EQ(mission_sensors.size(), 1u);
  return getSensor<DerivedSensor>(*mission_sensors.begin());
}

template <class DerivedSensor>
bool SensorManager::getSensor(
    const SensorId& sensor_id, DerivedSensor* sensor) const {
  CHECK_NOTNULL(sensor);
  CHECK(sensor_id.isValid());
  if (hasSensor(sensor_id)) {
    *sensor = getSensor<SensorType>(sensor_id);
    return true;
  }
  return false;
}

template <class DerivedSensor>
bool SensorManager::getSensor(DerivedSensor* const sensor) const {
  ASSERT_DERIVED(DerivedSensor, Sensor);
  CHECK_NOTNULL(sensor);
  SensorIdSet sensor_ids;
  const SensorType sensor_type = sensorToType<DerivedSensor>();
  getAllSensorIdsOfType(sensor_type, &sensor_ids);
  if (sensor_ids.empty()) {
    VLOG(3) << "Unable to retrieve sensor of type "
            << sensorTypeToString(sensor_type) << ", because there are zero "
            << "sensors of that type registered in the sensor manager.";
    return false;
  } else if (sensor_ids.size() > 1u) {
    VLOG(3) << "Unable to retrieve sensor of type "
            << sensorTypeToString(sensor_type)
            << ", because there is more than "
            << "one sensor of that type registered in the sensor manager.";
    return false;
  }
  *sensor = getSensor<DerivedSensor>(*sensor_ids.begin());
  return true;
}

template <class DerivedSensor>
size_t SensorManager::getNumSensorsOfTypeAssociatedWithMission(
    const MissionId& mission_id) const {
  ASSERT_DERIVED(DerivedSensor, Sensor);
  CHECK(mission_id.isValid());
  SensorIdSet sensor_ids;
  getAllSensorIdsOfTypeAssociatedWithMission(
      sensorToType<DerivedSensor>(), mission_id, &sensor_ids);
  return sensor_ids.size();
}

}  // namespace vi_map

#endif  // VI_MAP_SENSOR_MANAGER_INL_H_
