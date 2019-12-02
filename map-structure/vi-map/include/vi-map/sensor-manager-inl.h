#ifndef VI_MAP_SENSOR_MANAGER_INL_H_
#define VI_MAP_SENSOR_MANAGER_INL_H_

#include <memory>
#include <utility>

namespace vi_map {

template <typename DerivedSensor>
void SensorManager::addSensor(
    typename DerivedSensor::UniquePtr sensor,
    const aslam::SensorId& base_sensor_id, const aslam::Transformation& T_B_S) {
  CHECK(sensor);
  typename DerivedSensor::Ptr sensor_shared = std::move(sensor);
  addSensor<DerivedSensor>(sensor_shared, base_sensor_id, T_B_S);
}

template <typename DerivedSensor>
void SensorManager::addSensor(
    const typename DerivedSensor::Ptr& sensor,
    const aslam::SensorId& base_sensor_id, const aslam::Transformation& T_B_S) {
  CHECK(sensor);
  const aslam::SensorId& sensor_id = sensor->getId();
  CHECK(sensor_id.isValid());

  CHECK(
      sensors_
          .emplace(sensor_id, std::dynamic_pointer_cast<aslam::Sensor>(sensor))
          .second)
      << "There already exists a sensor with the ID '" << sensor_id
      << "' in the SensorManger!";

  CHECK(hasSensor(base_sensor_id))
      << "Base sensor " << base_sensor_id << " of sensor " << sensor_id
      << " does not exist in the sensor manager.";
  CHECK(base_sensor_id_map_.emplace(sensor_id, base_sensor_id).second);
  CHECK(T_B_S_map_.emplace(sensor_id, T_B_S).second);
}

template <typename DerivedSensor>
void SensorManager::addSensor(
    typename DerivedSensor::UniquePtr sensor,
    const aslam::SensorId& base_sensor_id, const aslam::Position3D& p_B_S) {
  aslam::Transformation T_B_S;
  T_B_S.setIdentity();
  T_B_S.getPosition() = p_B_S;

  addSensor<DerivedSensor>(std::move(sensor), base_sensor_id, T_B_S);
}

template <typename DerivedSensor>
void SensorManager::addSensorAsBase(typename DerivedSensor::UniquePtr sensor) {
  CHECK(sensor);
  typename DerivedSensor::Ptr sensor_shared = std::move(sensor);
  addSensorAsBase<DerivedSensor>(sensor_shared);
}

template <typename DerivedSensor>
void SensorManager::addSensorAsBase(typename DerivedSensor::Ptr sensor) {
  CHECK(sensor);
  const aslam::SensorId& sensor_id = sensor->getId();
  CHECK(sensor_id.isValid());

  aslam::Transformation T_B_S;
  T_B_S.setIdentity();

  addSensor<DerivedSensor>(sensor, sensor_id, T_B_S);
}

template <typename DerivedSensor>
const DerivedSensor& SensorManager::getSensor(
    const aslam::SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());

  return dynamic_cast<const DerivedSensor&>(
      *CHECK_NOTNULL(common::getChecked(sensors_, sensor_id).get()));
}

template <typename DerivedSensor>
typename DerivedSensor::Ptr SensorManager::getSensorPtr(
    const aslam::SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());

  aslam::Sensor::Ptr sensor = common::getChecked(sensors_, sensor_id);
  CHECK_NOTNULL(sensor.get());
  return std::dynamic_pointer_cast<DerivedSensor>(sensor);
}

}  // namespace vi_map

#endif  // VI_MAP_SENSOR_MANAGER_INL_H_
