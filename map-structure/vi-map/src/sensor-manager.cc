#include "vi-map/sensor-manager.h"

#include <maplab-common/accessors.h>

namespace vi_map {
template<>
bool SensorManager::hasSensor<aslam::NCameraId>(
    const aslam::NCameraId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return ncameras_.count(sensor_id) > 0u;
}

template<>
bool SensorManager::hasSensor<SensorId>(const SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return sensors_.count(sensor_id) > 0u;
}

void SensorManager::addSensor(Sensor::UniquePtr sensor) {
  CHECK(sensor);
  const SensorId sensor_id = sensor->getId();
  CHECK(sensor_id.isValid());
  CHECK(sensors_.emplace(sensor_id, std::move(sensor)).second);
}

void SensorManager::addSensor(
    Sensor::UniquePtr sensor, const MissionId& mission_id) {
  const SensorId sensor_id = sensor->getId();
  CHECK(sensor_id.isValid());
  addSensor(std::move(sensor));
  associateExistingSensorWithMission(sensor_id, mission_id);
}

void SensorManager::associateExistingSensorWithMission(
    const SensorId& sensor_id, const MissionId& mission_id) {
  CHECK(mission_id.isValid());
  CHECK(hasSensor(sensor_id));
  AlignedUnorderedMap<MissionId, SensorIdSet>::iterator
  mission_sensor_ids_iterator = mission_id_to_sensors_map_.find(mission_id);
  if (mission_sensor_ids_iterator == mission_id_to_sensors_map_.end()) {
    SensorIdSet sensor_ids;
    sensor_ids.emplace(sensor_id);
    CHECK(mission_id_to_sensors_map_.emplace(mission_id, sensor_ids).second);
  } else {
    LOG_IF(
        FATAL, !mission_sensor_ids_iterator->second.emplace(sensor_id).second)
        << "Sensor with id " << sensor_id.hexString()
        << " is already associated with mission " << mission_id.hexString()
        << '.';
  }
}

void SensorManager::associateExistingNCameraWithMission(
    const aslam::NCameraId& ncamera_id, const MissionId& mission_id) {
  CHECK(mission_id.isValid());
  CHECK(hasSensor(ncamera_id));
  LOG_IF(
      FATAL, !mission_id_to_ncamera_map_.emplace(mission_id, ncamera_id).second)
      << "NCamera with id " << ncamera_id.hexString()
      << " is already associated with mission " << mission_id.hexString()
      << '.';
}

void SensorManager::swap(SensorManager* other) {
  CHECK_NOTNULL(other);
  ncameras_.swap(other->ncameras_);
  sensors_.swap(other->sensors_);
  mission_id_to_ncamera_map_.swap(other->mission_id_to_ncamera_map_);
  mission_id_to_sensors_map_.swap(other->mission_id_to_sensors_map_);
  CHECK(sensor_system_);
  sensor_system_.swap(other->sensor_system_);
}

bool SensorManager::hasSensorSystem() const {
  return static_cast<bool>(sensor_system_);
}

void SensorManager::addSensorSystem(SensorSystem::UniquePtr sensor_system) {
  CHECK(sensor_system);
  CHECK(!sensor_system_)
      << "There is already a sensor system with id "
      << sensor_system_->getId().hexString() << " present in the "
      << "sensor-manager. Currently, at most one sensor system is supported.";
  const SensorSystemId sensor_system_id = sensor_system->getId();
  CHECK(sensor_system_id.isValid());
  SensorIdSet sensor_ids;
  sensor_system->getAllSensorIds(&sensor_ids);
  for (const SensorId& sensor_id : sensor_ids) {
    CHECK(hasSensor(sensor_id));
  }
  sensor_system_ = std::move(sensor_system);
}

void SensorManager::addNCamera(const aslam::NCamera::Ptr& ncamera) {
  CHECK(ncamera);
  const aslam::NCameraId& ncamera_id = ncamera->getId();
  CHECK(ncamera_id.isValid());
  if (!hasSensor(ncamera_id)) {
    ncameras_.emplace(ncamera_id, ncamera);
  }
}

void SensorManager::addNCamera(
    const aslam::NCamera::Ptr& ncamera, const MissionId& mission_id) {
  addNCamera(ncamera);
  const aslam::NCameraId& ncamera_id = ncamera->getId();
  CHECK(ncamera_id.isValid());
  CHECK(mission_id.isValid());
  LOG_IF(
      FATAL, !mission_id_to_ncamera_map_.emplace(mission_id, ncamera_id).second)
      << "NCamera with id " << ncamera_id.hexString()
      << " is already associated with mission " << mission_id.hexString()
      << '.';
}

aslam::NCamera::Ptr SensorManager::getNCameraShared(
    const aslam::NCameraId& ncamera_id) {
  CHECK(ncamera_id.isValid());
  AlignedUnorderedMap<aslam::NCameraId, aslam::NCamera::Ptr>::iterator
      ncamera_iterator = ncameras_.find(ncamera_id);
  CHECK(ncamera_iterator != ncameras_.end());
  CHECK(ncamera_iterator->second);
  return ncamera_iterator->second;
}

aslam::NCamera::ConstPtr SensorManager::getNCameraShared(
    const aslam::NCameraId& ncamera_id) const {
  CHECK(ncamera_id.isValid());
  AlignedUnorderedMap<aslam::NCameraId, aslam::NCamera::Ptr>::const_iterator
  ncamera_iterator = ncameras_.find(ncamera_id);
  CHECK(ncamera_iterator != ncameras_.end());
  return static_cast<aslam::NCamera::ConstPtr>(ncamera_iterator->second);
}

bool hasExactlyOneNCameraId(const aslam::NCameraIdSet ncamera_ids) {
  if (ncamera_ids.empty()) {
    LOG(ERROR) << "Unable to retrieve the NCamera, because there is none "
               << "registerted in the sensor manager.";
    return false;
  } else if (ncamera_ids.size() > 1u) {
    LOG(ERROR) << "Unable to retrieve the NCamera, because there is more than "
               << "one registerted in the sensor manager.";
    return false;
  }
  return true;
}

aslam::NCamera::ConstPtr SensorManager::getNCameraShared() const {
  aslam::NCamera::ConstPtr ncamera;
  aslam::NCameraIdSet ncamera_ids;
  getAllNCameraIds(&ncamera_ids);
  if (hasExactlyOneNCameraId(ncamera_ids)) {
    ncamera = getNCameraShared(*ncamera_ids.begin());
  }
  return ncamera;
}

aslam::NCamera::Ptr SensorManager::getNCameraShared() {
  aslam::NCamera::Ptr ncamera;
  aslam::NCameraIdSet ncamera_ids;
  getAllNCameraIds(&ncamera_ids);
  if (hasExactlyOneNCameraId(ncamera_ids)) {
    ncamera = getNCameraShared(*ncamera_ids.begin());
  }
  return ncamera;
}

const aslam::NCamera& SensorManager::getNCamera(
    const aslam::NCameraId& ncamera_id) const {
  CHECK(ncamera_id.isValid());
  AlignedUnorderedMap<aslam::NCameraId, aslam::NCamera::Ptr>::const_iterator
  ncamera_iterator = ncameras_.find(ncamera_id);
  CHECK(ncamera_iterator != ncameras_.end());
  CHECK(ncamera_iterator->second);
  return *ncamera_iterator->second;
}

aslam::NCamera::Ptr SensorManager::getNCameraSharedForMission(
    const MissionId& mission_id) {
  CHECK(mission_id.isValid());
  AlignedUnorderedMap<MissionId, aslam::NCameraId>::const_iterator
      ncamera_id_iterator = mission_id_to_ncamera_map_.find(mission_id);
  CHECK(ncamera_id_iterator != mission_id_to_ncamera_map_.end());
  return getNCameraShared(ncamera_id_iterator->second);
}

aslam::NCamera::ConstPtr SensorManager::getNCameraSharedForMission(
    const MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  AlignedUnorderedMap<MissionId, aslam::NCameraId>::const_iterator
  ncamera_id_iterator = mission_id_to_ncamera_map_.find(mission_id);
  CHECK(ncamera_id_iterator != mission_id_to_ncamera_map_.end());
  return getNCameraShared(ncamera_id_iterator->second);
}

const aslam::NCamera& SensorManager::getNCameraForMission(
    const MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  AlignedUnorderedMap<MissionId, aslam::NCameraId>::const_iterator
  ncamera_id_iterator = mission_id_to_ncamera_map_.find(mission_id);
  CHECK(ncamera_id_iterator != mission_id_to_ncamera_map_.end());
  return getNCamera(ncamera_id_iterator->second);
}

bool SensorManager::hasNCamera(const MissionId& mission_id) const {
  CHECK(mission_id.isValid());
  return mission_id_to_ncamera_map_.count(mission_id) > 0u;
}

bool SensorManager::hasNCamera(const aslam::NCameraId& ncamera_id) const {
  CHECK(ncamera_id.isValid());
  return ncameras_.count(ncamera_id) > 0u;
}

void SensorManager::getAllNCameraIds(
    aslam::NCameraIdSet* sensor_ids) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  for (const AlignedUnorderedMap<
           aslam::NCameraId, aslam::NCamera::Ptr>::value_type& sensor_with_id :
       ncameras_) {
    CHECK(sensor_with_id.first.isValid());
    sensor_ids->emplace(sensor_with_id.first);
  }
}

void SensorManager::getAllSensorIds(SensorIdSet* sensor_ids) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  for (const AlignedUnorderedMap<SensorId, Sensor::UniquePtr>::value_type&
      sensor_with_id : sensors_) {
    CHECK(sensor_with_id.first.isValid());
    sensor_ids->emplace(sensor_with_id.first);
  }
}

void SensorManager::getAllSensorIdsOfType(
    SensorType sensor_type, SensorIdSet* sensor_ids) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  for (const AlignedUnorderedMap<SensorId, Sensor::UniquePtr>::value_type&
      sensor_with_id : sensors_) {
    CHECK(sensor_with_id.first.isValid());
    CHECK(sensor_with_id.second);
    if (sensor_with_id.second->getSensorType() == sensor_type) {
      sensor_ids->emplace(sensor_with_id.first);
    }
  }
}

void SensorManager::getAllSensorIdsOfTypeAssociatedWithMission(
    SensorType sensor_type, const MissionId& mission_id,
    SensorIdSet* sensor_ids) const {
  CHECK_NOTNULL(sensor_ids);
  CHECK(mission_id.isValid());
  getAllSensorIdsOfType(sensor_type, sensor_ids);

  SensorIdSet mission_sensor_ids;
  getAllSensorIdsAssociatedWithMission(mission_id, &mission_sensor_ids);

  SensorIdSet::iterator sensor_id_iterator = sensor_ids->begin();
  while (sensor_id_iterator != sensor_ids->end()) {
    if (mission_sensor_ids.count(*sensor_id_iterator) > 0u) {
      ++sensor_id_iterator;
    } else {
      sensor_id_iterator = sensor_ids->erase(sensor_id_iterator);
    }
  }
}

bool SensorManager::hasSensorOfType(SensorType sensor_type) const {
  SensorIdSet all_sensor_ids_of_type;
  getAllSensorIdsOfType(sensor_type, &all_sensor_ids_of_type);
  return !all_sensor_ids_of_type.empty();
}

const Sensor& SensorManager::getSensor(const SensorId& sensor_id) const {
  CHECK(hasSensor(sensor_id));
  return *CHECK_NOTNULL(common::getChecked(sensors_, sensor_id).get());
}

Sensor& SensorManager::getSensor(const SensorId& sensor_id) {
  CHECK(hasSensor(sensor_id));
  return *CHECK_NOTNULL(common::getChecked(sensors_, sensor_id).get());
}

void SensorManager::getAllSensorIdsAssociatedWithMission(
    const MissionId& mission_id, SensorIdSet* sensor_ids) const {
  CHECK(mission_id.isValid());
  CHECK_NOTNULL(sensor_ids)->clear();
  AlignedUnorderedMap<MissionId, SensorIdSet>::const_iterator
  sensor_ids_iterator =
      mission_id_to_sensors_map_.find(mission_id);
  if (sensor_ids_iterator != mission_id_to_sensors_map_.end()) {
    sensor_ids->insert(sensor_ids_iterator->second.begin(),
                       sensor_ids_iterator->second.end());
  }
}

bool SensorManager::getSensorExtrinsicsType(
    const vi_map::SensorId& id, vi_map::ExtrinsicsType* type) const {
  CHECK_NOTNULL(type);
  CHECK(id.isValid());
  if (sensor_system_ && sensor_system_->hasExtrinsicsForSensor(id)) {
    *type = sensor_system_->getSensorExtrinsicsType(id);
    return true;
  }
  return false;
}

bool SensorManager::getSensor_T_R_S(
    const vi_map::SensorId& id, aslam::Transformation* T_R_S) const {
  CHECK_NOTNULL(T_R_S);
  CHECK(id.isValid());
  if (!sensor_system_ || !sensor_system_->hasExtrinsicsForSensor(id)) {
    return false;
  }
  *T_R_S = sensor_system_->getSensor_T_R_S(id);
  return true;
}

bool SensorManager::getSensor_p_R_S(
    const vi_map::SensorId& id, Eigen::Vector3d* p_R_S) const {
  CHECK_NOTNULL(p_R_S);
  CHECK(id.isValid());
  if (!sensor_system_ || !sensor_system_->hasSensor(id) ||
      sensor_system_->getSensorExtrinsicsType(id) !=
          vi_map::ExtrinsicsType::kPositionOnly) {
    return false;
  }
  *p_R_S = sensor_system_->getSensor_p_R_S(id);
  return true;
}

void SensorManager::setSensor_T_R_S(
    const vi_map::SensorId& id, const aslam::Transformation& T_R_S) {
  CHECK(id.isValid());
  CHECK(sensor_system_);
  Extrinsics(ExtrinsicsType::kTransformation, T_R_S);
  sensor_system_->setSensorExtrinsics(id, Extrinsics(T_R_S));
}
void SensorManager::setSensor_T_R_S(
    const vi_map::SensorId& id, const aslam::Position3D& p_R_S) {
  CHECK(id.isValid());
  CHECK(sensor_system_);
  sensor_system_->setSensorExtrinsics(id, Extrinsics(p_R_S));
}

bool SensorManager::operator==(const SensorManager& other) const {
  for (const std::pair<aslam::NCameraId, aslam::NCamera::Ptr>&
           ncameras_with_id : ncameras_) {
    const aslam::NCameraId& ncamera_id = ncameras_with_id.first;
    if (!other.hasNCamera(ncamera_id)) {
      return false;
    }
    CHECK(ncameras_with_id.second);
    if (!(*ncameras_with_id.second == other.getNCamera(ncamera_id))) {
      return false;
    }
  }

  for (const AlignedUnorderedMap<SensorId, Sensor::UniquePtr>::value_type&
           sensor_id_with_sensor : sensors_) {
    const SensorId& sensor_id = sensor_id_with_sensor.first;
    if (!other.hasSensor<SensorId>(sensor_id)) {
      return false;
    }
    CHECK(sensor_id_with_sensor.second);
    if (!(*sensor_id_with_sensor.second == other.getSensor(sensor_id))) {
      return false;
    }
  }

  if (sensor_system_) {
    if (!other.sensor_system_) {
      return false;
    }
    if (!(*sensor_system_ == *other.sensor_system_)) {
      return false;
    }
  }

  return mission_id_to_sensors_map_ == other.mission_id_to_sensors_map_ &&
         mission_id_to_ncamera_map_ == other.mission_id_to_ncamera_map_;
}

void SensorManager::checkIsConsistent() const {
  SensorIdSet sensor_ids_associcated_with_missions;
  for (const AlignedUnorderedMap<MissionId, SensorIdSet>::value_type&
           mission_id_sensor_ids : mission_id_to_sensors_map_) {
    const MissionId& mission_id = mission_id_sensor_ids.first;
    CHECK(mission_id.isValid());

    for (const SensorId& sensor_id : mission_id_sensor_ids.second) {
      CHECK(sensor_id.isValid());
      CHECK(hasSensor<SensorId>(sensor_id));
      sensor_ids_associcated_with_missions.emplace(sensor_id);
    }
  }

  aslam::NCameraIdSet ncameras_associated_with_missions;
  for (const AlignedUnorderedMap<MissionId, aslam::NCameraId>::value_type&
           mission_id_ncamera_id : mission_id_to_ncamera_map_) {
    const MissionId& mission_id = mission_id_ncamera_id.first;
    CHECK(mission_id.isValid());

    const aslam::NCameraId& ncamera_id = mission_id_ncamera_id.second;
    CHECK(ncamera_id.isValid());
    CHECK(hasNCamera(ncamera_id));
    ncameras_associated_with_missions.emplace(ncamera_id);
  }

  // In case there are any sensor to mission association, every sensor needs
  // to be associated with at least one mission.
  if (!sensor_ids_associcated_with_missions.empty()) {
    for (const AlignedUnorderedMap<SensorId, Sensor::UniquePtr>::value_type&
             sensor_with_id : sensors_) {
      CHECK(sensor_with_id.first.isValid());
      CHECK(sensor_with_id.second);
      CHECK_EQ(sensor_with_id.first, sensor_with_id.second->getId());
      CHECK_GT(
          sensor_ids_associcated_with_missions.count(sensor_with_id.first), 0u);
    }
    for (const std::pair<aslam::NCameraId, aslam::NCamera::Ptr>&
             ncameras_with_id : ncameras_) {
      CHECK(ncameras_with_id.first.isValid());
      CHECK(ncameras_with_id.second);
      CHECK(hasNCamera(ncameras_with_id.first));
      CHECK_GT(
          ncameras_associated_with_missions.count(ncameras_with_id.first), 0u);
    }
  }

  if (sensor_system_) {
    CHECK(sensor_system_->getId().isValid());
    SensorIdSet sensor_ids_in_system;
    sensor_system_->getAllSensorIds(&sensor_ids_in_system);
    for (const SensorId& sensor_id : sensor_ids_in_system) {
      CHECK(sensor_id.isValid());
      CHECK(hasSensor(sensor_id));
    }
  }
}

}  // namespace vi_map
