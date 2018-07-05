#include "vi-map/sensor-manager.h"

#include <maplab-common/accessors.h>

namespace vi_map {
template <>
bool SensorManager::hasSensor<aslam::NCameraId>(
    const aslam::NCameraId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return ncameras_.count(sensor_id) > 0u;
}

template <>
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

void SensorManager::associateExistingOptionalNCameraWithMission(
    const aslam::NCameraId& ncamera_id, const MissionId& mission_id) {
  CHECK(mission_id.isValid());
  CHECK(hasSensor(ncamera_id));
  std::unordered_set<aslam::NCameraId>& ncamera_ids =
      mission_id_to_optional_ncamera_map_[mission_id];
  CHECK(ncamera_ids.emplace(ncamera_id).second)
      << "Optional nCamera with id " << ncamera_id.hexString()
      << " is already associated with mission " << mission_id.hexString()
      << '.';
}

void SensorManager::removeSensorFromMission(
    const SensorId& sensor_id, const vi_map::MissionId& mission_id) {
  CHECK(sensor_id.isValid());
  CHECK(mission_id.isValid());
  CHECK(hasSensor(sensor_id));

  typedef AlignedUnorderedMap<MissionId, SensorIdSet> MissionIdToSensorIdSet;
  const MissionIdToSensorIdSet::iterator it_mission_to_sensors =
      mission_id_to_sensors_map_.find(mission_id);
  CHECK(it_mission_to_sensors != mission_id_to_sensors_map_.cend());
  SensorIdSet& sensor_ids_associated_with_mission =
      it_mission_to_sensors->second;
  CHECK_GT(sensor_ids_associated_with_mission.count(sensor_id), 0u);

  // Build inverse map from sensor id to associated mission ids.
  typedef std::unordered_map<SensorId, MissionIdSet> SensorIdToMissionIdSet;
  SensorIdToMissionIdSet sensor_to_missions_map;
  for (const MissionIdToSensorIdSet::value_type& mission_id_to_sensor_set :
       mission_id_to_sensors_map_) {
    for (const SensorId& sensor_id : mission_id_to_sensor_set.second) {
      sensor_to_missions_map[sensor_id].emplace(mission_id_to_sensor_set.first);
    }
  }

  SensorIdToMissionIdSet::const_iterator sensor_to_mission_it =
      sensor_to_missions_map.find(sensor_id);
  CHECK(sensor_to_mission_it != sensor_to_missions_map.cend());
  CHECK_GT(sensor_to_mission_it->second.count(mission_id), 0u);
  if (sensor_to_mission_it->second.size() == 1u) {
    sensors_.erase(sensor_id);
    it_mission_to_sensors->second.erase(sensor_id);
    if (sensor_system_ && sensor_system_->hasSensor(sensor_id)) {
      CHECK_NE(sensor_id, sensor_system_->getReferenceSensorId())
          << "Attempting to remove the reference sensor from the sensor "
          << "system. In this case, the entire sensor system must be removed. "
          << "This case is not supported.";
      sensor_system_->removeSensor(sensor_id);
    }
  }
}

void SensorManager::removeAllSensorsAssociatedToMission(
    const vi_map::MissionId& mission_id) {
  CHECK(mission_id.isValid());
  typedef AlignedUnorderedMap<MissionId, SensorIdSet> MissionIdToSensorIdSet;
  const MissionIdToSensorIdSet::const_iterator it_mission_to_sensors =
      mission_id_to_sensors_map_.find(mission_id);
  if (it_mission_to_sensors != mission_id_to_sensors_map_.cend()) {
    // Build inverse map from sensor id to associated mission ids.
    typedef std::unordered_map<SensorId, MissionIdSet> SensorIdToMissionIdSet;
    SensorIdToMissionIdSet sensor_to_missions_map;
    for (const MissionIdToSensorIdSet::value_type& mission_id_to_sensor_set :
         mission_id_to_sensors_map_) {
      for (const SensorId& sensor_id : mission_id_to_sensor_set.second) {
        sensor_to_missions_map[sensor_id].emplace(
            mission_id_to_sensor_set.first);
      }
    }

    // Delete sensor if it's only used by the mission to remove.
    for (const SensorIdToMissionIdSet::value_type& sensor_id_to_mission_id_set :
         sensor_to_missions_map) {
      const MissionIdSet& mission_ids_for_sensor =
          sensor_id_to_mission_id_set.second;
      if (mission_ids_for_sensor.size() == 1u &&
          mission_id == *mission_ids_for_sensor.cbegin()) {
        const SensorId& sensor_id = sensor_id_to_mission_id_set.first;
        sensors_.erase(sensor_id);
        if (sensor_system_ && sensor_system_->hasSensor(sensor_id)) {
          CHECK_NE(sensor_id, sensor_system_->getReferenceSensorId())
              << "Attempting to remove the reference sensor from the sensor "
              << "system. In this case, the entire sensor system must be "
              << "removed. This case is not supported.";
          sensor_system_->removeSensor(sensor_id);
        }
      }
    }

    mission_id_to_sensors_map_.erase(it_mission_to_sensors);
  }

  typedef AlignedUnorderedMap<MissionId, aslam::NCameraId> MissionIdToCameraId;
  const MissionIdToCameraId::const_iterator it_mission_to_camera =
      mission_id_to_ncamera_map_.find(mission_id);
  if (it_mission_to_camera != mission_id_to_ncamera_map_.cend()) {
    const aslam::NCameraId& n_camera_id = it_mission_to_camera->second;
    size_t references_to_n_camera = 0u;
    for (const MissionIdToCameraId::value_type& map_entry :
         mission_id_to_ncamera_map_) {
      if (map_entry.second == n_camera_id) {
        ++references_to_n_camera;
      }
    }

    if (references_to_n_camera == 1u) {
      // If there's only one reference to the NCamera, we can erase it.
      ncameras_.erase(n_camera_id);
    }
    mission_id_to_ncamera_map_.erase(it_mission_to_camera);
  }

  typedef AlignedUnorderedMap<MissionId, std::unordered_set<aslam::NCameraId>>
      MissionIdToOptionalNCameraIdMapd;
  const MissionIdToOptionalNCameraIdMapd::const_iterator
      it_mission_to_optional_ncamera_ids =
          mission_id_to_optional_ncamera_map_.find(mission_id);
  if (it_mission_to_optional_ncamera_ids !=
      mission_id_to_optional_ncamera_map_.cend()) {
    for (const aslam::NCameraId& n_camera_id :
         it_mission_to_optional_ncamera_ids->second) {
      size_t references_to_n_camera = 0u;
      for (const MissionIdToOptionalNCameraIdMapd::value_type& map_entry :
           mission_id_to_optional_ncamera_map_) {
        references_to_n_camera += map_entry.second.count(n_camera_id);
      }

      if (references_to_n_camera == 1u) {
        // If there's only one reference to the NCamera, we can erase it.
        ncameras_.erase(n_camera_id);
      }
    }
    mission_id_to_optional_ncamera_map_.erase(mission_id);
  }
}

void SensorManager::swap(SensorManager* other) {
  CHECK_NOTNULL(other);
  ncameras_.swap(other->ncameras_);
  sensors_.swap(other->sensors_);
  mission_id_to_ncamera_map_.swap(other->mission_id_to_ncamera_map_);
  mission_id_to_optional_ncamera_map_.swap(
      other->mission_id_to_optional_ncamera_map_);
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

void SensorManager::getAllNCameraIds(aslam::NCameraIdSet* sensor_ids) const {
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
      sensor_ids_iterator = mission_id_to_sensors_map_.find(mission_id);
  if (sensor_ids_iterator != mission_id_to_sensors_map_.end()) {
    sensor_ids->insert(
        sensor_ids_iterator->second.begin(), sensor_ids_iterator->second.end());
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

  if (!((mission_id_to_sensors_map_ == other.mission_id_to_sensors_map_) &&
        (mission_id_to_ncamera_map_ == other.mission_id_to_ncamera_map_))) {
    return false;
  }

  if (!(mission_id_to_optional_ncamera_map_ ==
        other.mission_id_to_optional_ncamera_map_)) {
    return false;
  }
  return true;
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

  aslam::NCameraIdSet optional_ncameras_associated_with_missions;
  for (const AlignedUnorderedMap<
           MissionId, std::unordered_set<aslam::NCameraId>>::value_type&
           mission_id_ncamera_id_set : mission_id_to_optional_ncamera_map_) {
    const MissionId& mission_id = mission_id_ncamera_id_set.first;
    CHECK(mission_id.isValid());

    for (const aslam::NCameraId& ncamera_id :
         mission_id_ncamera_id_set.second) {
      CHECK(ncamera_id.isValid());
      CHECK(hasNCamera(ncamera_id));
      optional_ncameras_associated_with_missions.emplace(ncamera_id);
    }
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

      // The camera needs to be either associated with a mission as a main
      // NCamera or as an optional NCamera.
      const bool is_ncamera_associated_with_mission =
          ncameras_associated_with_missions.count(ncameras_with_id.first) > 0u;
      const bool is_optional_ncamera_associated_with_mission =
          optional_ncameras_associated_with_missions.count(
              ncameras_with_id.first) > 0u;
      CHECK(
          is_ncamera_associated_with_mission ^
          is_optional_ncamera_associated_with_mission)
          << "The ncamera " << ncameras_with_id.first
          << " is not associated with any mission, neither as main ncamera or "
          << "optional ncamera!";
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

backend::CameraWithExtrinsics SensorManager::getOptionalCameraWithExtrinsics(
    const aslam::CameraId& camera_id) const {
  for (const std::pair<aslam::NCameraId, aslam::NCamera::Ptr>& ncamera :
       ncameras_) {
    CHECK(ncamera.second) << "The sensor manager contains an invalid NCamera!";
    const size_t num_cameras = ncamera.second->getNumCameras();

    for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
      aslam::Camera::Ptr camera_ptr =
          ncamera.second->getCameraShared(camera_idx);
      CHECK(camera_ptr)
          << "The sensor manager contains an invalid Camera at index "
          << camera_idx << " in NCamera "
          << ncamera.second->getId().hexString();
      if (camera_ptr->getId() == camera_id) {
        const aslam::Transformation& T_C_B =
            ncamera.second->get_T_C_B(camera_idx);
        return backend::CameraWithExtrinsics(T_C_B, camera_ptr);
      }
    }
  }
  LOG(FATAL) << "Could not find an NCamera that contains a camera with the id: "
             << camera_id.hexString();
  return backend::CameraWithExtrinsics();
}

void SensorManager::addOptionalCameraWithExtrinsics(
    const aslam::Camera& camera, const aslam::Transformation& T_C_B,
    const MissionId& mission_id) {
  std::vector<MissionId> mission_ids;
  mission_ids.push_back(mission_id);
  addOptionalCameraWithExtrinsics(camera, T_C_B, mission_ids);
}

void SensorManager::addOptionalCameraWithExtrinsics(
    const aslam::Camera& camera, const aslam::Transformation& T_C_B,
    const std::vector<MissionId>& mission_ids) {
  aslam::TransformationVector T_C_B_vector;
  T_C_B_vector.push_back(T_C_B);

  std::vector<aslam::Camera::Ptr> camera_vector;
  camera_vector.push_back(aslam::Camera::Ptr(camera.clone()));

  const std::string kCameraLabel = "single camera";

  aslam::NCameraId ncamera_id;
  common::generateId(&ncamera_id);

  aslam::NCamera::Ptr new_dummy_ncamera(new aslam::NCamera(
      ncamera_id, T_C_B_vector, camera_vector, kCameraLabel));

  addNCamera(new_dummy_ncamera);

  for (const MissionId& mission_id : mission_ids) {
    associateExistingOptionalNCameraWithMission(ncamera_id, mission_id);
  }
}

bool SensorManager::hasOptionalCameraWithExtrinsics(
    const aslam::CameraId& camera_id) const {
  for (const std::pair<aslam::NCameraId, aslam::NCamera::Ptr>& ncamera :
       ncameras_) {
    CHECK(ncamera.second) << "The sensor manager contains an invalid NCamera!";
    const size_t num_cameras = ncamera.second->getNumCameras();

    for (size_t camera_idx = 0u; camera_idx < num_cameras; ++camera_idx) {
      aslam::Camera::Ptr camera_ptr =
          ncamera.second->getCameraShared(camera_idx);
      CHECK(camera_ptr)
          << "The sensor manager contains an invalid Camera at index "
          << camera_idx << " in NCamera "
          << ncamera.second->getId().hexString();
      if (camera_ptr->getId() == camera_id) {
        return true;
      }
    }
  }
  return false;
}

bool SensorManager::getOptionalCamerasWithExtrinsicsForMissionId(
    const MissionId& mission_id,
    std::vector<backend::CameraWithExtrinsics>* cameras_with_extrinsics) const {
  CHECK_NOTNULL(cameras_with_extrinsics)->clear();

  aslam::NCameraIdSet optional_ncamera_ids;
  getOptionalNCameraIdsForMissionId(mission_id, &optional_ncamera_ids);
  if (optional_ncamera_ids.empty()) {
    return false;
  }

  for (const aslam::NCameraId& ncamera_id : optional_ncamera_ids) {
    AlignedUnorderedMap<aslam::NCameraId, aslam::NCamera::Ptr>::const_iterator
        ncamera_it = ncameras_.find(ncamera_id);
    CHECK(ncamera_it != ncameras_.end())
        << "Cannot find optional NCamera " << ncamera_id
        << " that is registered to mission " << mission_id.hexString() << ".";
    aslam::NCamera::Ptr ncamera = ncamera_it->second;
    CHECK(ncamera) << "The sensor manager contains an invalid NCamera!";
    // There should always be only one camera in an optional ncamera.
    CHECK_EQ(ncamera->getNumCameras(), 1u);

    aslam::Camera::Ptr camera_ptr = ncamera->getCameraShared(0u);
    const aslam::Transformation& T_C_B = ncamera->get_T_C_B(0u);

    CHECK(camera_ptr)
        << "The sensor manager contains an invalid Camera in optional NCamera "
        << ncamera->getId().hexString();
    cameras_with_extrinsics->emplace_back(T_C_B, camera_ptr);
  }

  return true;
}

void SensorManager::getOptionalNCameraIdsForMissionId(
    const MissionId& mission_id,
    aslam::NCameraIdSet* optional_ncamera_ids) const {
  CHECK_NOTNULL(optional_ncamera_ids)->clear();

  AlignedUnorderedMap<MissionId,
                      std::unordered_set<aslam::NCameraId>>::const_iterator it =
      mission_id_to_optional_ncamera_map_.find(mission_id);
  if (it == mission_id_to_optional_ncamera_map_.end()) {
    return;
  }

  *optional_ncamera_ids = it->second;
}

void SensorManager::merge(
    const SensorManager& other, const MissionId& mission_id_to_merge) {
  const aslam::NCamera& other_mission_ncamera =
      other.getNCameraForMission(mission_id_to_merge);
  const aslam::NCameraId& other_mission_ncamera_id =
      other_mission_ncamera.getId();
  CHECK(other_mission_ncamera_id.isValid());
  if (hasNCamera(other_mission_ncamera_id)) {
    associateExistingNCameraWithMission(
        other_mission_ncamera_id, mission_id_to_merge);
  } else {
    addNCamera(other_mission_ncamera.cloneToShared(), mission_id_to_merge);
  }

  aslam::NCameraIdSet mission_to_merge_optional_ncamera_ids;
  other.getOptionalNCameraIdsForMissionId(
      mission_id_to_merge, &mission_to_merge_optional_ncamera_ids);

  for (const aslam::NCameraId& mission_to_merge_optional_ncamera_id :
       mission_to_merge_optional_ncamera_ids) {
    CHECK(mission_to_merge_optional_ncamera_id.isValid());
    if (hasNCamera(mission_to_merge_optional_ncamera_id)) {
      associateExistingOptionalNCameraWithMission(
          mission_to_merge_optional_ncamera_id, mission_id_to_merge);
    } else {
      const aslam::NCamera& optional_ncamera =
          other.getNCamera(mission_to_merge_optional_ncamera_id);
      CHECK_EQ(optional_ncamera.numCameras(), 1u);
      const MissionIdList mission_ids_to_associate_with = {mission_id_to_merge};
      addOptionalCameraWithExtrinsics(
          optional_ncamera.getCamera(0u), optional_ncamera.get_T_C_B(0u),
          mission_ids_to_associate_with);
    }
  }

  SensorIdSet other_mission_sensor_ids;
  other.getAllSensorIdsAssociatedWithMission(
      mission_id_to_merge, &other_mission_sensor_ids);
  for (const SensorId& other_mission_sensor_id : other_mission_sensor_ids) {
    CHECK(other_mission_sensor_id.isValid());
    if (hasSensor(other_mission_sensor_id)) {
      associateExistingSensorWithMission(
          other_mission_sensor_id, mission_id_to_merge);
    } else {
      addSensor(
          other.getSensor(other_mission_sensor_id).clone(),
          mission_id_to_merge);
    }
  }
}

template <>
bool SensorManager::getSensorOrCamera_T_R_S(
    const vi_map::SensorId& id, aslam::Transformation* T_R_S) const {
  CHECK_NOTNULL(T_R_S);
  return getSensor_T_R_S(id, T_R_S);
}

template <>
bool SensorManager::getSensorOrCamera_T_R_S(
    const aslam::CameraId& id, aslam::Transformation* T_R_S) const {
  CHECK_NOTNULL(T_R_S);
  backend::CameraWithExtrinsics camera_with_extrinsics;
  camera_with_extrinsics = getOptionalCameraWithExtrinsics(id);
  *T_R_S = camera_with_extrinsics.first;
  return true;
}

}  // namespace vi_map
