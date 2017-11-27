#ifndef VI_MAP_SENSOR_MANAGER_H_
#define VI_MAP_SENSOR_MANAGER_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <maplab-common/macros.h>
#include <maplab-common/yaml-file-serializable.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <sensors/imu.h>
#include <sensors/relative-6dof-pose.h>
#include <sensors/sensor-system.h>

#include "vi-map/unique-id.h"

namespace vi_map {

template<typename SensorType, typename IdType>
inline void addSensorImpl(
    typename SensorType::UniquePtr sensor,
    AlignedUnorderedMap<IdType, typename SensorType::UniquePtr>* container) {
  CHECK_NOTNULL(container);
  CHECK(sensor);
  const IdType& sensor_id = sensor->getId();
  CHECK(sensor_id.isValid());
  CHECK(container->emplace(sensor_id, std::move(sensor)).second);
}

class SensorManager : public common::YamlFileSerializable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(SensorManager);
  friend class VIMapGenerator;

  SensorManager() = default;
  virtual ~SensorManager() = default;

  void addSensor(Sensor::UniquePtr sensor, const MissionId& mission_id);
  void associateExistingSensorWithMission(
      const SensorId& sensor_id, const MissionId& mission_id);
  void associateExistingNCameraWithMission(
      const aslam::NCameraId& ncamera_id, const MissionId& mission_id);

  bool hasSensorSystem() const;
  void addSensorSystem(SensorSystem::UniquePtr sensor_system);
  const SensorSystem& getSensorSystem() const {
    CHECK(sensor_system_);
    return *sensor_system_;
  }

  template <class SensorType>
  bool getSensor(const SensorId& sensor_id, SensorType* sensor) const;
  template <class SensorType>
  bool getSensor(SensorType* sensor) const;

  // Convenience function to retrieve the sensor if only a single one is
  // associated with a mission. Check fails if more than one sensor is present.
  template<class SensorType>
  const SensorType& getSensor(const SensorId& sensor_id) const;
  template <class SensorType>
  const SensorType& getSensorForMission(const MissionId& mission_id) const;

  const Sensor& getSensor(const SensorId& sensor_id) const;
  Sensor& getSensor(const SensorId& sensor_id);

  void getAllSensorIds(SensorIdSet* sensor_ids) const;
  void getAllSensorIdsAssociatedWithMission(
      const MissionId& mission_id, SensorIdSet* sensor_ids) const;

  void getAllSensorIdsOfType(
      SensorType sensor_type, SensorIdSet* sensor_ids) const;
  void getAllSensorIdsOfTypeAssociatedWithMission(
      SensorType sensor_type, const MissionId& mission_id,
      SensorIdSet* sensor_ids) const;

  void addNCamera(
      const aslam::NCamera::Ptr& ncamera, const MissionId& mission_id);

  void getAllNCameraIds(aslam::NCameraIdSet* sensor_ids) const;

  const aslam::NCamera& getNCamera(const aslam::NCameraId& ncamera_id) const;
  const aslam::NCamera& getNCameraForMission(const MissionId& mission_id) const;

  aslam::NCamera::Ptr getNCameraShared(const aslam::NCameraId& ncamera_id);
  aslam::NCamera::Ptr getNCameraSharedForMission(const MissionId& mission_id);
  aslam::NCamera::ConstPtr getNCameraShared(
      const aslam::NCameraId& ncamera_id) const;
  aslam::NCamera::ConstPtr getNCameraShared() const;
  aslam::NCamera::Ptr getNCameraShared();
  aslam::NCamera::ConstPtr getNCameraSharedForMission(
      const MissionId& mission_id) const;

  bool hasNCamera(const aslam::NCameraId& ncamera_id) const;
  bool hasNCamera(const MissionId& mission_id) const;

  template<class IdType>
  bool hasSensor(const IdType& sensor_id) const;

  bool hasSensorOfType(SensorType sensor_type) const;

  size_t getNumNCameraSensors() const {
    return ncameras_.size();
  }
  size_t getNumSensors() const {
    return sensors_.size();
  }
  template <class DerivedSensor>
  inline size_t getNumSensorsOfTypeAssociatedWithMission(
      const MissionId& mission_id) const;

  // Returns false if no extrinsics of this type are available.
  bool getSensorExtrinsicsType(
      const vi_map::SensorId& id, vi_map::ExtrinsicsType* type) const;
  bool getSensor_T_R_S(
      const vi_map::SensorId& id, aslam::Transformation* T_R_S) const;
  bool getSensor_p_R_S(
      const vi_map::SensorId& id, aslam::Position3D* p_R_S) const;
  void setSensor_T_R_S(
      const vi_map::SensorId& id, const aslam::Transformation& T_R_S);
  void setSensor_T_R_S(
      const vi_map::SensorId& id, const aslam::Position3D& p_R_S);

  void serialize(YAML::Node* yaml_node) const override;
  bool deserialize(const YAML::Node& yaml_node) override;

  bool operator==(const SensorManager& other) const;
  bool operator!=(const SensorManager& other) const {
    return !operator==(other);
  }

  void swap(SensorManager* other);

 protected:
  void addSensor(Sensor::UniquePtr sensor);
  void addNCamera(const aslam::NCamera::Ptr& ncamera);

  AlignedUnorderedMap<aslam::NCameraId, aslam::NCamera::Ptr> ncameras_;
  AlignedUnorderedMap<SensorId, Sensor::UniquePtr> sensors_;
  AlignedUnorderedMap<MissionId, SensorIdSet> mission_id_to_sensors_map_;
  AlignedUnorderedMap<MissionId, aslam::NCameraId> mission_id_to_ncamera_map_;
  SensorSystem::UniquePtr sensor_system_;

 private:
  void checkIsConsistent() const;
};

}  // namespace vi_map

#include "./sensor-manager-inl.h"

#endif  // VI_MAP_SENSOR_MANAGER_H_
