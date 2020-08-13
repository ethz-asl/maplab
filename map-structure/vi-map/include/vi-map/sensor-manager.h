#ifndef VI_MAP_SENSOR_MANAGER_H_
#define VI_MAP_SENSOR_MANAGER_H_

#include <algorithm>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/common/sensor.h>
#include <aslam/common/unique-id.h>
#include <aslam/common/yaml-file-serialization.h>
#include <glog/logging.h>
#include <maplab-common/accessors.h>
#include <maplab-common/macros.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/loop-closure-sensor.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/sensor-types.h>
#include <sensors/wheel-odometry-sensor.h>

#include "vi-map/unique-id.h"

namespace vi_map {

class SensorManager : public aslam::YamlFileSerializable {
 public:
  MAPLAB_POINTER_TYPEDEFS(SensorManager);

  SensorManager() = default;
  ~SensorManager() = default;

  SensorManager(const SensorManager& other);
  void operator=(const SensorManager&) = delete;

  bool isEqual(const SensorManager& other, const bool verbose = false) const;
  bool operator==(const SensorManager& other) const;
  bool operator!=(const SensorManager& other) const;

  SensorManager::Ptr clone() const {
    return aligned_shared<SensorManager>(*this);
  }

  // Merge by cloning the sensors and extrinsics from another sensor manager.
  void merge(const SensorManager& other);

  // Swap the contents of two sensor managers.
  void swap(SensorManager* other);

  // Adds a sensor to the manager, which takes over the sensor but offers shared
  // ownership if retreived later.
  template <typename DerivedSensor>
  void addSensor(
      typename DerivedSensor::UniquePtr sensor,
      const aslam::SensorId& base_sensor_id,
      const aslam::Transformation& T_B_S);
  template <typename DerivedSensor>
  void addSensor(
      typename DerivedSensor::UniquePtr sensor,
      const aslam::SensorId& base_sensor_id, const aslam::Position3D& p_B_S);
  template <typename DerivedSensor>
  void addSensorAsBase(typename DerivedSensor::UniquePtr sensor);

  // Removes a sensor and its extrinsics from the node. The function will check
  // fail if the sensor id does not exist or if the sensor is a base sensor and
  // still has any other sensors defined with respect to it
  void removeSensor(const aslam::SensorId& sensor_id);

  template <typename DerivedSensor>
  const DerivedSensor& getSensor(const aslam::SensorId& sensor_id) const;
  template <typename DerivedSensor>
  typename DerivedSensor::Ptr getSensorPtr(
      const aslam::SensorId& sensor_id) const;

  void getAllSensorIds(aslam::SensorIdSet* all_sensor_ids) const;
  void getAllSensorIdsOfType(
      const vi_map::SensorType& sensor_type,
      aslam::SensorIdSet* all_sensor_ids) const;

  const aslam::SensorIdSet getAllSensorIdsOfType(
      const vi_map::SensorType& sensor_type) const;
      
  size_t getNumSensors() const;
  size_t getNumSensorsOfType(const vi_map::SensorType& sensor_type) const;

  // Check if a sensor with a given id exists.
  bool hasSensor(const aslam::SensorId& sensor_id) const;
  // Check if a sensor with a given type exists.
  bool hasSensorOfType(const vi_map::SensorType& sensor_type) const;
  // Get the type of a sensor.
  SensorType getSensorType(const aslam::SensorId& sensor_id) const;

  // Check if a sensor is a base reference sensor.
  bool isBaseSensor(const aslam::SensorId& base_sensor_id) const;
  // Get the base reference sensor id of a sensor, if it is defined.
  const aslam::SensorId& getBaseSensorId(
      const aslam::SensorId& sensor_id) const;

  // Get the base sensor id, if there is only one, returns false otherwise.
  bool getBaseSensorIdIfUnique(aslam::SensorId* sensor_id) const;

  // Set the sensors calibration transformation to the base sensor
  void setSensor_T_B_S(
      const aslam::SensorId& sensor_id, const aslam::Transformation& T_B_S);
  // Assumes no rotation in the extrinsics with respect to the base sensor.
  void setSensor_T_B_S(
      const aslam::SensorId& sensor_id, const aslam::Position3D& p_B_S);
  // Returns the relative transformation T_B_S to the base sensor.
  const aslam::Transformation& getSensor_T_B_S(
      const aslam::SensorId& sensor_id) const;
  aslam::Transformation& getSensor_T_B_S(const aslam::SensorId& sensor_id);

  void serialize(YAML::Node* yaml_node) const override;
  bool deserialize(const YAML::Node& yaml_node) override;

 private:
  template <typename DerivedSensor>
  void addSensor(
      const typename DerivedSensor::Ptr& sensor,
      const aslam::SensorId& base_sensor_id,
      const aslam::Transformation& T_B_S);
  template <typename DerivedSensor>
  void addSensorAsBase(typename DerivedSensor::Ptr sensor);

  AlignedUnorderedMap<aslam::SensorId, aslam::Sensor::Ptr> sensors_;
  AlignedUnorderedMap<aslam::SensorId, aslam::SensorId> base_sensor_id_map_;
  AlignedUnorderedMap<aslam::SensorId, aslam::Transformation> T_B_S_map_;
};

}  // namespace vi_map

#include "./sensor-manager-inl.h"

#endif  // VI_MAP_SENSOR_MANAGER_H_
