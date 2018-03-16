#ifndef VI_MAP_VI_MISSION_INL_H_
#define VI_MAP_VI_MISSION_INL_H_

#include <unordered_map>
#include <vector>

#include <glog/logging.h>
#include <map-resources/optional-sensor-resources.h>

#include "vi-map/vi-mission.h"

namespace vi_map {

template <typename SensorId>
bool VIMission::getOptionalSensorResourceId(
    const backend::ResourceType& type, const SensorId& camera_id,
    const int64_t timestamp_ns, backend::ResourceId* resource_id) const {
  CHECK_NOTNULL(resource_id);
  CHECK_GE(timestamp_ns, 0);

  const backend::OptionalSensorResources* optional_resources =
      getAllOptionalSensorResourceIdsForSensorOfType(type, camera_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->getResourceId(timestamp_ns, resource_id);
}

template <typename SensorId>
bool VIMission::getClosestOptionalSensorResourceId(
    const backend::ResourceType& type, const SensorId& sensor_id,
    const int64_t timestamp_ns, const int64_t tolerance_ns,
    backend::StampedResourceId* stamped_resource_id) const {
  CHECK_NOTNULL(stamped_resource_id);
  CHECK_GE(timestamp_ns, 0);

  const backend::OptionalSensorResources* optional_resources =
      getAllOptionalSensorResourceIdsForSensorOfType(type, sensor_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->getClosestResourceId(
      timestamp_ns, tolerance_ns, stamped_resource_id);
}

template <typename SensorId>
bool VIMission::deleteOptionalSensorResourceId(
    const backend::ResourceType& type, const SensorId& sensor_id,
    const int64_t timestamp_ns) {
  CHECK_GE(timestamp_ns, 0);

  backend::OptionalSensorResources* optional_resources =
      getAllOptionalSensorResourceIdsForSensorOfTypeMutable(type, sensor_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->deleteResourceId(timestamp_ns);
}

template <typename SensorId>
const backend::OptionalSensorResources*
VIMission::getAllOptionalSensorResourceIdsForSensorOfType(
    const backend::ResourceType& type, const SensorId& sensor_id) const {
  const std::unordered_map<SensorId, backend::OptionalSensorResources>*
      optional_sensor_resources_of_type =
          getAllOptionalSensorResourceIdsOfType<SensorId>(type);
  if (optional_sensor_resources_of_type == nullptr) {
    return nullptr;
  }

  typename std::unordered_map<
      SensorId, backend::OptionalSensorResources>::const_iterator sensor_it =
      optional_sensor_resources_of_type->find(sensor_id);
  if (sensor_it == optional_sensor_resources_of_type->cend()) {
    return nullptr;
  }

  return &(sensor_it->second);
}

template <typename SensorId>
backend::OptionalSensorResources*
VIMission::getAllOptionalSensorResourceIdsForSensorOfTypeMutable(
    const backend::ResourceType& type, const SensorId& sensor_id) {
  std::unordered_map<SensorId, backend::OptionalSensorResources>*
      optional_sensor_resources_of_type =
          getAllOptionalSensorResourceIdsOfTypeMutable<SensorId>(type);
  if (optional_sensor_resources_of_type == nullptr) {
    return nullptr;
  }

  typename std::unordered_map<
      SensorId, backend::OptionalSensorResources>::iterator sensor_it =
      optional_sensor_resources_of_type->find(sensor_id);
  if (sensor_it == optional_sensor_resources_of_type->cend()) {
    return nullptr;
  }

  return &(sensor_it->second);
}

template <typename SensorId>
bool VIMission::findAllCloseOptionalSensorResources(
    const backend::ResourceType& type, const int64_t timestamp_ns,
    const int64_t tolerance_ns, std::vector<SensorId>* sensor_ids,
    std::vector<int64_t>* closest_timestamps_ns) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  CHECK_NOTNULL(closest_timestamps_ns)->clear();

  const backend::CameraIdToResourcesMap* optional_resources =
      getAllOptionalSensorResourceIdsOfType<SensorId>(type);

  if (optional_resources == nullptr) {
    return false;
  }

  size_t num_resources_found = 0u;
  for (const typename std::unordered_map<
           SensorId, backend::OptionalSensorResources>::value_type&
           resources_per_sensor : *optional_resources) {
    const SensorId& sensor_id = resources_per_sensor.first;
    const backend::OptionalSensorResources& stamped_resource_buffer =
        resources_per_sensor.second;
    backend::StampedResourceId stamped_resource_id;
    if (!stamped_resource_buffer.getClosestResourceId(
            timestamp_ns, tolerance_ns, &stamped_resource_id)) {
      continue;
    }

    sensor_ids->emplace_back(sensor_id);
    closest_timestamps_ns->push_back(stamped_resource_id.first);
    ++num_resources_found;
  }

  return num_resources_found > 0u;
}

template <typename SensorId>
bool VIMission::hasOptionalSensorResourceId(
    const backend::ResourceType& type, const SensorId& sensor_id,
    const int64_t timestamp_ns) const {
  const backend::OptionalSensorResources* optional_resources =
      getAllOptionalSensorResourceIdsForSensorOfType(type, sensor_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->hasResourceId(timestamp_ns);
}
}  //  namespace vi_map

#endif  // VI_MAP_VI_MISSION_INL_H_
