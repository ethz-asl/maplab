#ifndef MAP_RESOURCES_TEMPORAL_RESOURCE_ID_BUFFER_H_
#define MAP_RESOURCES_TEMPORAL_RESOURCE_ID_BUFFER_H_

#include <algorithm>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <aslam/cameras/camera.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/sensor.h>
#include <aslam/common/unique-id.h>
#include <maplab-common/temporal-buffer.h>

#include "map-resources/resource-common.h"

namespace backend {

typedef std::pair<int64_t, ResourceId> StampedResourceId;

inline bool checkEqualStampedResourceId(
    const int64_t& time, const StampedResourceId& second) {
  VLOG(3) << "Test resource " << time << " vs " << second.first;
  return time == second.first;
}

struct TemporalResourceIdBuffer : public common::TemporalBuffer<ResourceId> {
  TemporalResourceIdBuffer() : common::TemporalBuffer<ResourceId>() {}

  inline void addResourceId(
      const int64_t timestamp_ns, const ResourceId& resource_id) {
    CHECK_GE(timestamp_ns, 0);
    addValue(timestamp_ns, resource_id);
  }

  inline bool hasResourceId(const int64_t timestamp_ns) const {
    CHECK_GE(timestamp_ns, 0);
    ResourceId resource_id;
    return getValueAtTime(timestamp_ns, &resource_id);
  }

  inline bool getResourceId(
      const int64_t timestamp_ns, ResourceId* resource_id) const {
    CHECK_NOTNULL(resource_id);
    CHECK_GE(timestamp_ns, 0);
    return getValueAtTime(timestamp_ns, resource_id);
  }

  inline bool deleteResourceId(const int64_t timestamp_ns) {
    CHECK_GE(timestamp_ns, 0);
    return deleteValueAtTime(timestamp_ns);
  }

  inline bool getClosestResourceId(
      const int64_t timestamp_ns, const int64_t tolerance_ns,
      StampedResourceId* stamped_resource_id) const {
    CHECK_NOTNULL(stamped_resource_id);
    CHECK_GE(timestamp_ns, 0);
    CHECK_GE(tolerance_ns, 0);
    return getNearestValueToTime(
        timestamp_ns, tolerance_ns, &stamped_resource_id->second,
        &stamped_resource_id->first);
  }

  inline common::TemporalBuffer<ResourceId>::BufferType& resource_id_map() {
    return getValues();
  }

  inline const common::TemporalBuffer<ResourceId>::BufferType& resource_id_map()
      const {
    return getValues();
  }
};

typedef common::TemporalBuffer<ResourceId>::BufferType StampedResourceIds;

using SensorIdToResourcesMap =
    std::unordered_map<aslam::SensorId, TemporalResourceIdBuffer>;

using ResourceTypeToSensorIdToResourcesMap =
    std::unordered_map<ResourceType, SensorIdToResourcesMap, ResourceTypeHash>;

}  // namespace backend

#endif  // MAP_RESOURCES_TEMPORAL_RESOURCE_ID_BUFFER_H_
