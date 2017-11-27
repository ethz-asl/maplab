#ifndef MAP_RESOURCES_OPTIONAL_SENSOR_RESOURCES_H_
#define MAP_RESOURCES_OPTIONAL_SENSOR_RESOURCES_H_

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <aslam/cameras/camera.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <maplab-common/temporal-buffer.h>
#include <maplab-common/unique-id.h>

#include "map-resources/resource-common.h"

namespace backend {

typedef std::pair<int64_t, ResourceId> StampedResourceId;

struct OptionalSensorResources : public common::TemporalBuffer<ResourceId> {
  OptionalSensorResources() : common::TemporalBuffer<ResourceId>() {}

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
    return values_;
  }

  inline const common::TemporalBuffer<ResourceId>::BufferType& resource_id_map()
      const {
    return values_;
  }
};

typedef common::TemporalBuffer<ResourceId>::BufferType StampedResourceIds;

//
// aslam::Camera specific optional sensor data.
//
typedef std::unordered_map<aslam::CameraId, OptionalSensorResources>
    OptionalCameraResourcesMap;

typedef std::unordered_map<ResourceType, OptionalCameraResourcesMap,
                           ResourceTypeHash>
    ResourceTypeToOptionalCameraResourcesMap;

typedef std::pair<aslam::Transformation, std::shared_ptr<aslam::Camera>>
    CameraWithExtrinsics;

typedef AlignedUnorderedMap<aslam::CameraId, CameraWithExtrinsics>
    OptionalCameraMap;

}  // namespace backend

#endif  // MAP_RESOURCES_OPTIONAL_SENSOR_RESOURCES_H_
