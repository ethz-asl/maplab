#ifndef VI_MAP_VI_MISSION_H_
#define VI_MAP_VI_MISSION_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/common/pose-types.h>
#include <map-resources/optional-sensor-resources.h>
#include <map-resources/resource-common.h>
#include <maplab-common/pose_types.h>
#include <sensors/imu.h>
#include <sensors/sensor.h>

#include "vi-map/mission.h"
#include "vi-map/optional-sensor-data.h"
#include "vi-map/unique-id.h"
#include "vi-map/vertex.h"

namespace vi_map {

// Used to shift vertex timestamps when user-defined mission ordering is used.
const int64_t kFakeNumNanosecondsBetweenMissions = 864e11;  // 1 day.

class VIMap;

class VIMission : public Mission {
  friend class MissionResourcesTest;

 public:
  MAPLAB_POINTER_TYPEDEFS(VIMission);

  VIMission();

  /// Create a new Mission. It does not set the mission root vertex id,
  /// so make sure to set this using mission->setRootVertexId(id) once
  /// the first vertex is created.
  VIMission(
      const MissionId& mission_id,
      const MissionBaseFrameId& mission_base_frame_id, BackBone backbone_type);
  explicit VIMission(const VIMission& other);

  inline bool operator==(const VIMission& lhs) const {
    bool is_same = true;
    is_same &= Mission::operator==(lhs);
    is_same &= type_to_resource_id_map_ == lhs.type_to_resource_id_map_;
    is_same &= ordering_ == lhs.ordering_;
    return is_same;
  }
  inline bool operator!=(const VIMission& lhs) const {
    return !operator==(lhs);
  }

  void setOrdering(int ordering);

  int getOrdering() const;

  void serialize(vi_map::proto::Mission* proto) const;
  void deserialize(
      const vi_map::MissionId& mission_id, const vi_map::proto::Mission& proto);

  std::string getComparisonString(const VIMission& other) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Mission-set-based resources
  // ===========================
  // A set of missions can own a single resource of every type. Like all
  // resources, this data is serialized and deserialzed (with cache) upon
  // accessing the data.
  // NOTE: The functions here are used to manage the resource ids
  // that are associated with this specific mission and should not be called
  // directly. They should only be used by the main accessors for this type of
  // resource in the VIMap.

  void getAllResourceIds(
      const backend::ResourceType& type,
      backend::ResourceIdSet* resource_ids) const;

  void addResourceId(
      const backend::ResourceId& resource_id,
      const backend::ResourceType& type);

  void deleteResourceId(
      const backend::ResourceId& resource_id,
      const backend::ResourceType& type);

  // Optional sensor/camera resources
  // ================================
  // Timestamped resources that are only associated with a mission and an
  // optional sensor or camera with extrinsics (stored in the VIMission). The
  // optional here means that these sensors/cameras are not directly used for
  // the main visual-inertial pose estimation. The main example is a color or
  // depth camera that provides timestamped images/depth maps. These resources
  // need to be timestamped based on a common clock, but not necessarily
  // synchronized with the vertex timestamp. Like all resources, this data is
  // serialized and deserialzed (with cache) upon accessing the data.
  // NOTE: The functions here are used to manage the resource ids
  // that are associated with this specific mission and should not be called
  // directly. They should only be used by the main accessors for this type of
  // resource in the VIMap.

  template <typename SensorId>
  bool hasOptionalSensorResourceId(
      const backend::ResourceType& type, const SensorId& camera_id,
      const int64_t timestamp_ns) const;

  template <typename SensorId>
  bool getOptionalSensorResourceId(
      const backend::ResourceType& type, const SensorId& camera_id,
      const int64_t timestamp_ns, backend::ResourceId* resource_id) const;

  template <typename SensorId>
  bool getClosestOptionalSensorResourceId(
      const backend::ResourceType& type, const SensorId& camera_id,
      const int64_t timestamp_ns, const int64_t tolerance_ns,
      backend::StampedResourceId* stamped_resource_id) const;

  template <typename SensorId>
  bool findAllCloseOptionalSensorResources(
      const backend::ResourceType& type, const int64_t timestamp_ns,
      const int64_t tolerance_ns, std::vector<SensorId>* sensor_ids,
      std::vector<int64_t>* closest_timestamps_ns) const;

  template <typename SensorId>
  const std::unordered_map<
      backend::ResourceType,
      typename std::unordered_map<SensorId, backend::OptionalSensorResources>,
      backend::ResourceTypeHash>&
  getAllOptionalSensorResourceIds() const;

  template <typename SensorId>
  void addOptionalSensorResourceId(
      const backend::ResourceType& type, const SensorId& sensor_id,
      const backend::ResourceId& resource_id, const int64_t timestamp_ns);

  // NOTE: When deleting optional camera resources will not clean up the list of
  // optional cameras.
  template <typename SensorId>
  bool deleteOptionalSensorResourceId(
      const backend::ResourceType& type, const SensorId& camera_id,
      const int64_t timestamp_ns);

  template <typename SensorId>
  const backend::OptionalSensorResources*
  getAllOptionalSensorResourceIdsForSensorOfType(
      const backend::ResourceType& type, const SensorId& sensor_id) const;

  template <typename SensorId>
  backend::OptionalSensorResources*
  getAllOptionalSensorResourceIdsForSensorOfTypeMutable(
      const backend::ResourceType& type, const SensorId& sensor_id);

  template <typename SensorId>
  const std::unordered_map<SensorId, backend::OptionalSensorResources>*
  getAllOptionalSensorResourceIdsOfType(
      const backend::ResourceType& type) const;

  template <typename SensorId>
  std::unordered_map<SensorId, backend::OptionalSensorResources>*
  getAllOptionalSensorResourceIdsOfTypeMutable(
      const backend::ResourceType& type);

 private:
  // A value used to order missions.
  int ordering_;

  backend::ResourceTypeToIdsMap type_to_resource_id_map_;

  backend::ResourceTypeToCameraIdToResourcesMap optional_camera_resources_;
  backend::ResourceTypeToSensorIdToResourcesMap optional_sensor_resources_;
};

typedef std::unordered_map<MissionId, VIMission::UniquePtr> VIMissionMap;
}  //  namespace vi_map

#include "./vi-mission-inl.h"

#endif  // VI_MAP_VI_MISSION_H_
