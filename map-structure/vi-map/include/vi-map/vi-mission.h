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
#include <aslam/common/unique-id.h>
#include <map-resources/resource-common.h>
#include <map-resources/temporal-resource-id-buffer.h>
#include <maplab-common/pose_types.h>

#include "vi-map/mission.h"
#include "vi-map/unique-id.h"
#include "vi-map/vertex.h"

// Forward declarations.
namespace vi_map {
class VIMap;
class VIMissionSensorResourcesTest;
}  // namespace vi_map

namespace vi_map {

class VIMission : public Mission {
  friend class VIMap;
  friend class VIMissionSensorResourcesTest;

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
    is_same &= ordering_ == lhs.ordering_;

    is_same &= ncamera_id_ == lhs.ncamera_id_;
    is_same &= imu_id_ == lhs.imu_id_;
    is_same &= lidar_id_ == lhs.lidar_id_;
    is_same &= pointcloud_map_id_ == lhs.pointcloud_map_id_;
    is_same &= odometry_6dof_id_ == lhs.odometry_6dof_id_;
    is_same &= loop_closure_id_ == lhs.loop_closure_id_;
    is_same &= absolute_6dof_id_ == lhs.absolute_6dof_id_;

    is_same &= type_to_resource_id_map_ == lhs.type_to_resource_id_map_;
    is_same &= sensor_resources_ == lhs.sensor_resources_;

    return is_same;
  }
  inline bool operator!=(const VIMission& lhs) const {
    return !operator==(lhs);
  }

  void setOrdering(int ordering);
  int getOrdering() const;

  void setNCameraId(const aslam::SensorId& ncamera_id);
  void setImuId(const aslam::SensorId& imu_id);
  void setLidarId(const aslam::SensorId& lidar_id);
  void setPointCloudMapId(const aslam::SensorId& pointcloud_map_id);
  void setOdometry6DoFSensor(const aslam::SensorId& odometry_6dof_id);
  void setLoopClosureSensor(const aslam::SensorId& loop_closure_id);
  void setAbsolute6DoFSensor(const aslam::SensorId& absolute_6dof_id);
  void setWheelOdometrySensor(const aslam::SensorId& absolute_6dof_id);

  // NOTE: These functions will check-fail if there is no such sensor associated
  // with the mission. Use the hasNCamera and related functions below to check
  // first!
  const aslam::SensorId& getNCameraId() const;
  const aslam::SensorId& getImuId() const;
  const aslam::SensorId& getLidarId() const;
  const aslam::SensorId& getPointCloudMapSensorId() const;
  const aslam::SensorId& getOdometry6DoFSensor() const;
  const aslam::SensorId& getLoopClosureSensor() const;
  const aslam::SensorId& getAbsolute6DoFSensor() const;
  const aslam::SensorId& getWheelOdometrySensor() const;

  bool hasNCamera() const;
  bool hasImu() const;
  bool hasLidar() const;
  bool hasPointCloudMap() const;
  bool hasOdometry6DoFSensor() const;
  bool hasLoopClosureSensor() const;
  bool hasAbsolute6DoFSensor() const;
  bool hasWheelOdometrySensor() const;

  void serialize(vi_map::proto::Mission* proto) const;
  void deserialize(
      const vi_map::MissionId& mission_id, const vi_map::proto::Mission& proto);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // IDs of mission-set-associated resources
  // =======================================
  // A set of missions can own a single resource of every type. Like all
  // resources, this data is serialized and deserialzed (with cache) upon
  // accessing the data.
  // NOTE: Do not use these functions directly to modify the resource
  // bookkeeping, use the interface in the VIMap instead.
  void getAllMissionResourceIds(
      const backend::ResourceType& type,
      backend::ResourceIdSet* resource_ids) const;

  void addMissionResourceId(
      const backend::ResourceId& resource_id,
      const backend::ResourceType& type);

  void deleteMissionResourceId(
      const backend::ResourceId& resource_id,
      const backend::ResourceType& type);

  // IDs of timestamp/sensor-associated resources
  // ============================================
  // Timestamped resources that are only associated with a mission and a sensor.
  // These resources need to be timestamped based on a common clock, but not
  // necessarily synchronized with the vertex timestamp. Like all resources,
  // this data is serialized and deserialzed (with cache) upon accessing the
  // data.
  // NOTE: Do not use these functions directly to modify the resource
  // bookkeeping, use the interface in the VIMap instead.

  bool hasSensorResource(const backend::ResourceType& type) const;

  bool hasSensorResourceId(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id,
      const int64_t timestamp_ns) const;

  bool getSensorResourceId(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id,
      const int64_t timestamp_ns, backend::ResourceId* resource_id) const;

  bool getClosestSensorResourceId(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id,
      const int64_t timestamp_ns, const int64_t tolerance_ns,
      backend::StampedResourceId* stamped_resource_id) const;

  bool findAllCloseSensorResources(
      const backend::ResourceType& type, const int64_t timestamp_ns,
      const int64_t tolerance_ns, std::vector<aslam::SensorId>* sensor_ids,
      std::vector<int64_t>* closest_timestamps_ns) const;

  const std::unordered_map<
      backend::ResourceType,
      typename std::unordered_map<
          aslam::SensorId, backend::TemporalResourceIdBuffer>,
      backend::ResourceTypeHash>&
  getAllSensorResourceIds() const;
  std::unordered_map<
      backend::ResourceType,
      typename std::unordered_map<
          aslam::SensorId, backend::TemporalResourceIdBuffer>,
      backend::ResourceTypeHash>&
  getAllSensorResourceIds();

  const backend::TemporalResourceIdBuffer*
  getAllSensorResourceIdsForSensorOfType(
      const backend::ResourceType& type,
      const aslam::SensorId& sensor_id) const;

  void addSensorResourceId(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id,
      const backend::ResourceId& resource_id, const int64_t timestamp_ns);

  bool deleteSensorResourceId(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id,
      const int64_t timestamp_ns);

  backend::TemporalResourceIdBuffer*
  getAllSensorResourceIdsForSensorOfTypeMutable(
      const backend::ResourceType& type, const aslam::SensorId& sensor_id);

  const std::unordered_map<aslam::SensorId, backend::TemporalResourceIdBuffer>*
  getAllSensorResourceIdsOfType(const backend::ResourceType& type) const;

  std::unordered_map<aslam::SensorId, backend::TemporalResourceIdBuffer>*
  getAllSensorResourceIdsOfTypeMutable(const backend::ResourceType& type);

  void mergeAllSensorResources(const vi_map::VIMission& other);

 private:
  // A value used to order missions.
  int ordering_;

  aslam::SensorId ncamera_id_;
  aslam::SensorId imu_id_;
  aslam::SensorId lidar_id_;
  aslam::SensorId pointcloud_map_id_;
  aslam::SensorId odometry_6dof_id_;
  aslam::SensorId loop_closure_id_;
  aslam::SensorId absolute_6dof_id_;
  aslam::SensorId wheel_odometry_id_;

  // This container stores a resource type to resource id mapping. It is uses to
  // associate resources with a set of missions.
  backend::ResourceTypeToIdsMap type_to_resource_id_map_;

  // Sensor resources can be any data type supported by the resource system,
  // which is associated with a sensor and a timestamp. This member here only
  // associates the resource id with the mission while the data is stored in the
  // resource system, i.e. to disk (with caching).
  backend::ResourceTypeToSensorIdToResourcesMap sensor_resources_;
};

typedef std::unordered_map<MissionId, VIMission::UniquePtr> VIMissionMap;
}  //  namespace vi_map

#include "./vi-mission-inl.h"

#endif  // VI_MAP_VI_MISSION_H_
