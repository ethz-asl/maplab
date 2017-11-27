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

#include "vi-map/deprecated/vi-map-serialization-deprecated.h"
#include "vi-map/mission.h"
#include "vi-map/optional-sensor-data.h"
#include "vi-map/unique-id.h"
#include "vi-map/vertex.h"

namespace vi_map {

// Used to shift vertex timestamps when user-defined mission ordering is used.
const int64_t kFakeNumNanosecondsBetweenMissions = 864e11;  // 1 day.

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
  void deserializeDeprecated(
      const vi_map::MissionId& mission_id,
      const vi_map_deprecated::proto::Mission& proto);

  std::string getComparisonString(const VIMission& other) const;

  void addResourceId(
      const backend::ResourceId& resource_id,
      const backend::ResourceType& type);

  void deleteResourceId(
      const backend::ResourceId& resource_id,
      const backend::ResourceType& type);

  void getAllResourceIds(
      const backend::ResourceType& type,
      backend::ResourceIdSet* resource_ids) const;

  // Optional Camera Resources:
  // Resources that are associated with a aslam::Camera but are not part of the
  // VI-map, i.e. are not used for mapping. For example color images or point
  // clouds that are not recorded at the same time as the frame resources of
  // the map.

  bool hasOptionalCameraResourceId(
      const backend::ResourceType& type, const aslam::CameraId& camera_id,
      const int64_t timestamp_ns) const;

  bool getOptionalCameraResourceId(
      const backend::ResourceType& type, const aslam::CameraId& camera_id,
      const int64_t timestamp_ns, backend::ResourceId* resource_id) const;

  bool getClosestOptionalCameraResourceId(
      const backend::ResourceType& type, const aslam::CameraId& camera_id,
      const int64_t timestamp_ns, const int64_t tolerance_ns,
      backend::StampedResourceId* stamped_resource_id) const;

  void addOptionalCameraResourceId(
      const backend::ResourceType& type, const aslam::CameraId& camera_id,
      const backend::ResourceId& resource_id, const int64_t timestamp_ns);

  // NOTE: When deleting optional camera resources will not clean up the list of
  // optional cameras.
  bool deleteOptionalCameraResourceId(
      const backend::ResourceType& type, const aslam::CameraId& camera_id,
      const int64_t timestamp_ns);

  bool findAllCloseOptionalCameraResources(
      const backend::ResourceType& type, const int64_t timestamp_ns,
      const int64_t tolerance_ns, aslam::CameraIdList* camera_ids,
      std::vector<int64_t>* closest_timestamps_ns) const;

  const backend::CameraWithExtrinsics& getOptionalCameraWithExtrinsics(
      const aslam::CameraId& camera_id) const;

  backend::CameraWithExtrinsics& getOptionalCameraWithExtrinsics(
      const aslam::CameraId& camera_id);

  const backend::OptionalCameraMap& getOptionalCameraWithExtrinsicsMap() const;

  void addOptionalCameraWithExtrinsics(
      const aslam::Camera& camera, const aslam::Transformation& T_C_B);

  bool hasOptionalCameraWithExtrinsics(const aslam::CameraId& camera_id) const;

  const backend::OptionalCameraMap& getAllOptionalCamerasWithExtrinsics() const;

  const backend::ResourceTypeToOptionalCameraResourcesMap&
  getAllOptionalCameraResourceIds() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const backend::OptionalSensorResources*
  getAllOptionalCameraResourceIdsForCameraOfType(
      const backend::ResourceType& type,
      const aslam::CameraId& camera_id) const;

  backend::OptionalSensorResources*
  getAllOptionalCameraResourceIdsForCameraOfTypeMutable(
      const backend::ResourceType& type, const aslam::CameraId& camera_id);

  const backend::OptionalCameraResourcesMap*
  getAllOptionalCameraResourceIdsOfType(
      const backend::ResourceType& type) const;

  backend::OptionalCameraResourcesMap*
  getAllOptionalCameraResourceIdsOfTypeMutable(
      const backend::ResourceType& type);

  // A value used to order missions.
  int ordering_;

  backend::ResourceTypeToIdsMap type_to_resource_id_map_;

  backend::ResourceTypeToOptionalCameraResourcesMap optional_sensor_resources_;
  backend::OptionalCameraMap optional_cameras_;
};

typedef std::unordered_map<MissionId, VIMission::UniquePtr> VIMissionMap;
}  //  namespace vi_map

#endif  // VI_MAP_VI_MISSION_H_
