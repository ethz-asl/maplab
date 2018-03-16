#include "vi-map/vi-mission.h"

#include <ostream>  // NOLINT

#include <aslam-serialization/camera-serialization.h>
#include <aslam/cameras/camera-factory.h>
#include <maplab-common/aslam-id-proto.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/quaternion-math.h>

namespace vi_map {
VIMission::VIMission() : ordering_(-1) {}

VIMission::VIMission(
    const MissionId& mission_id,
    const MissionBaseFrameId& mission_base_frame_id,
    const BackBone backbone_type)
    : Mission(mission_id, mission_base_frame_id, backbone_type),
      ordering_(-1) {}

VIMission::VIMission(const VIMission& other)
    : Mission(other),
      ordering_(other.getOrdering()),
      type_to_resource_id_map_(other.type_to_resource_id_map_),
      optional_camera_resources_(other.optional_camera_resources_),
      optional_sensor_resources_(other.optional_sensor_resources_) {}

int VIMission::getOrdering() const {
  return ordering_;
}

void VIMission::setOrdering(int ordering) {
  ordering_ = ordering;
}

template <>
void VIMission::addOptionalSensorResourceId<aslam::CameraId>(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const backend::ResourceId& resource_id, const int64_t timestamp_ns) {
  CHECK_GE(timestamp_ns, 0);

  backend::OptionalSensorResources& optional_sensor_resources =
      optional_camera_resources_[type][camera_id];
  CHECK(!optional_sensor_resources.hasResourceId(timestamp_ns))
      << "Cannot store two resources of the same type ("
      << static_cast<int>(type) << "), camera (" << camera_id
      << ") and timestamp (" << timestamp_ns << ")!";

  optional_sensor_resources.addResourceId(timestamp_ns, resource_id);
}

template <>
void VIMission::addOptionalSensorResourceId<SensorId>(
    const backend::ResourceType& type, const SensorId& sensor_id,
    const backend::ResourceId& resource_id, const int64_t timestamp_ns) {
  CHECK_GE(timestamp_ns, 0);

  backend::OptionalSensorResources& optional_sensor_resources =
      optional_sensor_resources_[type][sensor_id];
  CHECK(!optional_sensor_resources.hasResourceId(timestamp_ns))
      << "Cannot store two resources of the same type ("
      << static_cast<int>(type) << "), sensor (" << sensor_id
      << ") and timestamp (" << timestamp_ns << ")!";

  optional_sensor_resources.addResourceId(timestamp_ns, resource_id);
}

void VIMission::serialize(vi_map::proto::Mission* proto) const {
  CHECK_NOTNULL(proto);

  Mission::root_vertex_id_.serialize(proto->mutable_root_vertex_id());
  Mission::base_frame_id_.serialize(proto->mutable_baseframe_id());

  switch (backbone_type_) {
    case BackBone::kViwls: {
      proto->set_backbone(::vi_map::proto::Mission_BackBone_kViwls);
      break;
    }
    case BackBone::kOdometry: {
      proto->set_backbone(::vi_map::proto::Mission_BackBone_kOdometry);
      break;
    }
    default: {
      LOG(FATAL) << "Unknown backbone edge value "
                 << static_cast<int>(backbone_type_);
    }
  }

  // Serialize resource ids associated with this mission.
  for (const backend::ResourceTypeToIdsMap::value_type& pair :
       type_to_resource_id_map_) {
    for (const backend::ResourceId& resource_id : pair.second) {
      resource_id.serialize(proto->add_mission_resource_ids());
      proto->add_mission_resource_types(static_cast<int>(pair.first));
    }
  }

  // Serialize optional sensor/camera resources associated with
  // this mission.
  for (const backend::ResourceTypeToCameraIdToResourcesMap::value_type&
           type_to_res : optional_camera_resources_) {
    const backend::ResourceType resource_type = type_to_res.first;
    for (const backend::CameraIdToResourcesMap::value_type& cam_to_res :
         type_to_res.second) {
      const aslam::CameraId& camera_id = cam_to_res.first;
      const backend::StampedResourceIds& stamped_resources =
          cam_to_res.second.resource_id_map();

      for (const backend::StampedResourceId& stamped_resource :
           stamped_resources) {
        const int64_t timestamp_ns = stamped_resource.first;
        const backend::ResourceId& resource_id = stamped_resource.second;

        opt_cam_res::proto::OptionalCameraResources* opt_res_proto =
            proto->add_optional_camera_resources();
        CHECK_NOTNULL(opt_res_proto);

        // Fill in values.
        opt_res_proto->set_timestamp_ns(timestamp_ns);
        common::aslam_id_proto::serialize(
            camera_id, opt_res_proto->mutable_camera_id());
        resource_id.serialize(opt_res_proto->mutable_resource_id());
        opt_res_proto->set_resource_type(static_cast<int32_t>(resource_type));
      }
    }
  }

  for (const backend::ResourceTypeToSensorIdToResourcesMap::value_type&
           type_to_res : optional_sensor_resources_) {
    const backend::ResourceType resource_type = type_to_res.first;
    for (const backend::SensorIdToResourcesMap::value_type& sen_to_res :
         type_to_res.second) {
      const SensorId& sensor_id = sen_to_res.first;
      const backend::StampedResourceIds& stamped_resources =
          sen_to_res.second.resource_id_map();

      for (const backend::StampedResourceId& stamped_resource :
           stamped_resources) {
        const int64_t timestamp_ns = stamped_resource.first;
        const backend::ResourceId& resource_id = stamped_resource.second;

        opt_cam_res::proto::OptionalSensorResources* opt_res_proto =
            proto->add_optional_sensor_resources();
        CHECK_NOTNULL(opt_res_proto);

        // Fill in values.
        opt_res_proto->set_timestamp_ns(timestamp_ns);
        sensor_id.serialize(opt_res_proto->mutable_sensor_id());
        resource_id.serialize(opt_res_proto->mutable_resource_id());
        opt_res_proto->set_resource_type(static_cast<int32_t>(resource_type));
      }
    }
  }
}

void VIMission::deserialize(
    const vi_map::MissionId& mission_id, const vi_map::proto::Mission& proto) {
  CHECK(mission_id.isValid());
  Mission::mission_id_ = mission_id;

  Mission::root_vertex_id_.deserialize(proto.root_vertex_id());
  Mission::base_frame_id_.deserialize(proto.baseframe_id());

  if (proto.has_backbone()) {
    switch (proto.backbone()) {
      case ::vi_map::proto::Mission_BackBone_kViwls: {
        backbone_type_ = BackBone::kViwls;
        break;
      }
      case ::vi_map::proto::Mission_BackBone_kOdometry: {
        backbone_type_ = BackBone::kOdometry;
        break;
      }
      default: {
        LOG(FATAL) << "Unknown backbone type "
                   << static_cast<int>(proto.backbone());
      }
    }
  } else {
    backbone_type_ = BackBone::kViwls;
  }

  const int num_mission_resources = proto.mission_resource_ids_size();
  CHECK_EQ(num_mission_resources, proto.mission_resource_types_size());
  for (int proto_idx = 0; proto_idx < num_mission_resources; ++proto_idx) {
    backend::ResourceId resource_id;
    backend::ResourceType type;

    // Retrieve data from proto.
    resource_id.deserialize(proto.mission_resource_ids(proto_idx));
    type = static_cast<backend::ResourceType>(
        proto.mission_resource_types(proto_idx));

    // Store in mission.
    type_to_resource_id_map_[type].insert(resource_id);
  }

  const int num_opt_cam_resources = proto.optional_camera_resources_size();
  for (int proto_idx = 0; proto_idx < num_opt_cam_resources; ++proto_idx) {
    backend::ResourceId resource_id;
    int64_t timestamp_ns;
    aslam::CameraId camera_id;
    backend::ResourceType resource_type;

    // Retrieve data from proto.
    const ::opt_cam_res::proto::OptionalCameraResources& resource =
        proto.optional_camera_resources(proto_idx);
    resource_id.deserialize(resource.resource_id());
    ::common::aslam_id_proto::deserialize(resource.camera_id(), &camera_id);
    CHECK(camera_id.isValid());
    timestamp_ns = static_cast<int64_t>(resource.timestamp_ns());
    int32_t resource_type_int = static_cast<int32_t>(resource.resource_type());
    CHECK_LT(
        resource_type_int, static_cast<int32_t>(backend::ResourceType::kCount));
    CHECK_GE(resource_type_int, 0);
    resource_type = static_cast<backend::ResourceType>(resource_type_int);

    // Store in mission.
    addOptionalSensorResourceId<aslam::CameraId>(
        resource_type, camera_id, resource_id, timestamp_ns);
  }

  const int num_opt_sen_resources = proto.optional_sensor_resources_size();
  for (int proto_idx = 0; proto_idx < num_opt_sen_resources; ++proto_idx) {
    backend::ResourceId resource_id;
    int64_t timestamp_ns;
    SensorId sensor_id;
    backend::ResourceType resource_type;

    // Retrieve data from proto.
    const ::opt_cam_res::proto::OptionalSensorResources& resource =
        proto.optional_sensor_resources(proto_idx);
    resource_id.deserialize(resource.resource_id());
    sensor_id.deserialize(resource.sensor_id());
    CHECK(sensor_id.isValid());
    timestamp_ns = static_cast<int64_t>(resource.timestamp_ns());
    int32_t resource_type_int = static_cast<int32_t>(resource.resource_type());
    CHECK_LT(
        resource_type_int, static_cast<int32_t>(backend::ResourceType::kCount));
    CHECK_GE(resource_type_int, 0);
    resource_type = static_cast<backend::ResourceType>(resource_type_int);

    // Store in mission.
    addOptionalSensorResourceId<SensorId>(
        resource_type, sensor_id, resource_id, timestamp_ns);
  }
}

std::string VIMission::getComparisonString(const VIMission& other) const {
  if (operator==(other)) {
    return "There is no difference between the given missions.\n";
  }

  std::ostringstream ss;
  if (!Mission::operator==(other)) {
    ss << "The mission bases differ!\n";
  }
  if (type_to_resource_id_map_ != other.type_to_resource_id_map_) {
    ss << "The mission resource ids differ!\n";
  }

  return ss.str();
}

void VIMission::addResourceId(
    const backend::ResourceId& resource_id, const backend::ResourceType& type) {
  type_to_resource_id_map_[type].emplace(resource_id);
}

void VIMission::deleteResourceId(
    const backend::ResourceId& resource_id, const backend::ResourceType& type) {
  backend::ResourceTypeToIdsMap::iterator it =
      type_to_resource_id_map_.find(type);
  CHECK(it != type_to_resource_id_map_.end());
  it->second.erase(resource_id);
}

void VIMission::getAllResourceIds(
    const backend::ResourceType& type,
    backend::ResourceIdSet* resource_ids) const {
  CHECK_NOTNULL(resource_ids)->clear();
  backend::ResourceTypeToIdsMap::const_iterator it =
      type_to_resource_id_map_.find(type);
  if (it != type_to_resource_id_map_.end()) {
    resource_ids->insert(it->second.begin(), it->second.end());
  }
}

template <>
const backend::CameraIdToResourcesMap*
VIMission::getAllOptionalSensorResourceIdsOfType<aslam::CameraId>(
    const backend::ResourceType& type) const {
  backend::ResourceTypeToCameraIdToResourcesMap::const_iterator type_it =
      optional_camera_resources_.find(type);
  if (type_it == optional_camera_resources_.cend()) {
    return nullptr;
  }

  return &(type_it->second);
}

template <>
const backend::SensorIdToResourcesMap*
VIMission::getAllOptionalSensorResourceIdsOfType<SensorId>(
    const backend::ResourceType& type) const {
  backend::ResourceTypeToSensorIdToResourcesMap::const_iterator type_it =
      optional_sensor_resources_.find(type);
  if (type_it == optional_sensor_resources_.cend()) {
    return nullptr;
  }

  return &(type_it->second);
}

template <>
backend::CameraIdToResourcesMap*
VIMission::getAllOptionalSensorResourceIdsOfTypeMutable<aslam::CameraId>(
    const backend::ResourceType& type) {
  backend::ResourceTypeToCameraIdToResourcesMap::iterator type_it =
      optional_camera_resources_.find(type);
  if (type_it == optional_camera_resources_.cend()) {
    return nullptr;
  }

  return &(type_it->second);
}

template <>
backend::SensorIdToResourcesMap*
VIMission::getAllOptionalSensorResourceIdsOfTypeMutable<SensorId>(
    const backend::ResourceType& type) {
  backend::ResourceTypeToSensorIdToResourcesMap::iterator type_it =
      optional_sensor_resources_.find(type);
  if (type_it == optional_sensor_resources_.cend()) {
    return nullptr;
  }

  return &(type_it->second);
}

template <>
const backend::ResourceTypeToSensorIdToResourcesMap&
VIMission::getAllOptionalSensorResourceIds<SensorId>() const {
  return optional_sensor_resources_;
}

template <>
const backend::ResourceTypeToCameraIdToResourcesMap&
VIMission::getAllOptionalSensorResourceIds<aslam::CameraId>() const {
  return optional_camera_resources_;
}

}  // namespace vi_map
