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

int VIMission::getOrdering() const {
  return ordering_;
}

void VIMission::setOrdering(int ordering) {
  ordering_ = ordering;
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

  // Serialize optional cameras and optional camera resources associated with
  // this mission.
  for (const backend::OptionalCameraMap::value_type& id_camera_pair :
       optional_cameras_) {
    const aslam::CameraId& camera_id = id_camera_pair.first;
    const backend::CameraWithExtrinsics& cam_w_extrinsics =
        id_camera_pair.second;
    const aslam::Transformation& T_C_B = cam_w_extrinsics.first;

    ::opt_cam_res::proto::CamerasWithExtrinsics* cam_proto =
        proto->add_optional_cameras_with_extrinsics();
    CHECK_NOTNULL(cam_proto);

    // Serialize id and extrinsics.
    ::common::aslam_id_proto::serialize(
        camera_id, cam_proto->mutable_camera_id());
    common::eigen_proto::serialize(T_C_B, cam_proto->mutable_t_c_b());

    // Serialize camera if available.
    if (cam_w_extrinsics.second != nullptr) {
      const aslam::Camera& camera = *cam_w_extrinsics.second;
      aslam::serialization::serializeCamera(
          camera, cam_proto->mutable_camera());
    }
  }
  for (const backend::ResourceTypeToOptionalCameraResourcesMap::value_type&
           type_to_res : optional_sensor_resources_) {
    const backend::ResourceType resource_type = type_to_res.first;
    for (const backend::OptionalCameraResourcesMap::value_type& cam_to_res :
         type_to_res.second) {
      const aslam::CameraId& camera_id = cam_to_res.first;
      const backend::StampedResourceIds& stamped_resources =
          cam_to_res.second.resource_id_map();

      for (const backend::StampedResourceId& stamped_resource :
           stamped_resources) {
        const int64_t timestamp_ns = stamped_resource.first;
        const backend::ResourceId& resource_id = stamped_resource.second;

        ::opt_cam_res::proto::OptionalCameraResources* opt_res_proto =
            proto->add_optional_camera_resources();
        CHECK_NOTNULL(opt_res_proto);

        // Fill in values.
        opt_res_proto->set_timestamp_ns(timestamp_ns);
        ::common::aslam_id_proto::serialize(
            camera_id, opt_res_proto->mutable_camera_id());
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

  const size_t num_mission_resources = proto.mission_resource_ids_size();
  CHECK_EQ(num_mission_resources, proto.mission_resource_types_size());
  for (size_t proto_idx = 0u; proto_idx < num_mission_resources; ++proto_idx) {
    backend::ResourceId resource_id;
    backend::ResourceType type;

    // Retrieve data from proto.
    resource_id.deserialize(proto.mission_resource_ids(proto_idx));
    type = static_cast<backend::ResourceType>(
        proto.mission_resource_types(proto_idx));

    // Store in mission.
    type_to_resource_id_map_[type].insert(resource_id);
  }

  const size_t num_opt_cameras = proto.optional_cameras_with_extrinsics_size();
  for (size_t proto_idx = 0u; proto_idx < num_opt_cameras; ++proto_idx) {
    aslam::CameraId camera_id;
    aslam::Transformation T_C_B;
    aslam::Camera::Ptr camera_ptr;

    // Retrieve data from proto.
    const ::opt_cam_res::proto::CamerasWithExtrinsics& cam_w_extrinsics =
        proto.optional_cameras_with_extrinsics(proto_idx);
    ::common::aslam_id_proto::deserialize(
        cam_w_extrinsics.camera_id(), &camera_id);
    CHECK(camera_id.isValid());
    common::eigen_proto::deserialize(cam_w_extrinsics.t_c_b(), &T_C_B);

    if (cam_w_extrinsics.has_camera()) {
      aslam::serialization::deserializeCamera(
          cam_w_extrinsics.camera(), &camera_ptr);
      CHECK(camera_ptr != nullptr);
    }

    // Store in mission.
    if (camera_ptr) {
      addOptionalCameraWithExtrinsics(*camera_ptr, T_C_B);
    } else {
      LOG(FATAL) << "OptionalCameraResources without a camera model are "
                    "currently not supported!";
    }
  }

  const size_t num_opt_cam_resources = proto.optional_camera_resources_size();
  for (size_t proto_idx = 0u; proto_idx < num_opt_cam_resources; ++proto_idx) {
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

    // Make sure this camera already exists.
    CHECK(hasOptionalCameraWithExtrinsics(camera_id))
        << "Optional camera " << camera_id << " does not exist.";

    // Store in mission.
    addOptionalCameraResourceId(
        resource_type, camera_id, resource_id, timestamp_ns);
  }
}

void VIMission::deserializeDeprecated(
    const vi_map::MissionId& mission_id,
    const vi_map_deprecated::proto::Mission& proto) {
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

  const int num_opt_cameras = proto.optional_cameras_with_extrinsics_size();
  for (int proto_idx = 0; proto_idx < num_opt_cameras; ++proto_idx) {
    aslam::CameraId camera_id;
    aslam::Transformation T_C_B;
    aslam::Camera::Ptr camera_ptr;

    // Retrieve data from proto.
    const ::opt_cam_res::proto::CamerasWithExtrinsics& cam_w_extrinsics =
        proto.optional_cameras_with_extrinsics(proto_idx);
    ::common::aslam_id_proto::deserialize(
        cam_w_extrinsics.camera_id(), &camera_id);
    CHECK(camera_id.isValid());
    common::eigen_proto::deserialize(cam_w_extrinsics.t_c_b(), &T_C_B);

    if (cam_w_extrinsics.has_camera()) {
      aslam::serialization::deserializeCamera(
          cam_w_extrinsics.camera(), &camera_ptr);
      CHECK(camera_ptr != nullptr);
    }

    // Store in mission.
    if (camera_ptr) {
      addOptionalCameraWithExtrinsics(*camera_ptr, T_C_B);
    } else {
      LOG(FATAL) << "OptionalCameraResources without a camera model are "
                    "currently not supported!";
    }
  }

  const size_t num_opt_cam_resources = proto.optional_camera_resources_size();
  for (size_t proto_idx = 0u; proto_idx < num_opt_cam_resources; ++proto_idx) {
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

    // Make sure this camera already exists.
    CHECK(hasOptionalCameraWithExtrinsics(camera_id))
        << "Optional camera " << camera_id << " does not exist.";

    // Store in mission.
    addOptionalCameraResourceId(
        resource_type, camera_id, resource_id, timestamp_ns);
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

bool VIMission::hasOptionalCameraResourceId(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const int64_t timestamp_ns) const {
  const backend::OptionalSensorResources* optional_resources =
      getAllOptionalCameraResourceIdsForCameraOfType(type, camera_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->hasResourceId(timestamp_ns);
}

bool VIMission::getOptionalCameraResourceId(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const int64_t timestamp_ns, backend::ResourceId* resource_id) const {
  CHECK_NOTNULL(resource_id);
  CHECK_GE(timestamp_ns, 0);

  const backend::OptionalSensorResources* optional_resources =
      getAllOptionalCameraResourceIdsForCameraOfType(type, camera_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->getResourceId(timestamp_ns, resource_id);
}

bool VIMission::getClosestOptionalCameraResourceId(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const int64_t timestamp_ns, const int64_t tolerance_ns,
    backend::StampedResourceId* stamped_resource_id) const {
  CHECK_NOTNULL(stamped_resource_id);
  CHECK_GE(timestamp_ns, 0);

  const backend::OptionalSensorResources* optional_resources =
      getAllOptionalCameraResourceIdsForCameraOfType(type, camera_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->getClosestResourceId(
      timestamp_ns, tolerance_ns, stamped_resource_id);
}

void VIMission::addOptionalCameraResourceId(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const backend::ResourceId& resource_id, const int64_t timestamp_ns) {
  CHECK_GE(timestamp_ns, 0);

  backend::OptionalSensorResources& optional_sensor_resources =
      optional_sensor_resources_[type][camera_id];
  CHECK(!optional_sensor_resources.hasResourceId(timestamp_ns))
      << "Cannot store two resources of the same type ("
      << static_cast<int>(type) << "), camera (" << camera_id
      << ") and timestamp (" << timestamp_ns << ")!";

  CHECK_GT(optional_cameras_.count(camera_id), 0u)
      << "Camera with an ID " << camera_id
      << " does not exist. Add this camera first.";

  optional_sensor_resources.addResourceId(timestamp_ns, resource_id);
}

bool VIMission::deleteOptionalCameraResourceId(
    const backend::ResourceType& type, const aslam::CameraId& camera_id,
    const int64_t timestamp_ns) {
  CHECK_GE(timestamp_ns, 0);

  backend::OptionalSensorResources* optional_resources =
      getAllOptionalCameraResourceIdsForCameraOfTypeMutable(type, camera_id);
  if (optional_resources == nullptr) {
    return false;
  }

  return optional_resources->deleteResourceId(timestamp_ns);
}

const backend::OptionalSensorResources*
VIMission::getAllOptionalCameraResourceIdsForCameraOfType(
    const backend::ResourceType& type, const aslam::CameraId& camera_id) const {
  const backend::OptionalCameraResourcesMap* optional_camera_resources_of_type =
      getAllOptionalCameraResourceIdsOfType(type);
  if (optional_camera_resources_of_type == nullptr) {
    return nullptr;
  }

  backend::OptionalCameraResourcesMap::const_iterator camera_it =
      optional_camera_resources_of_type->find(camera_id);
  if (camera_it == optional_camera_resources_of_type->cend()) {
    return nullptr;
  }

  return &(camera_it->second);
}

backend::OptionalSensorResources*
VIMission::getAllOptionalCameraResourceIdsForCameraOfTypeMutable(
    const backend::ResourceType& type, const aslam::CameraId& camera_id) {
  backend::OptionalCameraResourcesMap* optional_camera_resources_of_type =
      getAllOptionalCameraResourceIdsOfTypeMutable(type);
  if (optional_camera_resources_of_type == nullptr) {
    return nullptr;
  }

  backend::OptionalCameraResourcesMap::iterator camera_it =
      optional_camera_resources_of_type->find(camera_id);
  if (camera_it == optional_camera_resources_of_type->end()) {
    return nullptr;
  }

  return &(camera_it->second);
}

const backend::OptionalCameraResourcesMap*
VIMission::getAllOptionalCameraResourceIdsOfType(
    const backend::ResourceType& type) const {
  backend::ResourceTypeToOptionalCameraResourcesMap::const_iterator type_it =
      optional_sensor_resources_.find(type);
  if (type_it == optional_sensor_resources_.cend()) {
    return nullptr;
  }

  return &(type_it->second);
}

backend::OptionalCameraResourcesMap*
VIMission::getAllOptionalCameraResourceIdsOfTypeMutable(
    const backend::ResourceType& type) {
  backend::ResourceTypeToOptionalCameraResourcesMap::iterator type_it =
      optional_sensor_resources_.find(type);
  if (type_it == optional_sensor_resources_.end()) {
    return nullptr;
  }

  return &(type_it->second);
}

void VIMission::addOptionalCameraWithExtrinsics(
    const aslam::Camera& camera, const aslam::Transformation& T_C_B) {
  const aslam::CameraId& camera_id = camera.getId();
  CHECK(!hasOptionalCameraWithExtrinsics(camera_id));
  backend::CameraWithExtrinsics& camera_w_extrinsics =
      optional_cameras_[camera_id];
  camera_w_extrinsics.first = T_C_B;
  camera_w_extrinsics.second.reset(camera.clone());
}

const backend::CameraWithExtrinsics& VIMission::getOptionalCameraWithExtrinsics(
    const aslam::CameraId& camera_id) const {
  CHECK(camera_id.isValid());
  backend::OptionalCameraMap::const_iterator it =
      optional_cameras_.find(camera_id);
  CHECK(it != optional_cameras_.end());
  return it->second;
}

backend::CameraWithExtrinsics& VIMission::getOptionalCameraWithExtrinsics(
    const aslam::CameraId& camera_id) {
  CHECK(camera_id.isValid());
  backend::OptionalCameraMap::iterator it = optional_cameras_.find(camera_id);
  CHECK(it != optional_cameras_.end());
  return it->second;
}

bool VIMission::hasOptionalCameraWithExtrinsics(
    const aslam::CameraId& camera_id) const {
  CHECK(camera_id.isValid());
  return (optional_cameras_.count(camera_id) > 0u);
}

const backend::ResourceTypeToOptionalCameraResourcesMap&
VIMission::getAllOptionalCameraResourceIds() const {
  return optional_sensor_resources_;
}

const backend::OptionalCameraMap&
VIMission::getAllOptionalCamerasWithExtrinsics() const {
  return optional_cameras_;
}

bool VIMission::findAllCloseOptionalCameraResources(
    const backend::ResourceType& type, const int64_t timestamp_ns,
    const int64_t tolerance_ns, aslam::CameraIdList* camera_ids,
    std::vector<int64_t>* closest_timestamps_ns) const {
  CHECK_NOTNULL(camera_ids)->clear();
  CHECK_NOTNULL(closest_timestamps_ns)->clear();

  const backend::OptionalCameraResourcesMap* optional_resources =
      getAllOptionalCameraResourceIdsOfType(type);

  if (optional_resources == nullptr) {
    return false;
  }

  size_t num_resources_found = 0u;
  for (const backend::OptionalCameraResourcesMap::value_type&
           resources_per_camera : *optional_resources) {
    const aslam::CameraId& camera_id = resources_per_camera.first;
    const backend::OptionalSensorResources& stamped_resource_buffer =
        resources_per_camera.second;
    backend::StampedResourceId stamped_resource_id;
    if (!stamped_resource_buffer.getClosestResourceId(
            timestamp_ns, tolerance_ns, &stamped_resource_id)) {
      continue;
    }

    camera_ids->push_back(camera_id);
    closest_timestamps_ns->push_back(stamped_resource_id.first);
    ++num_resources_found;
  }

  return num_resources_found > 0u;
}

const backend::OptionalCameraMap&
VIMission::getOptionalCameraWithExtrinsicsMap() const {
  return optional_cameras_;
}

}  // namespace vi_map
