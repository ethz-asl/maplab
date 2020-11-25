#include "vi-map/vi-mission.h"

#include <ostream>  // NOLINT

#include <aslam-serialization/camera-serialization.h>
#include <aslam/cameras/camera-factory.h>
#include <map-resources/temporal-resource-id-buffer.h>
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
      ncamera_id_(other.ncamera_id_),
      imu_id_(other.imu_id_),
      lidar_id_(other.lidar_id_),
      odometry_6dof_id_(other.odometry_6dof_id_),
      loop_closure_id_(other.loop_closure_id_),
      absolute_6dof_id_(other.absolute_6dof_id_),
      wheel_odometry_id_(other.wheel_odometry_id_),
      type_to_resource_id_map_(other.type_to_resource_id_map_),
      sensor_resources_(other.sensor_resources_),
      robot_name_(other.robot_name_) {}

int VIMission::getOrdering() const {
  return ordering_;
}

void VIMission::setOrdering(int ordering) {
  ordering_ = ordering;
}

void VIMission::setNCameraId(const aslam::SensorId& ncamera_id) {
  CHECK(ncamera_id.isValid());
  ncamera_id_ = ncamera_id;
}

void VIMission::setImuId(const aslam::SensorId& imu_id) {
  CHECK(imu_id.isValid());
  imu_id_ = imu_id;
}

void VIMission::setLidarId(const aslam::SensorId& lidar_id) {
  CHECK(lidar_id.isValid());
  lidar_id_ = lidar_id;
}

void VIMission::setOdometry6DoFSensor(const aslam::SensorId& odometry_6dof_id) {
  CHECK(odometry_6dof_id.isValid());
  odometry_6dof_id_ = odometry_6dof_id;
}

void VIMission::setLoopClosureSensor(const aslam::SensorId& loop_closure_id) {
  CHECK(loop_closure_id.isValid());
  loop_closure_id_ = loop_closure_id;
}

void VIMission::setAbsolute6DoFSensor(const aslam::SensorId& absolute_6dof_id) {
  CHECK(absolute_6dof_id.isValid());
  absolute_6dof_id_ = absolute_6dof_id;
}

void VIMission::setWheelOdometrySensor(
    const aslam::SensorId& wheel_odometry_id) {
  CHECK(wheel_odometry_id.isValid());
  wheel_odometry_id_ = wheel_odometry_id;
}

void VIMission::setRobotName(const std::string& robot_name) {
  CHECK(!robot_name.empty());
  robot_name_ = robot_name;
}

const aslam::SensorId& VIMission::getNCameraId() const {
  CHECK(ncamera_id_.isValid())
      << "There is no valid NCamera ID associated with mission "
      << mission_id_.shortHex() << "!";
  return ncamera_id_;
}

const aslam::SensorId& VIMission::getImuId() const {
  CHECK(imu_id_.isValid())
      << "There is no valid IMU ID associated with mission "
      << mission_id_.shortHex() << "!";
  return imu_id_;
}

const aslam::SensorId& VIMission::getLidarId() const {
  CHECK(lidar_id_.isValid())
      << "There is no valid Lidar ID associated with mission "
      << mission_id_.shortHex() << "!";
  return lidar_id_;
}

const aslam::SensorId& VIMission::getOdometry6DoFSensor() const {
  CHECK(odometry_6dof_id_.isValid())
      << "There is no valid Odometry 6DoF Sensor ID associated with mission "
      << mission_id_.shortHex() << "!";
  return odometry_6dof_id_;
}

const aslam::SensorId& VIMission::getLoopClosureSensor() const {
  CHECK(loop_closure_id_.isValid())
      << "There is no valid Loop Closure Sensor ID associated with mission "
      << mission_id_.shortHex() << "!";
  return loop_closure_id_;
}

const aslam::SensorId& VIMission::getAbsolute6DoFSensor() const {
  CHECK(absolute_6dof_id_.isValid())
      << "There is no valid absolute 6DoF Sensor ID associated with mission "
      << mission_id_.shortHex() << "!";
  return absolute_6dof_id_;
}

const aslam::SensorId& VIMission::getWheelOdometrySensor() const {
  CHECK(wheel_odometry_id_.isValid())
      << "There is no valid wheel odometry Sensor ID associated with mission "
      << mission_id_.shortHex() << "!";
  return wheel_odometry_id_;
}

const std::string& VIMission::getRobotName() const {
  CHECK(!robot_name_.empty());
  return robot_name_;
}

bool VIMission::hasNCamera() const {
  return ncamera_id_.isValid();
}
bool VIMission::hasImu() const {
  return imu_id_.isValid();
}
bool VIMission::hasLidar() const {
  return lidar_id_.isValid();
}
bool VIMission::hasOdometry6DoFSensor() const {
  return odometry_6dof_id_.isValid();
}
bool VIMission::hasLoopClosureSensor() const {
  return loop_closure_id_.isValid();
}
bool VIMission::hasAbsolute6DoFSensor() const {
  return absolute_6dof_id_.isValid();
}
bool VIMission::hasWheelOdometrySensor() const {
  return wheel_odometry_id_.isValid();
}
bool VIMission::hasRobotName() const {
  return !robot_name_.empty();
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

  if (ncamera_id_.isValid()) {
    ncamera_id_.serialize(proto->mutable_ncamera_id());
  }
  if (imu_id_.isValid()) {
    imu_id_.serialize(proto->mutable_imu_id());
  }
  if (lidar_id_.isValid()) {
    lidar_id_.serialize(proto->mutable_lidar_id());
  }
  if (odometry_6dof_id_.isValid()) {
    odometry_6dof_id_.serialize(proto->mutable_odometry_6dof_id());
  }
  if (loop_closure_id_.isValid()) {
    loop_closure_id_.serialize(proto->mutable_loop_closure_id());
  }
  if (absolute_6dof_id_.isValid()) {
    absolute_6dof_id_.serialize(proto->mutable_absolute_6dof_id());
  }
  if (wheel_odometry_id_.isValid()) {
    wheel_odometry_id_.serialize(proto->mutable_wheel_odometry_id());
  }

  // Serialize sensor resources associated with this mission.
  for (const backend::ResourceTypeToSensorIdToResourcesMap::value_type&
           type_to_res : sensor_resources_) {
    const backend::ResourceType resource_type = type_to_res.first;
    for (const backend::SensorIdToResourcesMap::value_type& sen_to_res :
         type_to_res.second) {
      const aslam::SensorId& sensor_id = sen_to_res.first;
      const backend::StampedResourceIds& stamped_resources =
          sen_to_res.second.resource_id_map();

      for (const backend::StampedResourceId& stamped_resource :
           stamped_resources) {
        const int64_t timestamp_ns = stamped_resource.first;
        const backend::ResourceId& resource_id = stamped_resource.second;

        sensor_resources::proto::StampedSensorResourceId* opt_res_proto =
            proto->add_sensor_resources();
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

  if (proto.has_ncamera_id()) {
    ncamera_id_.deserialize(proto.ncamera_id());
  }
  if (proto.has_imu_id()) {
    imu_id_.deserialize(proto.imu_id());
  }
  if (proto.has_lidar_id()) {
    lidar_id_.deserialize(proto.lidar_id());
  }
  if (proto.has_odometry_6dof_id()) {
    odometry_6dof_id_.deserialize(proto.odometry_6dof_id());
  }
  if (proto.has_loop_closure_id()) {
    loop_closure_id_.deserialize(proto.loop_closure_id());
  }
  if (proto.has_absolute_6dof_id()) {
    absolute_6dof_id_.deserialize(proto.absolute_6dof_id());
  }
  if (proto.has_wheel_odometry_id()) {
    wheel_odometry_id_.deserialize(proto.wheel_odometry_id());
  }

  const int num_opt_sen_resources = proto.sensor_resources_size();
  for (int proto_idx = 0; proto_idx < num_opt_sen_resources; ++proto_idx) {
    backend::ResourceId resource_id;
    int64_t timestamp_ns;
    aslam::SensorId sensor_id;
    backend::ResourceType resource_type;

    // Retrieve data from proto.
    const ::sensor_resources::proto::StampedSensorResourceId& resource =
        proto.sensor_resources(proto_idx);
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
    addSensorResourceId(resource_type, sensor_id, resource_id, timestamp_ns);
  }
}

void VIMission::addMissionResourceId(
    const backend::ResourceId& resource_id, const backend::ResourceType& type) {
  type_to_resource_id_map_[type].emplace(resource_id);
}

void VIMission::deleteMissionResourceId(
    const backend::ResourceId& resource_id, const backend::ResourceType& type) {
  backend::ResourceTypeToIdsMap::iterator it =
      type_to_resource_id_map_.find(type);
  CHECK(it != type_to_resource_id_map_.end());
  it->second.erase(resource_id);
}

void VIMission::getAllMissionResourceIds(
    const backend::ResourceType& type,
    backend::ResourceIdSet* resource_ids) const {
  CHECK_NOTNULL(resource_ids)->clear();
  backend::ResourceTypeToIdsMap::const_iterator it =
      type_to_resource_id_map_.find(type);
  if (it != type_to_resource_id_map_.end()) {
    resource_ids->insert(it->second.begin(), it->second.end());
  }
}

bool VIMission::hasSensorResource(const backend::ResourceType& type) const {
  const backend::SensorIdToResourcesMap* resources =
      getAllSensorResourceIdsOfType(type);

  if (resources == nullptr) {
    LOG(WARNING) << "No sensor found of type "
                 << backend::ResourceTypeNames[static_cast<int>(type)];
    return false;
  }
  VLOG(3) << "Found " << resources->size() << " sensors of type "
          << backend::ResourceTypeNames[static_cast<int>(type)];
  return true;
}

bool VIMission::hasSensorResourceId(
    const backend::ResourceType& type, const aslam::SensorId& sensor_id,
    const int64_t timestamp_ns) const {
  const backend::TemporalResourceIdBuffer* resources =
      getAllSensorResourceIdsForSensorOfType(type, sensor_id);
  if (resources == nullptr) {
    return false;
  }

  return resources->hasResourceId(timestamp_ns);
}

bool VIMission::getSensorResourceId(
    const backend::ResourceType& type, const aslam::SensorId& sensor_id,
    const int64_t timestamp_ns, backend::ResourceId* resource_id) const {
  CHECK_NOTNULL(resource_id);
  CHECK_GE(timestamp_ns, 0);

  const backend::TemporalResourceIdBuffer* resources =
      getAllSensorResourceIdsForSensorOfType(type, sensor_id);
  if (resources == nullptr) {
    return false;
  }

  return resources->getResourceId(timestamp_ns, resource_id);
}

bool VIMission::getClosestSensorResourceId(
    const backend::ResourceType& type, const aslam::SensorId& sensor_id,
    const int64_t timestamp_ns, const int64_t tolerance_ns,
    backend::StampedResourceId* stamped_resource_id) const {
  CHECK_NOTNULL(stamped_resource_id);
  CHECK_GE(timestamp_ns, 0);

  const backend::TemporalResourceIdBuffer* resources =
      getAllSensorResourceIdsForSensorOfType(type, sensor_id);
  if (resources == nullptr) {
    return false;
  }

  return resources->getClosestResourceId(
      timestamp_ns, tolerance_ns, stamped_resource_id);
}

bool VIMission::findAllCloseSensorResources(
    const backend::ResourceType& type, const int64_t timestamp_ns,
    const int64_t tolerance_ns, std::vector<aslam::SensorId>* sensor_ids,
    std::vector<int64_t>* closest_timestamps_ns) const {
  CHECK_NOTNULL(sensor_ids)->clear();
  CHECK_NOTNULL(closest_timestamps_ns)->clear();

  const backend::SensorIdToResourcesMap* resources =
      getAllSensorResourceIdsOfType(type);

  if (resources == nullptr) {
    return false;
  }

  size_t num_resources_found = 0u;
  for (const backend::SensorIdToResourcesMap::value_type& resources_per_sensor :
       *resources) {
    const aslam::SensorId& sensor_id = resources_per_sensor.first;
    const backend::TemporalResourceIdBuffer& stamped_resource_buffer =
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

const backend::ResourceTypeToSensorIdToResourcesMap&
VIMission::getAllSensorResourceIds() const {
  return sensor_resources_;
}

backend::ResourceTypeToSensorIdToResourcesMap&
VIMission::getAllSensorResourceIds() {
  return sensor_resources_;
}

void VIMission::addSensorResourceId(
    const backend::ResourceType& type, const aslam::SensorId& sensor_id,
    const backend::ResourceId& resource_id, const int64_t timestamp_ns) {
  CHECK_GE(timestamp_ns, 0);

  backend::TemporalResourceIdBuffer& sensor_resources =
      sensor_resources_[type][sensor_id];
  CHECK(!sensor_resources.hasResourceId(timestamp_ns))
      << "Cannot store two resources of the same type ("
      << static_cast<int>(type) << "), sensor (" << sensor_id
      << ") and timestamp (" << timestamp_ns << ")!";

  sensor_resources.addResourceId(timestamp_ns, resource_id);
}

bool VIMission::deleteSensorResourceId(
    const backend::ResourceType& type, const aslam::SensorId& sensor_id,
    const int64_t timestamp_ns) {
  CHECK_GE(timestamp_ns, 0);

  backend::TemporalResourceIdBuffer* resources =
      getAllSensorResourceIdsForSensorOfTypeMutable(type, sensor_id);
  if (resources == nullptr) {
    return false;
  }

  return resources->deleteResourceId(timestamp_ns);
}

const backend::TemporalResourceIdBuffer*
VIMission::getAllSensorResourceIdsForSensorOfType(
    const backend::ResourceType& type, const aslam::SensorId& sensor_id) const {
  const std::unordered_map<aslam::SensorId, backend::TemporalResourceIdBuffer>*
      sensor_resources_of_type = getAllSensorResourceIdsOfType(type);
  if (sensor_resources_of_type == nullptr) {
    return nullptr;
  }

  typename std::unordered_map<
      aslam::SensorId, backend::TemporalResourceIdBuffer>::const_iterator
      sensor_it = sensor_resources_of_type->find(sensor_id);
  if (sensor_it == sensor_resources_of_type->cend()) {
    return nullptr;
  }

  return &(sensor_it->second);
}

backend::TemporalResourceIdBuffer*
VIMission::getAllSensorResourceIdsForSensorOfTypeMutable(
    const backend::ResourceType& type, const aslam::SensorId& sensor_id) {
  std::unordered_map<aslam::SensorId, backend::TemporalResourceIdBuffer>*
      sensor_resources_of_type = getAllSensorResourceIdsOfTypeMutable(type);
  if (sensor_resources_of_type == nullptr) {
    return nullptr;
  }

  typename std::unordered_map<
      aslam::SensorId, backend::TemporalResourceIdBuffer>::iterator sensor_it =
      sensor_resources_of_type->find(sensor_id);
  if (sensor_it == sensor_resources_of_type->cend()) {
    return nullptr;
  }

  return &(sensor_it->second);
}

const backend::SensorIdToResourcesMap* VIMission::getAllSensorResourceIdsOfType(
    const backend::ResourceType& type) const {
  backend::ResourceTypeToSensorIdToResourcesMap::const_iterator type_it =
      sensor_resources_.find(type);
  if (type_it == sensor_resources_.cend()) {
    return nullptr;
  }

  return &(type_it->second);
}

backend::SensorIdToResourcesMap*
VIMission::getAllSensorResourceIdsOfTypeMutable(
    const backend::ResourceType& type) {
  backend::ResourceTypeToSensorIdToResourcesMap::iterator type_it =
      sensor_resources_.find(type);
  if (type_it == sensor_resources_.cend()) {
    return nullptr;
  }

  return &(type_it->second);
}

void VIMission::mergeAllSensorResources(const vi_map::VIMission& other) {
  const backend::ResourceTypeToSensorIdToResourcesMap&
      resource_types_to_sensor_to_resource_map =
          other.getAllSensorResourceIds();
  for (const auto& resource_type_to_sensor_to_resource_map :
       resource_types_to_sensor_to_resource_map) {
    for (const auto& sensor_to_resource_map :
         resource_type_to_sensor_to_resource_map.second) {
      const backend::TemporalResourceIdBuffer& resource_map =
          sensor_to_resource_map.second;
      sensor_resources_[resource_type_to_sensor_to_resource_map.first]
                       [sensor_to_resource_map.first]
                           .insert(resource_map);
    }
  }
}

}  // namespace vi_map
