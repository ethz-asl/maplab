#include "vi-map/sensor-manager.h"

#include <aslam-serialization/camera-serialization.h>

namespace vi_map {

constexpr char kYamlFieldNameSensors[] = "sensors";
constexpr char kYamlFieldNameExtrinsics[] = "extrinsics";
constexpr char kYamlFieldNameSensorId[] = "sensor_id";
constexpr char kYamlFieldNameBaseSensorId[] = "base_sensor_id";
constexpr char kYamlFieldNameT_B_S[] = "T_B_S";
constexpr char kYamlFieldNameT_S_B[] = "T_S_B";

SensorManager::SensorManager(const SensorManager& other)
    : base_sensor_id_map_(other.base_sensor_id_map_),
      T_B_S_map_(other.T_B_S_map_) {
  for (const auto& sensor_with_id : other.sensors_) {
    sensors_.emplace(
        sensor_with_id.first, sensor_with_id.second->cloneAsSensor());
  }
}

void SensorManager::merge(const SensorManager& other) {
  aslam::SensorIdSet non_base_sensor_ids;
  for (const auto& sensor_with_id : other.sensors_) {
    const aslam::SensorId& sensor_id = sensor_with_id.first;

    // If there are matching sensor ids, check the sensors are completely equal.
    if (hasSensor(sensor_id)) {
      CHECK(getSensor<aslam::Sensor>(sensor_id).isEqual(
          *sensor_with_id.second, true /*verbose*/))
          << "Sensor id " << sensor_id << " appears in both sensor managers, "
          << "but is a different sensor. This case is not handled when merging "
          << "sensor managers.";

      double T_B_S_diff =
          (getSensor_T_B_S(sensor_id).getTransformationMatrix() -
           other.getSensor_T_B_S(sensor_id).getTransformationMatrix())
              .cwiseAbs()
              .maxCoeff();
      CHECK(
          isBaseSensor(sensor_id) == other.isBaseSensor(sensor_id) &&
          T_B_S_diff < aslam::common::macros::kEpsilon)
          << "Sensors with id " << sensor_id << " are identical, but have"
          << "different extrinsic calibrations. This case is not handled when"
          << " merging sensor managers.";
      continue;
    }

    // First only add base sensors.
    if (other.isBaseSensor(sensor_id)) {
      addSensorAsBase<aslam::Sensor>(sensor_with_id.second->cloneAsSensor());
    } else {
      non_base_sensor_ids.emplace(sensor_id);
    }
  }

  // Afterwards add the rest of the sensors.
  for (const auto& sensor_id : non_base_sensor_ids) {
    addSensor<aslam::Sensor>(
        other.getSensor<aslam::Sensor>(sensor_id).cloneAsSensor(),
        common::getChecked(other.base_sensor_id_map_, sensor_id),
        common::getChecked(other.T_B_S_map_, sensor_id));
  }
}

void SensorManager::swap(SensorManager* other) {
  sensors_.swap(other->sensors_);
  base_sensor_id_map_.swap(other->base_sensor_id_map_);
  T_B_S_map_.swap(other->T_B_S_map_);
}

void SensorManager::removeSensor(const aslam::SensorId& sensor_id) {
  CHECK(hasSensor(sensor_id));

  if (isBaseSensor(sensor_id)) {
    for (const auto& base_id_with_sensor_id : base_sensor_id_map_) {
      CHECK(sensor_id != base_id_with_sensor_id.second)
          << "Attempting to remove base sensor " << sensor_id
          << ", but there are still sensors based on this sensor";
    }
  }

  CHECK_GT(sensors_.erase(sensor_id), 0u);
  CHECK_GT(base_sensor_id_map_.erase(sensor_id), 0u);
  CHECK_GT(T_B_S_map_.erase(sensor_id), 0u);
}

void SensorManager::getAllSensorIds(aslam::SensorIdSet* all_sensor_ids) const {
  CHECK_NOTNULL(all_sensor_ids)->clear();
  for (const auto& sensor_with_id : sensors_) {
    all_sensor_ids->emplace(sensor_with_id.first);
  }
}

void SensorManager::getAllSensorIdsOfType(
    const SensorType& sensor_type, aslam::SensorIdSet* all_sensor_ids) const {
  CHECK_NOTNULL(all_sensor_ids)->clear();
  CHECK(isValidSensorType(sensor_type));
  for (const auto& sensor_with_id : sensors_) {
    if (sensor_with_id.second->getSensorType() == sensor_type) {
      all_sensor_ids->emplace(sensor_with_id.first);
    }
  }
}

bool SensorManager::hasSensor(const aslam::SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return sensors_.count(sensor_id) > 0u;
}

bool SensorManager::hasSensorOfType(
    const vi_map::SensorType& sensor_type) const {
  CHECK(isValidSensorType(sensor_type));
  aslam::SensorIdSet all_sensor_ids_if_type;
  getAllSensorIdsOfType(sensor_type, &all_sensor_ids_if_type);
  return all_sensor_ids_if_type.size() > 0u;
}

SensorType SensorManager::getSensorType(
    const aslam::SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return static_cast<SensorType>(
      getSensor<aslam::Sensor>(sensor_id).getSensorType());
}

bool SensorManager::isBaseSensor(const aslam::SensorId& base_sensor_id) const {
  return getBaseSensorId(base_sensor_id) == base_sensor_id;
}

const aslam::SensorId& SensorManager::getBaseSensorId(
    const aslam::SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  CHECK(hasSensor(sensor_id));
  return common::getChecked(base_sensor_id_map_, sensor_id);
}

bool SensorManager::getBaseSensorIdIfUnique(aslam::SensorId* sensor_id) const {
  CHECK_NOTNULL(sensor_id);

  if (base_sensor_id_map_.empty()) {
    return false;
  }

  bool is_unique = true;
  aslam::SensorId unique_base_id;
  for (auto sensor_id_to_base_id : base_sensor_id_map_) {
    if (!unique_base_id.isValid()) {
      unique_base_id = sensor_id_to_base_id.second;
    } else {
      if (unique_base_id != sensor_id_to_base_id.second) {
        return false;
      }
    }
  }
  CHECK(unique_base_id.isValid());
  *sensor_id = unique_base_id;
  return true;
}

void SensorManager::setSensor_T_B_S(
    const aslam::SensorId& sensor_id, const aslam::Transformation& T_B_S) {
  CHECK(sensor_id.isValid());
  CHECK(hasSensor(sensor_id));
  T_B_S_map_[sensor_id] = T_B_S;
}

void SensorManager::setSensor_T_B_S(
    const aslam::SensorId& sensor_id, const aslam::Position3D& p_B_S) {
  aslam::Transformation T_B_S;
  T_B_S.setIdentity();
  T_B_S.getPosition() = p_B_S;
  setSensor_T_B_S(sensor_id, T_B_S);
}

const aslam::Transformation& SensorManager::getSensor_T_B_S(
    const aslam::SensorId& sensor_id) const {
  CHECK(sensor_id.isValid());
  return common::getChecked(T_B_S_map_, sensor_id);
}

aslam::Transformation& SensorManager::getSensor_T_B_S(
    const aslam::SensorId& sensor_id) {
  CHECK(sensor_id.isValid());
  return common::getChecked(T_B_S_map_, sensor_id);
}

void SensorManager::serialize(YAML::Node* yaml_node) const {
  CHECK_NOTNULL(yaml_node);

  YAML::Node sensors_node;
  for (const auto& sensor_with_id : sensors_) {
    CHECK(sensor_with_id.second);
    CHECK_EQ(sensor_with_id.first, sensor_with_id.second->getId());
    YAML::Node sensor_node;
    sensor_with_id.second->serialize(&sensor_node);

    sensors_node.push_back(sensor_node);
  }
  (*yaml_node)[static_cast<std::string>(kYamlFieldNameSensors)] = sensors_node;

  YAML::Node extrinsics_node;
  for (const auto& base_id_with_sensor_id : base_sensor_id_map_) {
    const aslam::SensorId& sensor_id = base_id_with_sensor_id.first;
    const aslam::SensorId& base_sensor_id = base_id_with_sensor_id.second;
    CHECK_GT(T_B_S_map_.count(sensor_id), 0u);
    YAML::Node extrinsic_node;
    extrinsic_node[static_cast<std::string>(kYamlFieldNameSensorId)] =
        sensor_id.hexString();
    extrinsic_node[static_cast<std::string>(kYamlFieldNameBaseSensorId)] =
        base_sensor_id.hexString();
    extrinsic_node[static_cast<std::string>(kYamlFieldNameT_B_S)] =
        YAML::convert<Eigen::Matrix4d>::encode(
            getSensor_T_B_S(sensor_id).getTransformationMatrix());

    extrinsics_node.push_back(extrinsic_node);
  }
  (*yaml_node)[static_cast<std::string>(kYamlFieldNameExtrinsics)] =
      extrinsics_node;
}

bool SensorManager::deserialize(const YAML::Node& yaml_node) {
  CHECK(yaml_node.IsDefined());
  CHECK(yaml_node.IsMap());

  aslam::SensorIdSet base_sensor_ids;
  AlignedUnorderedMap<aslam::SensorId, aslam::SensorId> base_sensor_id_map;
  AlignedUnorderedMap<aslam::SensorId, aslam::Transformation> T_B_S_map;

  const YAML::Node extrinsics_node =
      yaml_node[static_cast<std::string>(kYamlFieldNameExtrinsics)];
  if (extrinsics_node.IsDefined() && !extrinsics_node.IsNull()) {
    CHECK(extrinsics_node.IsSequence());
    for (const YAML::Node& extrinsic_node : extrinsics_node) {
      CHECK(!extrinsic_node.IsNull());

      std::string id_as_string;
      if (!YAML::safeGet(
              extrinsic_node, static_cast<std::string>(kYamlFieldNameSensorId),
              &id_as_string)) {
        LOG(ERROR) << "Unable to find sensor id for extrinsics. ";
        return false;
      }
      aslam::SensorId sensor_id;
      CHECK(sensor_id.fromHexString(id_as_string));

      if (!YAML::safeGet(
              extrinsic_node,
              static_cast<std::string>(kYamlFieldNameBaseSensorId),
              &id_as_string)) {
        LOG(ERROR) << "Unable to find base sensor id for extrinsics.";
        return false;
      }
      aslam::SensorId base_sensor_id;
      CHECK(base_sensor_id.fromHexString(id_as_string));

      aslam::Transformation T_B_S;
      Eigen::Matrix4d input_matrix;
      if (extrinsic_node[kYamlFieldNameT_B_S]) {
        CHECK(YAML::safeGet(
            extrinsic_node, static_cast<std::string>(kYamlFieldNameT_B_S),
            &input_matrix));
        Eigen::Matrix3d R_B_S = input_matrix.topLeftCorner<3, 3>().eval();
        if (!aslam::Quaternion::isValidRotationMatrix(R_B_S)) {
          CHECK(aslam::Quaternion::isValidRotationMatrix(R_B_S, 1e-4))
              << "The extrinsics for sensor " << sensor_id
              << " do not contain a valid rotation matrix! \nT_B_S: "
              << input_matrix;

          input_matrix.topLeftCorner<3, 3>() =
              aslam::Quaternion::fromApproximateRotationMatrix(R_B_S)
                  .getRotationMatrix();
          LOG(WARNING) << "The extrinsics for sensor " << sensor_id
                       << " do not contain a valid rotation matrix! \nT_B_S: "
                       << input_matrix;
        }

        T_B_S = aslam::Transformation(input_matrix);
      } else if (extrinsic_node[kYamlFieldNameT_S_B]) {
        CHECK(YAML::safeGet(
            extrinsic_node, static_cast<std::string>(kYamlFieldNameT_S_B),
            &input_matrix));
        Eigen::Matrix3d R_S_B = input_matrix.topLeftCorner<3, 3>().eval();
        if (!aslam::Quaternion::isValidRotationMatrix(R_S_B)) {
          CHECK(aslam::Quaternion::isValidRotationMatrix(R_S_B, 1e-4))
              << "The extrinsics for sensor " << sensor_id
              << " do not contain a valid rotation matrix! \nT_S_B: "
              << input_matrix;

          input_matrix.topLeftCorner<3, 3>() =
              aslam::Quaternion::fromApproximateRotationMatrix(R_S_B)
                  .getRotationMatrix();
          LOG(WARNING)
              << "The extrinsics for sensor " << sensor_id
              << " do not contain a valid rotation matrix, approximating with "
              << "nearest orthonormal matrix! \nT_S_B: " << input_matrix;
        }

        T_B_S = aslam::Transformation(input_matrix).inverse();
      } else {
        LOG(ERROR) << "Unable to find T_B_S or T_S_B for extrinsics.";
        return false;
      }

      if (base_sensor_id == sensor_id) {
        // Check that for a base sensor the calibration matrix is identity
        if ((T_B_S.getTransformationMatrix() - Eigen::Matrix4d::Identity())
                .cwiseAbs()
                .maxCoeff() > aslam::common::macros::kEpsilon) {
          LOG(FATAL) << "Sensor " << sensor_id << " of type "
                     << " is a base sensor but the extrinsics associated with "
                     << "it (T_B_S) are not idenity! T_B_S: "
                     << T_B_S.getTransformationMatrix();
        }
        base_sensor_ids.emplace(sensor_id);
      } else {
        base_sensor_id_map.emplace(sensor_id, base_sensor_id);
        T_B_S_map.emplace(sensor_id, T_B_S);
      }
    }
  }

  std::vector<aslam::Sensor::Ptr> non_base_sensors;
  const YAML::Node sensors_node =
      yaml_node[static_cast<std::string>(kYamlFieldNameSensors)];
  if (sensors_node.IsDefined() && !sensors_node.IsNull()) {
    CHECK(sensors_node.IsSequence());
    for (const YAML::Node& sensor_node : sensors_node) {
      CHECK(!sensor_node.IsNull());

      std::string sensor_type_as_string;
      if (YAML::safeGet(
              sensor_node,
              static_cast<std::string>(aslam::kYamlFieldNameSensorType),
              &sensor_type_as_string)) {
      } else {
        std::string sensor_description;
        if (YAML::safeGet(
                sensor_node,
                static_cast<std::string>(aslam::kYamlFieldNameDescription),
                &sensor_description)) {
          LOG(ERROR)
              << "Unable to retrieve sensor type for sensor with description: '"
              << sensor_description << "'!";
        } else {
          LOG(ERROR) << "Unable to retrieve sensor type";
        }
        return false;
      }

      CHECK(!sensor_type_as_string.empty())
          << "The field 'sensor_type' cannot be empty!";
      const SensorType& sensor_type =
          convertStringToSensorType(sensor_type_as_string);
      if (!isValidSensorType(sensor_type)) {
        LOG(ERROR) << "Invalid sensor type " << sensor_type_as_string;
        return false;
      }

      aslam::Sensor::Ptr sensor;
      switch (sensor_type) {
        case SensorType::kNCamera:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<aslam::NCamera>());
          break;
        case SensorType::kImu:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::Imu>());
          break;
        case SensorType::kLidar:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::Lidar>());
          break;
        case SensorType::kOdometry6DoF:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::Odometry6DoF>());
          break;
        case SensorType::kLoopClosureSensor:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::LoopClosureSensor>());
          break;
        case SensorType::kWheelOdometry:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::WheelOdometry>());
          break;
        case SensorType::kAbsolute6DoF:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::Absolute6DoF>());
          break;
        case SensorType::kGpsWgs:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::GpsWgs>());
          break;
        case SensorType::kGpsUtm:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::GpsUtm>());
          break;
        case SensorType::kPointCloudMapSensor:
          sensor = std::dynamic_pointer_cast<aslam::Sensor>(
              aligned_shared<vi_map::PointCloudMapSensor>());
          break;
        case SensorType::kCamera:
          LOG(ERROR) << "Sensor deserialization of a single camera is "
                        "currently not supported, wrap it in an NCamera!";
          return false;
        default:
          LOG(ERROR) << "Sensor deserialization for sensor type " << sensor_type
                     << " is not implemented!";
          return false;
      }

      if (!sensor->deserialize(sensor_node)) {
        LOG(ERROR) << "Failed to deserialize sensor!";
        return false;
      }
      CHECK(sensor->isValid());

      // Only add the base sensors and buffer the other sensors for later
      // addition to guarantee validity of the sensor manager
      if (base_sensor_ids.count(sensor->getId()) > 0u) {
        addSensorAsBase<aslam::Sensor>(sensor);
      } else {
        non_base_sensors.emplace_back(sensor);
      }
    }
  }

  // Add the non base sensors now that all the base sensors have been added
  for (const aslam::Sensor::Ptr& sensor : non_base_sensors) {
    const aslam::SensorId& sensor_id = sensor->getId();

    if (base_sensor_id_map.count(sensor_id) > 0u) {
      addSensor<aslam::Sensor>(
          sensor, common::getChecked(base_sensor_id_map, sensor_id),
          common::getChecked(T_B_S_map, sensor_id));
    } else {
      LOG(ERROR)
          << "All sensors must have a base sensor defined with an associated "
          << "calibration matrix. Missing for sensor " << sensor_id;
      return false;
    }
  }

  return true;
}

size_t SensorManager::getNumSensors() const {
  return sensors_.size();
}

size_t SensorManager::getNumSensorsOfType(
    const vi_map::SensorType& sensor_type) const {
  size_t counter = 0u;
  CHECK(isValidSensorType(sensor_type));
  for (const auto& sensor_with_id : sensors_) {
    if (sensor_with_id.second->getSensorType() == sensor_type) {
      ++counter;
    }
  }
  return counter;
}

bool SensorManager::isEqual(
    const SensorManager& other, const bool verbose) const {
  if (sensors_.size() != other.sensors_.size()) {
    LOG_IF(WARNING, verbose) << "Different sensor size, " << sensors_.size()
                             << " vs " << other.sensors_.size() << "!";
    return false;
  }
  if (base_sensor_id_map_.size() != other.base_sensor_id_map_.size()) {
    LOG_IF(WARNING, verbose) << "Different base sensor id size!";
    return false;
  }
  if (T_B_S_map_.size() != other.T_B_S_map_.size()) {
    LOG_IF(WARNING, verbose) << "Different extrinsics size!";
    return false;
  }

  for (const std::pair<const aslam::SensorId, aslam::Sensor::Ptr> sensor_pair :
       sensors_) {
    const aslam::Sensor::Ptr& this_sensor = sensor_pair.second;
    auto it = other.sensors_.find(sensor_pair.first);
    if (it == other.sensors_.end()) {
      LOG_IF(WARNING, verbose) << "Sensor " << this_sensor->getId()
                               << " is not in the other sensor manager!";
      return false;
    }
    const aslam::Sensor::Ptr& other_sensor = it->second;

    CHECK(this_sensor);
    CHECK(other_sensor);

    if (!this_sensor->isEqual(*other_sensor, verbose)) {
      LOG_IF(WARNING, verbose)
          << "Sensor " << this_sensor->getId()
          << " is not the same the sensor in the other sensor manager!";
      return false;
    }
  }

  for (const std::pair<const aslam::SensorId, aslam::SensorId>
           base_sensor_pair : base_sensor_id_map_) {
    const aslam::SensorId& this_base_sensor_id = base_sensor_pair.second;
    auto it = other.base_sensor_id_map_.find(base_sensor_pair.first);
    if (it == other.base_sensor_id_map_.end()) {
      LOG_IF(WARNING, verbose)
          << "There are no base sensor id entry for sensor "
          << this_base_sensor_id << " in the other other sensor manager!";
      return false;
    }
    const aslam::SensorId& other_base_sensor_id = it->second;

    if (this_base_sensor_id != other_base_sensor_id) {
      LOG_IF(WARNING, verbose)
          << "The base sensor id entry for sensor " << this_base_sensor_id
          << "is different in the other sensor manager!";
      return false;
    }
  }

  for (const std::pair<const aslam::SensorId, aslam::Transformation>
           transformation_pair : T_B_S_map_) {
    const aslam::Transformation& this_transformation =
        transformation_pair.second;
    auto it = other.T_B_S_map_.find(transformation_pair.first);
    if (it == other.T_B_S_map_.end()) {
      LOG_IF(WARNING, verbose)
          << "There are no extrinsics for sensor " << transformation_pair.first
          << " in the other other sensor manager!";
      return false;
    }
    const aslam::Transformation& other_transformation = it->second;

    if ((this_transformation.getTransformationMatrix() -
         other_transformation.getTransformationMatrix())
            .cwiseAbs()
            .maxCoeff() > aslam::common::macros::kEpsilon) {
      LOG_IF(WARNING, verbose)
          << "The extrinsics for sensor " << transformation_pair.first
          << " are different in the other other sensor manager! this T_B_S: "
          << this_transformation << " vs other T_B_S: " << other_transformation;
      return false;
    }
  }
  return true;
}

bool SensorManager::operator==(const SensorManager& other) const {
  return isEqual(other, false /*verbose*/);
}

bool SensorManager::operator!=(const SensorManager& other) const {
  return !operator==(other);
}

}  // namespace vi_map
