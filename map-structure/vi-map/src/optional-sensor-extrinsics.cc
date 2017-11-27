#include "vi-map/deprecated/optional-sensor-extrinsics.h"

#include <aslam/common/memory.h>
#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/file-system-tools.h>

namespace vi_map {

OptionalSensorExtrinsics::OptionalSensorExtrinsics() {
  id_.setInvalid();
  sensor_type_ = static_cast<OptionalSensorType>(0);
  T_S_I_.setIdentity();
  T_S_I_covariance_.setZero();
}

OptionalSensorExtrinsics::OptionalSensorExtrinsics(
    const SensorId& id, const OptionalSensorType sensor_type,
    const aslam::Transformation& T_S_I,
    const aslam::TransformationCovariance& T_S_I_covariance)
    : id_(id),
      sensor_type_(sensor_type),
      T_S_I_(T_S_I),
      T_S_I_covariance_(T_S_I_covariance) {
  // Check if the covariance matrix is positive definite.
  Eigen::LLT<aslam::TransformationCovariance> llt_od_T_S_I_covariance(
      T_S_I_covariance_);
  CHECK_NE(llt_od_T_S_I_covariance.info(), Eigen::NumericalIssue)
      << "The given covariance matrix " << std::endl
      << T_S_I_covariance << std::endl
      << " is not positive definite.";
  CHECK(id_.isValid());
}

bool OptionalSensorExtrinsics::operator==(
    const OptionalSensorExtrinsics& other) const {
  bool result = id_ == other.id_;
  result = result && sensor_type_ == other.sensor_type_;
  result = result && T_S_I_ == other.T_S_I_;
  result = result && T_S_I_covariance_ == other.T_S_I_covariance_;
  return result;
}

bool OptionalSensorExtrinsics::operator!=(const OptionalSensorExtrinsics& other)
    const {
  return !operator==(other);
}

void OptionalSensorExtrinsics::serialize(
    vi_map_deprecated::proto::OptionalSensorExtrinsics* proto) const {
  CHECK_NOTNULL(proto);
  id_.serialize(proto->mutable_id());
  common::eigen_proto::serialize(get_T_S_I(), proto->mutable_t_s_i());
  common::eigen_proto::serialize(
      T_S_I_covariance_, proto->mutable_t_s_i_covariance());
  proto->set_sensor_type(static_cast<uint32_t>(sensor_type_));
}

void OptionalSensorExtrinsics::deserialize(
    const vi_map_deprecated::proto::OptionalSensorExtrinsics& proto) {
  id_.deserialize(proto.id());
  CHECK(id_.isValid());
  common::eigen_proto::deserialize(proto.t_s_i(), &T_S_I_);
  common::eigen_proto::deserialize(
      proto.t_s_i_covariance(), &T_S_I_covariance_);
  sensor_type_ = static_cast<OptionalSensorType>(proto.sensor_type());
}

OptionalSensorExtrinsics::UniquePtr OptionalSensorExtrinsics::loadFromYaml(
    const std::string& yaml_filename) {
  OptionalSensorExtrinsics::UniquePtr optional_sensor_extrinsics;
  CHECK(!optional_sensor_extrinsics);
  try {
    YAML::Node optional_sensor_extrinsics_node =
        YAML::LoadFile(yaml_filename.c_str());

    SensorId id;
    std::string id_as_string;
    if (!YAML::safeGet(optional_sensor_extrinsics_node, "id", &id_as_string)) {
      LOG(WARNING) << "Unable to find an ID field. Generating a new random id.";
      common::generateId(&id);
    } else {
      CHECK(!id_as_string.empty());
      CHECK(id.fromHexString(id_as_string));
      CHECK(id.isValid());
    }

    std::string sensor_type_as_string;
    if (!YAML::safeGet(
            optional_sensor_extrinsics_node, "sensor_type",
            &sensor_type_as_string)) {
      LOG(FATAL) << "Unable to retrieve the sensor type from YAML file: "
                 << yaml_filename;
    }

    OptionalSensorType sensor_type =
        convertOptionalSensorTypeAsStringToOptionalSensorType(
            sensor_type_as_string);

    Eigen::Matrix4d T_S_I_matrix;
    if (!YAML::safeGet(
            optional_sensor_extrinsics_node, "T_S_I", &T_S_I_matrix)) {
      LOG(FATAL)
          << "Unable to get extrinsic transformation T_S_I from YAML file: "
          << yaml_filename;
    }
    const aslam::Transformation T_S_I(T_S_I_matrix);

    aslam::TransformationCovariance T_S_I_covariance;
    if (!YAML::safeGet(
            optional_sensor_extrinsics_node, "T_S_I_covariance",
            &T_S_I_covariance)) {
      LOG(FATAL) << "Unable to get extrinsic transformation covariance "
                    "T_S_I_covariance from YAML "
                 << "file: " << yaml_filename;
    }

    optional_sensor_extrinsics = aligned_unique<OptionalSensorExtrinsics>(
        id, sensor_type, T_S_I, T_S_I_covariance);
  } catch (const std::exception& ex) {  // NOLINT
    LOG(ERROR) << "Failed to load sensor extrinsics from file " << yaml_filename
               << " with the error: " << ex.what();
  }
  return optional_sensor_extrinsics;
}

void OptionalSensorExtrinsics::saveToYaml(
    const std::string& yaml_filename) const {
  CHECK(!yaml_filename.empty());
  if (common::fileExists(yaml_filename)) {
    LOG(ERROR) << "The given yaml file " << yaml_filename
               << " already exists. Please specify a filename that does not "
                  "already exist.";
    return;
  }
  YAML::Node optional_sensor_extrinsics_node;

  optional_sensor_extrinsics_node["sensor_type"] =
      convertOptionalSensorTypeToString(sensor_type_);
  optional_sensor_extrinsics_node["T_S_I"] = T_S_I_.getTransformationMatrix();
  optional_sensor_extrinsics_node["T_S_I_covariance"] = T_S_I_covariance_;
  optional_sensor_extrinsics_node["id"] = id_.hexString();

  YAML::Save(optional_sensor_extrinsics_node, yaml_filename);
}

std::string convertOptionalSensorTypeToString(OptionalSensorType sensor_type) {
  switch (sensor_type) {
    case OptionalSensorType::kWheelOdometry:
      return kSensorTypeWheelOdometry;
      break;
    case OptionalSensorType::kGPSUTM:
      return kSensorTypeGPSUTM;
      break;
    case OptionalSensorType::kGPSWGS:
      return kSensorTypeGPSWGS;
      break;
    default:
      LOG(FATAL) << "Unknown sensor type: " << static_cast<int>(sensor_type);
      break;
  }
  return "";
}

OptionalSensorType convertOptionalSensorTypeAsStringToOptionalSensorType(
    const std::string& optional_sensor_type_as_string) {
  CHECK(!optional_sensor_type_as_string.empty());
  if (optional_sensor_type_as_string == kSensorTypeWheelOdometry) {
    return OptionalSensorType::kWheelOdometry;
  } else if (optional_sensor_type_as_string == kSensorTypeGPSUTM) {
    return OptionalSensorType::kGPSUTM;
  } else if (optional_sensor_type_as_string == kSensorTypeGPSWGS) {
    return OptionalSensorType::kGPSWGS;
  } else {
    LOG(FATAL) << "Unknown optional sensor type: "
               << optional_sensor_type_as_string;
    return static_cast<OptionalSensorType>(0);
  }
}

}  // namespace vi_map
