#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include <sensors/relative-6dof-pose.h>

constexpr char kYamlFieldNamePoseMeasurementCovariance[] =
    "pose_measurement_covariance";

namespace vi_map {

template <>
SensorType sensorToType<Relative6DoFPose>() {
  return SensorType::kRelative6DoFPose;
}

constexpr char kDefaultRelative6DoFHardwareId[] = "relative_6dof_pose";

Relative6DoFPose::Relative6DoFPose()
    : Sensor(
          SensorType::kRelative6DoFPose,
          static_cast<std::string>(kDefaultRelative6DoFHardwareId)) {
  pose_measurement_covariance_.setIdentity();
}

Relative6DoFPose::Relative6DoFPose(
    const SensorId& sensor_id, const std::string& hardware_id,
    const aslam::TransformationCovariance& pose_measurement_covariance)
    : Sensor(sensor_id, SensorType::kRelative6DoFPose, hardware_id),
      pose_measurement_covariance_(pose_measurement_covariance) {
  CHECK(!pose_measurement_covariance_.isZero(1e-12));
}

bool Relative6DoFPose::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (!YAML::safeGet(
          sensor_node,
          static_cast<std::string>(kYamlFieldNamePoseMeasurementCovariance),
          &pose_measurement_covariance_)) {
    LOG(ERROR) << "Unable to retrieve the general measurement covariance "
               << "from the given YAML file.";
    return false;
  }
  return true;
}
void Relative6DoFPose::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  YAML::Node& node = *CHECK_NOTNULL(sensor_node);
  node[static_cast<std::string>(kYamlFieldNamePoseMeasurementCovariance)] =
      pose_measurement_covariance_;
}

}  // namespace vi_map
