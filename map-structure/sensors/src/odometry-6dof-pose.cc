#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include <sensors/odometry-6dof-pose.h>

namespace vi_map {

Odometry6DoF::Odometry6DoF() : Sensor(), has_fixed_covariance_(false) {}

Odometry6DoF::Odometry6DoF(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic), has_fixed_covariance_(false) {}

bool Odometry6DoF::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (sensor_node[kYamlFieldNameFixedCovariance]) {
    aslam::TransformationCovariance input_matrix;
    CHECK(YAML::safeGet(
        sensor_node, static_cast<std::string>(kYamlFieldNameFixedCovariance),
        &input_matrix));
    T_St_Stp1_fixed_covariance_ = input_matrix;
    has_fixed_covariance_ = true;
  } else {
    has_fixed_covariance_ = false;
  }
  return true;
}

void Odometry6DoF::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  if (has_fixed_covariance_) {
    (*sensor_node)[static_cast<std::string>(kYamlFieldNameFixedCovariance)] =
        YAML::convert<aslam::TransformationCovariance>::encode(
            T_St_Stp1_fixed_covariance_);
  }
}

}  // namespace vi_map
