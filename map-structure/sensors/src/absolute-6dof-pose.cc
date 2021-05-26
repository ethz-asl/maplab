#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include <sensors/absolute-6dof-pose.h>

namespace vi_map {

constexpr char kYamlFieldNameT_G_S_fixed_covariance[] =
    "T_G_S_fixed_covariance";

Absolute6DoF::Absolute6DoF() : Sensor(), has_fixed_T_G_S_covariance_(false) {}

Absolute6DoF::Absolute6DoF(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic), has_fixed_T_G_S_covariance_(false) {}

bool Absolute6DoF::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (sensor_node[kYamlFieldNameT_G_S_fixed_covariance]) {
    aslam::TransformationCovariance input_matrix;
    CHECK(YAML::safeGet(
        sensor_node,
        static_cast<std::string>(kYamlFieldNameT_G_S_fixed_covariance),
        &input_matrix));
    T_G_S_fixed_covariance_ = input_matrix;
    has_fixed_T_G_S_covariance_ = true;
  } else {
    has_fixed_T_G_S_covariance_ = false;
  }
  return true;
}

void Absolute6DoF::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  CHECK_NOTNULL(sensor_node);
  if (has_fixed_T_G_S_covariance_) {
    (*sensor_node)[static_cast<std::string>(
        kYamlFieldNameT_G_S_fixed_covariance)] =
        YAML::convert<aslam::TransformationCovariance>::encode(
            T_G_S_fixed_covariance_);
  }
}

}  // namespace vi_map
