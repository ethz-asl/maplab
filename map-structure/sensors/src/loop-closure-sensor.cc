#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include <sensors/loop-closure-sensor.h>

namespace vi_map {

constexpr char kYamlFieldNameT_A_B_fixed_covariance[] =
    "T_A_B_fixed_covariance";

LoopClosureSensor::LoopClosureSensor()
    : Sensor(), has_fixed_T_A_B_covariance_(false) {}

LoopClosureSensor::LoopClosureSensor(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic), has_fixed_T_A_B_covariance_(false) {}

bool LoopClosureSensor::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (sensor_node[kYamlFieldNameT_A_B_fixed_covariance]) {
    aslam::TransformationCovariance input_matrix;
    CHECK(YAML::safeGet(
        sensor_node,
        static_cast<std::string>(kYamlFieldNameT_A_B_fixed_covariance),
        &input_matrix));
    T_A_B_fixed_covariance_ = input_matrix;
    has_fixed_T_A_B_covariance_ = true;
  } else {
    has_fixed_T_A_B_covariance_ = false;
  }
  return true;
}

void LoopClosureSensor::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  if (has_fixed_T_A_B_covariance_) {
    (*sensor_node)[static_cast<std::string>(
        kYamlFieldNameT_A_B_fixed_covariance)] =
        YAML::convert<aslam::TransformationCovariance>::encode(
            T_A_B_fixed_covariance_);
  }
}

}  // namespace vi_map
