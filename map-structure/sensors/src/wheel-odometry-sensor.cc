#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include <sensors/wheel-odometry-sensor.h>

namespace vi_map {

constexpr char kYamlFieldNameT_St_Stp1_fixed_covariance[] =
    "T_St_Stp1_fixed_covariance";

WheelOdometry::WheelOdometry()
    : Sensor(), has_T_St_Stp1_fixed_covariance_(false) {}

WheelOdometry::WheelOdometry(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic), has_T_St_Stp1_fixed_covariance_(false) {}

bool WheelOdometry::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (sensor_node[kYamlFieldNameT_St_Stp1_fixed_covariance]) {
    aslam::TransformationCovariance input_matrix;
    CHECK(YAML::safeGet(
        sensor_node,
        static_cast<std::string>(kYamlFieldNameT_St_Stp1_fixed_covariance),
        &input_matrix));
    T_St_Stp1_fixed_covariance_ = input_matrix;
    has_T_St_Stp1_fixed_covariance_ = true;
  } else {
    has_T_St_Stp1_fixed_covariance_ = false;
  }
  return true;
}

void WheelOdometry::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  if (has_T_St_Stp1_fixed_covariance_) {
    (*sensor_node)[static_cast<std::string>(
        kYamlFieldNameT_St_Stp1_fixed_covariance)] =
        YAML::convert<aslam::TransformationCovariance>::encode(
            T_St_Stp1_fixed_covariance_);
  }
}

}  // namespace vi_map
