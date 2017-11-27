#include "vi-map/imu-sigmas.h"

#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <yaml-cpp/yaml.h>

#include "vi-map/vi_map.pb.h"

namespace vi_map {
namespace serialization {

void serializeImuSigmas(const ImuSigmas& imu_sigmas, proto::ImuSigmas* proto) {
  CHECK_NOTNULL(proto);
  proto->set_gyro_noise(imu_sigmas.gyro_noise_density);
  proto->set_gyro_bias(imu_sigmas.gyro_bias_random_walk_noise_density);
  proto->set_acc_noise(imu_sigmas.acc_noise_density);
  proto->set_acc_bias(imu_sigmas.acc_bias_random_walk_noise_density);
}

void deserializeImuSigmas(
    const proto::ImuSigmas& proto, ImuSigmas* imu_sigmas) {
  CHECK(proto.has_gyro_noise());
  CHECK(proto.has_gyro_bias());
  CHECK(proto.has_acc_noise());
  CHECK(proto.has_acc_bias());
  imu_sigmas->gyro_noise_density = proto.gyro_noise();
  imu_sigmas->gyro_bias_random_walk_noise_density = proto.gyro_bias();
  imu_sigmas->acc_noise_density = proto.acc_noise();
  imu_sigmas->acc_bias_random_walk_noise_density = proto.acc_bias();
}
}  // namespace serialization

bool ImuSigmas::loadFromYaml(const std::string& yaml_path, ImuSigmas* config) {
  return YAML::Load(yaml_path, config);
}

bool ImuSigmas::saveToYaml(const std::string& yaml_path) const {
  return YAML::Save(*this, yaml_path);
}
}  // namespace vi_map

namespace YAML {
Node convert<vi_map::ImuSigmas>::encode(const vi_map::ImuSigmas& sigmas) {
  Node sigmas_node;
  sigmas_node["gyro_noise_density"] = sigmas.gyro_noise_density;
  sigmas_node["gyro_bias_random_walk_noise_density"] =
      sigmas.gyro_bias_random_walk_noise_density;
  sigmas_node["acc_noise_density"] = sigmas.acc_noise_density;
  sigmas_node["acc_bias_random_walk_noise_density"] =
      sigmas.acc_bias_random_walk_noise_density;
  return sigmas_node;
}

bool convert<vi_map::ImuSigmas>::decode(
    const Node& node, vi_map::ImuSigmas& config) {  // NOLINT
  bool success = true;
  success &=
      YAML::safeGet(node, "gyro_noise_density", &config.gyro_noise_density);
  success &= YAML::safeGet(
      node, "gyro_bias_random_walk_noise_density",
      &config.gyro_bias_random_walk_noise_density);
  success &=
      YAML::safeGet(node, "acc_noise_density", &config.acc_noise_density);
  success &= YAML::safeGet(
      node, "acc_bias_random_walk_noise_density",
      &config.acc_bias_random_walk_noise_density);
  return success;
}
}  // namespace YAML
