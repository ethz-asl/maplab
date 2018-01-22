#include "sensors/sensor-extrinsics.h"

#include <random>
#include <glog/logging.h>

namespace vi_map {

Extrinsics::Extrinsics() : extrinsics_type_(ExtrinsicsType::kInvalid) {}

Extrinsics::UniquePtr Extrinsics::createFromYaml(const YAML::Node& yaml_node) {
  Extrinsics::UniquePtr extrinsics(new Extrinsics());
  if (!extrinsics->deserialize(yaml_node)) {
    LOG(ERROR) << "YAML deserialization failed.";
    extrinsics.release();
  }
  return extrinsics;
}

bool Extrinsics::deserialize(const YAML::Node& yaml_node) {
  std::string type_as_string;
  if (YAML::safeGet(
          yaml_node, static_cast<std::string>(kYamlFieldNameExtrinsicsType),
          &type_as_string)) {
    CHECK(!type_as_string.empty());
    extrinsics_type_ = stringToExtrinsicsType(type_as_string);
  } else {
    LOG(ERROR) << "Unable to find the extrinsics type.";
    return false;
  }
  CHECK(extrinsics_type_ != ExtrinsicsType::kInvalid);

  Eigen::Matrix4d T_R_S_matrix;
  if (!YAML::safeGet(
          yaml_node, static_cast<std::string>(kYamlFieldName_T_R_S),
          &T_R_S_matrix)) {
    LOG(ERROR) << "Unable to find the T_R_S node.";
    return false;
  }
  T_R_S_ = aslam::Transformation(T_R_S_matrix);
  return true;
}

Extrinsics::UniquePtr Extrinsics::createRandomExtrinsics() {
  constexpr int kNumExtrinsicsTypes =
      static_cast<int>(ExtrinsicsType::kInvalid);
  CHECK_GT(kNumExtrinsicsTypes, 0);

  std::default_random_engine random_engine;
  std::uniform_int_distribution<int> extrinsics_type_distribution(
      0, kNumExtrinsicsTypes);
  const int extrinsics_type_int = extrinsics_type_distribution(random_engine);
  CHECK_GE(extrinsics_type_int, 0);
  CHECK_LT(extrinsics_type_int, kNumExtrinsicsTypes);

  const ExtrinsicsType extrinsics_type =
      static_cast<ExtrinsicsType>(extrinsics_type_int);

  aslam::Transformation T_R_S;
  T_R_S.setRandom();
  if (extrinsics_type == ExtrinsicsType::kPositionOnly) {
    T_R_S.getRotation().setIdentity();
  }
  Extrinsics::UniquePtr extrinsics =
      aligned_unique<Extrinsics>(extrinsics_type, T_R_S);
  return extrinsics;
}

}  // namespace vi_map
