#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <sensors/external-features.h>

namespace vi_map {

constexpr char kYamlFieldNameTargetSensor[] = "target_sensor";
constexpr char kYamlFieldNameFeatureType[] = "feature_type";

constexpr const char* kInvalidIdentifier = "Invalid";
constexpr const char* kBinaryIdentifier = "Binary";
constexpr const char* kSuperPointIdentifier = "SuperPoint";
constexpr const char* kR2D2Identifier = "R2D2";
constexpr const char* kSIFTIdentifier = "SIFT";
constexpr const char* kLIDARSuperPointIdentifier = "LIDARSuperPoint";

std::string FeatureTypeToString(FeatureType feature_type) {
  CHECK(feature_type != FeatureType::kInvalid);
  if (feature_type == FeatureType::kBinary) {
    return std::string(kBinaryIdentifier);
  } else if (feature_type == FeatureType::kSuperPoint) {
    return std::string(kSuperPointIdentifier);
  } else if (feature_type == FeatureType::kR2D2) {
    return std::string(kR2D2Identifier);
  } else if (feature_type == FeatureType::kSIFT) {
    return std::string(kSIFTIdentifier);
  } else if (feature_type == FeatureType::kLIDARSuperPoint) {
    return std::string(kLIDARSuperPointIdentifier);
  }
  LOG(FATAL) << "Unknown feature type!";
}

FeatureType StringToFeatureType(const std::string& feature_string) {
  const char* feature_c_string = feature_string.c_str();
  std::function<bool(const char*, const char*)> equals = [](const char* lhs,
                                                            const char* rhs) {
    return std::strcmp(lhs, rhs) == 0;
  };

  if (equals(feature_c_string, kInvalidIdentifier)) {
    return FeatureType::kInvalid;
  } else if (equals(feature_c_string, kBinaryIdentifier)) {
    return FeatureType::kBinary;
  } else if (equals(feature_c_string, kSuperPointIdentifier)) {
    return FeatureType::kSuperPoint;
  } else if (equals(feature_c_string, kR2D2Identifier)) {
    return FeatureType::kR2D2;
  } else if (equals(feature_c_string, kSIFTIdentifier)) {
    return FeatureType::kSIFT;
  } else if (equals(feature_c_string, kLIDARSuperPointIdentifier)) {
    return FeatureType::kLIDARSuperPoint;
  }
  LOG(FATAL) << "Unknown feature type!";
}

bool isFloatFeature(FeatureType feature_type) {
  CHECK(feature_type != FeatureType::kInvalid);
  if (feature_type == FeatureType::kBinary) {
    return false;
  }
  return true;
}

bool isLidarFeature(FeatureType feature_type) {
  CHECK(feature_type != FeatureType::kInvalid);
  if (feature_type == FeatureType::kLIDARSuperPoint) {
    return true;
  }
  return false;
}

ExternalFeatures::ExternalFeatures()
    : Sensor(), target_camera_index_set_(false) {}

ExternalFeatures::ExternalFeatures(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic), target_camera_index_set_(false) {}

bool ExternalFeatures::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  std::string id_as_string;
  CHECK(YAML::safeGet(
      sensor_node, static_cast<std::string>(kYamlFieldNameTargetSensor),
      &id_as_string));
  CHECK(target_sensor_id_.fromHexString(id_as_string));

  std::string feature_string;
  CHECK(YAML::safeGet(
      sensor_node, static_cast<std::string>(kYamlFieldNameFeatureType),
      &feature_string));
  feature_type_ = StringToFeatureType(feature_string);

  return true;
}

void ExternalFeatures::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  CHECK_NOTNULL(sensor_node);
  YAML::Node& node = *sensor_node;
  node[static_cast<std::string>(kYamlFieldNameTargetSensor)] =
      target_sensor_id_.hexString();
  node[static_cast<std::string>(kYamlFieldNameFeatureType)] =
      FeatureTypeToString(feature_type_);
}

}  // namespace vi_map
