#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <sensors/external-features.h>

namespace vi_map {

constexpr char kYamlFieldNameTargetSensor[] = "target_sensor";
constexpr char kYamlFieldNameHasUncertainties[] = "has_uncertainties";
constexpr char kYamlFieldNameHasOrientations[] = "has_orientations";
constexpr char kYamlFieldNameHasScores[] = "has_scores";
constexpr char kYamlFieldNameHasScales[] = "has_scales";
constexpr char kYamlFieldNameHasTrackIds[] = "has_track_ids";
constexpr char kYamlFieldNameFeatureType[] = "feature_type";

constexpr const char* kBinaryIdentifier = "Binary";
constexpr const char* kSIFTIdentifier = "SIFT";

std::string FeatureTypeToString(FeatureType feature_type) {
  if (feature_type == FeatureType::kBinary) {
    return std::string(kBinaryIdentifier);
  } else if (feature_type == FeatureType::kSIFT) {
    return std::string(kSIFTIdentifier);
  } else {
    LOG(FATAL) << "Unknown feature type!";
  }
}

FeatureType StringToFeatureType(const std::string& feature_string) {
  const char* feature_c_string = feature_string.c_str();
  std::function<bool(const char*, const char*)> equals = [](const char* lhs,
                                                            const char* rhs) {
    return std::strcmp(lhs, rhs) == 0;
  };

  if (equals(feature_c_string, kBinaryIdentifier)) {
    return FeatureType::kBinary;
  } else if (equals(feature_c_string, kSIFTIdentifier)) {
    return FeatureType::kSIFT;
  } else {
    LOG(FATAL) << "Unknown feature type!";
  }
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

  CHECK(YAML::safeGet(
      sensor_node, static_cast<std::string>(kYamlFieldNameHasUncertainties),
      &has_uncertainties_));
  CHECK(YAML::safeGet(
      sensor_node, static_cast<std::string>(kYamlFieldNameHasOrientations),
      &has_orientations_));
  CHECK(YAML::safeGet(
      sensor_node, static_cast<std::string>(kYamlFieldNameHasScores),
      &has_scores_));
  CHECK(YAML::safeGet(
      sensor_node, static_cast<std::string>(kYamlFieldNameHasScales),
      &has_scales_));
  CHECK(YAML::safeGet(
      sensor_node, static_cast<std::string>(kYamlFieldNameHasTrackIds),
      &has_track_ids_));

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

  node[static_cast<std::string>(kYamlFieldNameHasUncertainties)] =
      has_uncertainties_;
  node[static_cast<std::string>(kYamlFieldNameHasOrientations)] =
      has_orientations_;
  node[static_cast<std::string>(kYamlFieldNameHasScores)] = has_scores_;
  node[static_cast<std::string>(kYamlFieldNameHasScales)] = has_scales_;
  node[static_cast<std::string>(kYamlFieldNameHasTrackIds)] = has_track_ids_;
  node[static_cast<std::string>(kYamlFieldNameFeatureType)] =
      FeatureTypeToString(feature_type_);
}

}  // namespace vi_map
