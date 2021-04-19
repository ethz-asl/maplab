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

  feature_type_ = ExternalFeatureType::kVisualBinaryFeatures;

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
}

}  // namespace vi_map
