#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>

#include <sensors/external-features.h>

namespace vi_map {

constexpr char kYamlFieldNameTargetSensor[] = "target_sensor";

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

  // TODO(smauq): fetch these actually
  has_uncertainties_ = true;
  has_orientations_ = false;
  has_scores_ = false;
  has_scales_ = true;
  has_track_ids = true;

  feature_type_ = ExternalFeatureType::kVisualBinaryFeatures;

  return true;
}

void ExternalFeatures::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  // TODO(smauq): write serializer even though we will probably never use it
}

}  // namespace vi_map
