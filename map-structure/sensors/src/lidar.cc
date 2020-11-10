#include "sensors/lidar.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {

constexpr char kDefaultLidarTopic[] = "/os1_node/points";

Lidar::Lidar(const aslam::SensorId& sensor_id)
    : Lidar(sensor_id, static_cast<std::string>(kDefaultLidarTopic)) {}

Lidar::Lidar(const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {}

bool Lidar::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  if (sensor_node[kYamlFieldNameNumberOfBeams]) {
    CHECK(YAML::safeGet(
        sensor_node, static_cast<std::string>(kYamlFieldNameNumberOfBeams),
        &n_beams_));
    has_number_of_beams_ = true;
  } else {
    has_number_of_beams_ = false;
  }

  if (sensor_node[kYamlFieldNameFoVUpperAngle]) {
    CHECK(YAML::safeGet(
        sensor_node, static_cast<std::string>(kYamlFieldNameFoVUpperAngle),
        &fov_upper_angle_deg_));
    has_upper_fov_angle_ = true;
  } else {
    has_upper_fov_angle_ = false;
  }
  if (sensor_node[kYamlFieldNameFoVLowerAngle]) {
    CHECK(YAML::safeGet(
        sensor_node, static_cast<std::string>(kYamlFieldNameFoVLowerAngle),
        &fov_lower_angle_deg_));
    has_lower_fov_angle_ = true;
  } else {
    has_lower_fov_angle_ = false;
  }

  return true;
}

void Lidar::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  CHECK_NOTNULL(sensor_node);
  if (has_number_of_beams_) {
    (*sensor_node)[static_cast<std::string>(kYamlFieldNameNumberOfBeams)] =
        YAML::convert<uint16_t>::encode(n_beams_);
  }
  if (has_upper_fov_angle_) {
    (*sensor_node)[static_cast<std::string>(kYamlFieldNameFoVUpperAngle)] =
        YAML::convert<uint16_t>::encode(fov_upper_angle_deg_);
  }
  if (has_lower_fov_angle_) {
    (*sensor_node)[static_cast<std::string>(kYamlFieldNameFoVLowerAngle)] =
        YAML::convert<uint16_t>::encode(fov_lower_angle_deg_);
  }
}

template <>
void LidarMeasurement<resources::PointCloud>::setRandomImpl() {
  static constexpr double kMinDistanceMeters = 0.0;
  static constexpr double kMaxDistanceMeters = 1e3;
  static constexpr double kMinIntensity = 0.0;
  static constexpr double kMaxIntensity = 255.0;

  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_real_distribution<float> distance_distribution(
      kMinDistanceMeters, kMaxDistanceMeters);

  constexpr double kMinNormal = -1.0;
  constexpr double kMaxNormal = 1.0;
  std::uniform_real_distribution<float> normal_distribution(
      kMinNormal, kMaxNormal);

  std::uniform_real_distribution<float> intensity_distribution(
      kMinIntensity, kMaxIntensity);

  constexpr uint8_t kMinColor = 0u;
  constexpr uint8_t kMaxColor = 255u;
  std::uniform_int_distribution<uint8_t> color_distribution(
      kMinColor, kMaxColor);

  constexpr size_t kMinNumPoints = 100u;
  constexpr size_t kMaxNumPoints = 10000u;
  std::uniform_int_distribution<size_t> num_points_distribution(
      kMinNumPoints, kMaxNumPoints);

  constexpr size_t kNumPoints = 5000u;
  point_cloud_.resize(kNumPoints);

  for (size_t idx = 0u; idx < 3u * kNumPoints; ++idx) {
    const float distance_value = distance_distribution(random_engine);
    point_cloud_.xyz[idx] = distance_value;

    const float normal_value = normal_distribution(random_engine);
    point_cloud_.normals[idx] = normal_value;

    const uint8_t color_value = color_distribution(random_engine);
    point_cloud_.colors[idx] = color_value;
  }

  for (size_t idx = 0u; idx < kNumPoints; ++idx) {
    const float intensity_value = intensity_distribution(random_engine);
    point_cloud_.scalars[idx] = intensity_value;
  }
}

template <>
bool LidarMeasurement<resources::PointCloud>::isValidImpl() const {
  return point_cloud_.checkConsistency();
}

template <>
void LidarMeasurement<pcl::PointCloud<pcl::PointXYZI>>::setRandomImpl() {
  // TODO(mfehr): implement or remove. We could use the
  // general point cloud conversion tools to make one
  // function to fill all point cloud types randomly.
  LOG(FATAL) << "NOT IMPLEMENTED!";
}

template <>
bool LidarMeasurement<pcl::PointCloud<pcl::PointXYZI>>::isValidImpl() const {
  return true;
}

template <>
void LidarMeasurement<sensor_msgs::PointCloud2>::setRandomImpl() {
  // TODO(mfehr): implement or remove. We could use the
  // general point cloud conversion tools to make one
  // function to fill all point cloud types randomly.
  LOG(FATAL) << "NOT IMPLEMENTED!";
}

template <>
bool LidarMeasurement<sensor_msgs::PointCloud2>::isValidImpl() const {
  return true;
}

template <>
void LidarMeasurement<pcl::PointCloud<pcl::OusterPointType>>::setRandomImpl() {
  // TODO(mfehr): implement or remove. We could use the
  // general point cloud conversion tools to make one
  // function to fill all point cloud types randomly.
  LOG(FATAL) << "NOT IMPLEMENTED!";
}

template <>
bool LidarMeasurement<pcl::PointCloud<pcl::OusterPointType>>::isValidImpl()
    const {
  return true;
}

}  // namespace vi_map
