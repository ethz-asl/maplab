#include "sensors/lidar.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {

template <>
SensorType sensorToType<Lidar>() {
  return SensorType::kLidar;
}

template <>
SensorType measurementToSensorType<LidarMeasurement>() {
  return SensorType::kLidar;
}

constexpr char kDefaultLidarHardwareId[] = "/velodyne0";

Lidar::Lidar()
    : Sensor(
          SensorType::kLidar,
          static_cast<std::string>(kDefaultLidarHardwareId)) {}

Lidar::Lidar(const SensorId& sensor_id, const std::string& hardware_id)
    : Sensor(sensor_id, SensorType::kLidar, hardware_id) {}

bool Lidar::loadFromYamlNodeImpl(const YAML::Node& /*sensor_node*/) {
  return true;
}

void Lidar::saveToYamlNodeImpl(YAML::Node* /*sensor_node*/) const {}

void LidarMeasurement::setRandomImpl() {
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

bool LidarMeasurement::isValidImpl() const {
  if (!point_cloud_.colors.empty()) {
    return false;
  }
  if (!point_cloud_.normals.empty()) {
    return false;
  }
  const size_t num_points = point_cloud_.xyz.size();
  if (point_cloud_.scalars.size() != num_points) {
    return false;
  }

  for (const float distance : point_cloud_.xyz) {
    if (distance < kMinDistanceMeters || distance > kMaxDistanceMeters) {
      return false;
    }
  }
  for (const float intensity : point_cloud_.scalars) {
    if (intensity < kMinIntensity || intensity > kMaxIntensity) {
      return false;
    }
  }
  return true;
}

}  // namespace vi_map
