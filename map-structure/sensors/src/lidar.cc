#include "sensors/lidar.h"

#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {

constexpr char kYamlFieldNameHasPointTimestamps[] = "has_point_timestamps";
constexpr char kYamlFieldNameHasRelativePointTimestamps[] =
    "has_relative_point_timestamps";
constexpr char kYamlFieldNameTimestampUnit[] = "point_timestamp_unit";

constexpr char kTimeStampUnitNanoSeconds[] = "NANO";
constexpr char kTimeStampUnitMicroSeconds[] = "MICRO";
constexpr char kTimeStampUnitMilliSeconds[] = "MILLI";
constexpr char kTimeStampUnitSeconds[] = "SECONDS";

Lidar::Lidar() : Sensor() {
  has_point_timestamps_ = false;
  has_relative_point_timestamps_ = true;
  timestamp_unit_ = TimestampUnit::kNanoSeconds;
}

Lidar::Lidar(const aslam::SensorId& sensor_id) : Sensor(sensor_id) {
  has_point_timestamps_ = false;
  has_relative_point_timestamps_ = true;
  timestamp_unit_ = TimestampUnit::kNanoSeconds;
}

Lidar::Lidar(const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {
  has_point_timestamps_ = false;
  has_relative_point_timestamps_ = true;
  timestamp_unit_ = TimestampUnit::kNanoSeconds;
}

int32_t Lidar::getTimestampConversionToNanoseconds() const {
  switch (timestamp_unit_) {
    case TimestampUnit::kNanoSeconds:
      return 1;
    case TimestampUnit::kMicroSeconds:
      return 1000;
    case TimestampUnit::kMilliSeconds:
      return 1000 * 1000;
    case TimestampUnit::kSeconds:
      return 1000 * 1000 * 1000;
    default:
      LOG(FATAL) << "LiDAR point timestamp unit defined, but no case added to "
                 << "this function.";
  }
}

bool Lidar::loadFromYamlNodeImpl(const YAML::Node& sensor_node) {
  std::function<bool(const char*, const char*)> equals = [](const char* lhs,
                                                            const char* rhs) {
    return std::strcmp(lhs, rhs) == 0;
  };

  if (!YAML::safeGet(
          sensor_node,
          static_cast<std::string>(kYamlFieldNameHasPointTimestamps),
          &has_point_timestamps_)) {
    LOG(ERROR) << "The sensor config must specify if the LiDAR " << id_
               << " has individual timestamps for points.";
    return false;
  }

  if (has_point_timestamps_) {
    std::string timestamp_unit_string;
    if (!YAML::safeGet(
            sensor_node, static_cast<std::string>(kYamlFieldNameTimestampUnit),
            &timestamp_unit_string)) {
      LOG(ERROR) << "LiDAR has point timestamps, but the time unit is not "
                 << "specified for sensor " << id_ << ".";
      return false;
    }

    const char* timestamp_unit_c_string = timestamp_unit_string.c_str();
    if (equals(timestamp_unit_c_string, kTimeStampUnitNanoSeconds)) {
      timestamp_unit_ = TimestampUnit::kNanoSeconds;
    } else if (equals(timestamp_unit_c_string, kTimeStampUnitMicroSeconds)) {
      timestamp_unit_ = TimestampUnit::kMicroSeconds;
    } else if (equals(timestamp_unit_c_string, kTimeStampUnitMilliSeconds)) {
      timestamp_unit_ = TimestampUnit::kMilliSeconds;
    } else if (equals(timestamp_unit_c_string, kTimeStampUnitSeconds)) {
      timestamp_unit_ = TimestampUnit::kSeconds;
    } else {
      LOG(ERROR) << "Undefined LiDAR timestamp unit \"" << timestamp_unit_string
                 << "\" for sensor " << id_ << ".";
      return false;
    }

    if (!YAML::safeGet(
            sensor_node,
            static_cast<std::string>(kYamlFieldNameHasRelativePointTimestamps),
            &has_relative_point_timestamps_)) {
      LOG(ERROR)
          << "Config does not specify if LiDAR point timestamps are relative "
          << "or absolute with respect to the message timestamp.";
      return false;
    }
  }

  return true;
}

void Lidar::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  CHECK_NOTNULL(sensor_node);
  YAML::Node& node = *sensor_node;

  node[static_cast<std::string>(kYamlFieldNameHasPointTimestamps)] =
      has_point_timestamps_;
  if (has_point_timestamps_) {
    node[static_cast<std::string>(kYamlFieldNameHasRelativePointTimestamps)] =
        has_relative_point_timestamps_;

    if (timestamp_unit_ == TimestampUnit::kNanoSeconds) {
      node[static_cast<std::string>(kYamlFieldNameTimestampUnit)] =
          kTimeStampUnitNanoSeconds;
    } else if (timestamp_unit_ == TimestampUnit::kMicroSeconds) {
      node[static_cast<std::string>(kYamlFieldNameTimestampUnit)] =
          kTimeStampUnitMicroSeconds;
    } else if (timestamp_unit_ == TimestampUnit::kMilliSeconds) {
      node[static_cast<std::string>(kYamlFieldNameTimestampUnit)] =
          kTimeStampUnitMilliSeconds;
    } else if (timestamp_unit_ == TimestampUnit::kSeconds) {
      node[static_cast<std::string>(kYamlFieldNameTimestampUnit)] =
          kTimeStampUnitSeconds;
    }
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

}  // namespace vi_map
