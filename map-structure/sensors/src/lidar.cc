#include "sensors/lidar.h"

#include <Eigen/Core>
#include <aslam/common/yaml-serialization.h>
#include <maplab-common/eigen-proto.h>

namespace vi_map {

constexpr char kYamlFieldNameHasPointTimestamps[] = "has_point_timestamps";
constexpr char kYamlFieldNameHasRelativePointTimestamps[] =
    "has_relative_point_timestamps";
constexpr char kYamlFieldNameTimestampUnit[] = "point_timestamp_unit";
constexpr char kYamlFieldNameMinRange[] = "min_range_m";
constexpr char kYamlFieldNameMaxRange[] = "max_range_m";
constexpr char kYamlFieldNameRobotFilter[] = "robot_filter";

constexpr char kTimeStampUnitNanoSeconds[] = "NANO";
constexpr char kTimeStampUnitMicroSeconds[] = "MICRO";
constexpr char kTimeStampUnitMilliSeconds[] = "MILLI";
constexpr char kTimeStampUnitSeconds[] = "SECONDS";

constexpr double kDefaultMinRange = 0.1;
constexpr double kDefaultMaxRange = 100.0;

void Lidar::init_default() {
  has_point_timestamps_ = false;
  has_relative_point_timestamps_ = true;
  timestamp_unit_ = TimestampUnit::kNanoSeconds;
  min_range_m_ = kDefaultMinRange;
  max_range_m_ = kDefaultMaxRange;
  has_robot_filter_ = false;
}

Lidar::Lidar() : Sensor() {
  init_default();
}

Lidar::Lidar(const aslam::SensorId& sensor_id) : Sensor(sensor_id) {
  init_default();
}

Lidar::Lidar(const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {
  init_default();
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

  // Get information about timestamps.
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

  // LiDAR minimum and maximum range.
  if (sensor_node[static_cast<std::string>(kYamlFieldNameMinRange)]) {
    if (!YAML::safeGet(
            sensor_node, static_cast<std::string>(kYamlFieldNameMinRange),
            &min_range_m_)) {
      return false;
    }
  } else {
    LOG(WARNING) << "Min range not specified for LiDAR " << id_ << ". "
                 << "Defaulting to " << kDefaultMinRange << " m.";
  }

  if (sensor_node[static_cast<std::string>(kYamlFieldNameMaxRange)]) {
    if (!YAML::safeGet(
            sensor_node, static_cast<std::string>(kYamlFieldNameMaxRange),
            &max_range_m_)) {
      return false;
    }
  } else {
    LOG(WARNING) << "Max range not specified for LiDAR " << id_ << ". "
                 << "Defaulting to " << kDefaultMaxRange << " m.";
  }

  // Parse potential box filter around robot.
  const YAML::Node& filter_node =
      sensor_node[static_cast<std::string>(kYamlFieldNameRobotFilter)];
  if (filter_node) {
    has_robot_filter_ = true;

    if (!filter_node.IsMap()) {
      LOG(ERROR) << "Robot filter for LiDAR " << id_ << " is not a map.";
      return false;
    }

    if (!YAML::safeGet(filter_node, "x_min", &robot_filter_.x_min)) {
      LOG(ERROR) << "Robot filter for LiDAR " << id_
                 << "must have x_min defined.";
      return false;
    }

    if (!YAML::safeGet(filter_node, "x_max", &robot_filter_.x_max)) {
      LOG(ERROR) << "Robot filter for LiDAR " << id_
                 << "must have x_max defined.";
      return false;
    }

    if (!YAML::safeGet(filter_node, "y_min", &robot_filter_.y_min)) {
      LOG(ERROR) << "Robot filter for LiDAR " << id_
                 << "must have y_min defined.";
      return false;
    }

    if (!YAML::safeGet(filter_node, "y_max", &robot_filter_.y_max)) {
      LOG(ERROR) << "Robot filter for LiDAR " << id_
                 << "must have y_max defined.";
      return false;
    }

    if (!YAML::safeGet(filter_node, "z_min", &robot_filter_.z_min)) {
      LOG(ERROR) << "Robot filter for LiDAR " << id_
                 << "must have z_min defined.";
      return false;
    }

    if (!YAML::safeGet(filter_node, "z_max", &robot_filter_.z_max)) {
      LOG(ERROR) << "Robot filter for LiDAR " << id_
                 << "must have z_max defined.";
      return false;
    }

    CHECK_LT(robot_filter_.x_min, robot_filter_.x_max);
    CHECK_LT(robot_filter_.y_min, robot_filter_.y_max);
    CHECK_LT(robot_filter_.z_min, robot_filter_.z_max);
  } else {
    has_robot_filter_ = false;
  }

  return true;
}

void Lidar::saveToYamlNodeImpl(YAML::Node* sensor_node) const {
  CHECK_NOTNULL(sensor_node);
  YAML::Node& node = *sensor_node;

  // Save information on timestamps.
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

  // Save LiDAR min and max range.
  node[static_cast<std::string>(kYamlFieldNameMinRange)] = min_range_m_;
  node[static_cast<std::string>(kYamlFieldNameMaxRange)] = max_range_m_;

  // Save the robot filter if one is present.
  if (has_robot_filter_) {
    YAML::Node filter_node;
    filter_node["x_min"] = robot_filter_.x_min;
    filter_node["x_max"] = robot_filter_.x_max;
    filter_node["y_min"] = robot_filter_.y_min;
    filter_node["y_max"] = robot_filter_.y_max;
    filter_node["z_min"] = robot_filter_.z_min;
    filter_node["z_max"] = robot_filter_.z_max;
    node[static_cast<std::string>(kYamlFieldNameRobotFilter)] = filter_node;
  }
}

template <>
void LidarMeasurement<resources::PointCloud>::setRandomImpl() {
  static constexpr double kMinRangeMeters = 0.0;
  static constexpr double kMaxRangeMeters = 1e3;
  static constexpr double kMinIntensity = 0.0;
  static constexpr double kMaxIntensity = 255.0;

  std::random_device random_device;
  std::default_random_engine random_engine(random_device());
  std::uniform_real_distribution<float> range_distribution(
      kMinRangeMeters, kMaxRangeMeters);

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
    const float range_value = range_distribution(random_engine);
    point_cloud_.xyz[idx] = range_value;

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
