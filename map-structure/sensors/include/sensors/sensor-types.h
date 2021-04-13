#ifndef SENSORS_SENSOR_TYPES_H_
#define SENSORS_SENSOR_TYPES_H_

#include <string>

#include <aslam/common/sensor.h>

namespace vi_map {

// Warning: Ordering of sensor is important in that the first sensors in the
// enum have to be the aslam sensors and only afterwards can the other maplab
// sensors be added (due to how the integer assignment works)
enum SensorType : uint8_t {
  kUnknown = aslam::SensorType::kUnknown,
  kNCamera = aslam::SensorType::kNCamera,
  kCamera = aslam::SensorType::kCamera,
  kImu,
  kLoopClosureSensor,
  kGpsWgs,
  kGpsUtm,
  kLidar,
  kOdometry6DoF,
  kAbsolute6DoF,
  kWheelOdometry,
  kPointCloudMapSensor,
  kInvalid  // this must be the last one
};

constexpr const char* kNCameraIdentifier = aslam::kNCameraIdentifier;
constexpr const char* kCameraIdentifier = aslam::kCameraIdentifier;
constexpr const char* kImuIdentifier = "IMU";
constexpr const char* kGpsUtmIdentifier = "GPS_UTM";
constexpr const char* kGpsWgsIdentifier = "GPS_WGS";
constexpr const char* kLidarIdentifier = "LIDAR";
constexpr const char* kOdometry6DoFIdentifier = "ODOMETRY_6DOF";
constexpr const char* kLoopClosureSensorIdentifier = "LOOP_CLOSURE";
constexpr const char* kAbsolute6DoFIdentifier = "ABSOLUTE_6DOF";
constexpr const char* kWheelOdometryIdentifier = "WHEEL_ODOMETRY";
constexpr const char* kPointCloudMapSensorIdentifier = "POINTCLOUD_MAP";

// Check if an integer is a valid sensor type. Function argument is an int to
// avoid having to cast to SensorType before passing the argument, since for
// integers not in the enum this has undefined behavior
inline bool isValidSensorType(uint8_t sensor_type) {
  return sensor_type > kUnknown && sensor_type < kInvalid;
}

inline SensorType convertStringToSensorType(const std::string& sensor_string) {
  const char* sensor_c_string = sensor_string.c_str();
  std::function<bool(const char*, const char*)> equals = [](const char* lhs,
                                                            const char* rhs) {
    return std::strcmp(lhs, rhs) == 0;
  };

  if (equals(sensor_c_string, kNCameraIdentifier)) {
    return SensorType::kNCamera;
  } else if (equals(sensor_c_string, kCameraIdentifier)) {
    return SensorType::kCamera;
  } else if (equals(sensor_c_string, kImuIdentifier)) {
    return SensorType::kImu;
  } else if (equals(sensor_c_string, kLoopClosureSensorIdentifier)) {
    return SensorType::kLoopClosureSensor;
  } else if (equals(sensor_c_string, kGpsUtmIdentifier)) {
    return SensorType::kGpsWgs;
  } else if (equals(sensor_c_string, kGpsWgsIdentifier)) {
    return SensorType::kGpsUtm;
  } else if (equals(sensor_c_string, kLidarIdentifier)) {
    return SensorType::kLidar;
  } else if (equals(sensor_c_string, kOdometry6DoFIdentifier)) {
    return SensorType::kOdometry6DoF;
  } else if (equals(sensor_c_string, kAbsolute6DoFIdentifier)) {
    return SensorType::kAbsolute6DoF;
  } else if (equals(sensor_c_string, kWheelOdometryIdentifier)) {
    return SensorType::kWheelOdometry;
  } else if (equals(sensor_c_string, kPointCloudMapSensorIdentifier)) {
    return SensorType::kPointCloudMapSensor;
  } else {
    LOG(ERROR) << "Unable to parse sensor type string: '" << sensor_string
               << "' to a sensor type!";
    return SensorType::kUnknown;
  }
}

}  // namespace vi_map

#endif  // SENSORS_SENSOR_TYPES_H_
