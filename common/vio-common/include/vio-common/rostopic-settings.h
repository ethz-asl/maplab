#ifndef VIO_COMMON_ROSTOPIC_SETTINGS_H_
#define VIO_COMMON_ROSTOPIC_SETTINGS_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sensors/imu.h>
#include <vi-map/sensor-manager.h>

namespace aslam {
class NCamera;
}

namespace vio_common {

struct RosTopicSettings {
  RosTopicSettings(const aslam::NCamera& camera_system, const vi_map::Imu& imu);
  explicit RosTopicSettings(const vi_map::SensorManager& sensor_manager);

  // List of camera ROS topics. Topic list order matches the calibration
  // storage order.
  typedef std::unordered_map<std::string, size_t> CameraTopicIdxMap;
  CameraTopicIdxMap camera_topic_cam_index_map;
  // ROS topic for the imu messages.
  std::string imu_topic;
  // Image name which is appended to the camera name defined in the parameter
  // file.
  std::string camera_topic_suffix;
  // ROS topic for the absolute pose data (e.g. wheel-odometry).
  std::string absolute_pose_topic;
  // Hardware feature topic; currently only working with Strawberry.
  std::string hardware_features_topic;
  // ROS topic for GPS WGS measurement data.
  std::string gps_wgs_topic;
  // ROS topic for GPS UTM measurement data.
  std::string gps_utm_topic;

  void getAllValidTopics(std::vector<std::string>* topics);

  void makeAbsoluteTopics();
};

}  // namespace vio_common

#endif  // VIO_COMMON_ROSTOPIC_SETTINGS_H_
