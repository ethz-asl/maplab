#ifndef VIO_COMMON_ROSTOPIC_SETTINGS_H_
#define VIO_COMMON_ROSTOPIC_SETTINGS_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <aslam/common/unique-id.h>
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

  typedef std::unordered_map<std::string, aslam::SensorId> TopicSensorIdMap;

  typedef std::unordered_map<std::string, size_t> CameraTopicIdxMap;

  // List of camera ROS topics. Topic list order matches the calibration
  // storage order.
  CameraTopicIdxMap camera_topic_cam_index_map;
  // ROS topic for the imu messages.
  std::string imu_topic;
  // Image name which is appended to the camera name defined in the parameter
  // file.
  std::string camera_topic_suffix;
  // Hardware feature topic; currently only working with Strawberry.
  std::string hardware_features_topic;
  // ROS topic for GPS WGS measurement data.
  std::string gps_wgs_topic;
  // ROS topic for GPS UTM measurement data.
  std::string gps_utm_topic;
  // ROS topics for lidar point cloud data.
  TopicSensorIdMap lidar_topic_sensor_id_map;
  // ROS topic for the odometry 6DOF constraints between odometry frame and the
  // vertices(e.g. external odometry source).
  std::pair<std::string, aslam::SensorId> odometry_6dof_topic;
  // ROS topic for loop closure constraints between two timestamps.
  TopicSensorIdMap loop_closure_topic_map;
  // ROS topic for absolute 6DOF constraints bewtween the sensor frame and the
  // world frame.
  TopicSensorIdMap absolute_6dof_topic_map;
  // ROS topic for wheel odometry constraints bewtween the sensor frame and the
  // world frame.
  TopicSensorIdMap wheel_odometry_topic_map;
  // ROS topic for (accumulated) dense point clouds representing part or the
  // whole map.
  TopicSensorIdMap pointcloud_map_topic_map;

  void getAllValidTopics(std::vector<std::string>* topics);

  void makeAbsoluteTopics();
};

}  // namespace vio_common

#endif  // VIO_COMMON_ROSTOPIC_SETTINGS_H_
