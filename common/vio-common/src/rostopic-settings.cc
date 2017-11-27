#include "vio-common/rostopic-settings.h"

#include <string>
#include <unordered_map>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(
    vio_camera_topic_suffix, "/image_raw",
    "Image name appended to camera topic.");
DEFINE_string(
    vio_hardware_features_topic, "hw_features",
    "Name of the hardware features topic.");

namespace vio_common {

RosTopicSettings::RosTopicSettings(
    const aslam::NCamera& camera_system, const vi_map::Imu& imu)
    : imu_topic(imu.getHardwareId()),
      camera_topic_suffix(FLAGS_vio_camera_topic_suffix),
      absolute_pose_topic(""),
      hardware_features_topic(FLAGS_vio_hardware_features_topic),
      gps_wgs_topic(""),
      gps_utm_topic("") {
  CHECK(!imu_topic.empty());
  for (size_t cam_idx = 0u; cam_idx < camera_system.getNumCameras();
       ++cam_idx) {
    camera_topic_cam_index_map.emplace(
        std::make_pair(
            camera_system.getCameraShared(cam_idx)->getLabel() +
                camera_topic_suffix,
            cam_idx));
  }
}

RosTopicSettings::RosTopicSettings(const vi_map::SensorManager& sensor_manager)
    : imu_topic(""),
      camera_topic_suffix(FLAGS_vio_camera_topic_suffix),
      absolute_pose_topic(""),
      hardware_features_topic(FLAGS_vio_hardware_features_topic),
      gps_wgs_topic(""),
      gps_utm_topic("") {
  aslam::NCameraIdSet ncamera_ids;
  sensor_manager.getAllNCameraIds(&ncamera_ids);
  CHECK(!ncamera_ids.empty());
  for (const aslam::NCameraId& ncamera_id : ncamera_ids) {
    CHECK(ncamera_id.isValid());
    const aslam::NCamera& ncamera = sensor_manager.getNCamera(ncamera_id);
    const size_t num_cameras = ncamera.numCameras();
    for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
      const std::string camera_topic =
          ncamera.getCamera(cam_idx).getLabel() + camera_topic_suffix;
      CHECK(camera_topic_cam_index_map.emplace(camera_topic, cam_idx).second);
    }
  }

  vi_map::Imu imu;
  if (sensor_manager.getSensor<vi_map::Imu>(&imu)) {
    imu_topic = imu.getHardwareId();
  }

  vi_map::Relative6DoFPose wheel_odometry;
  if (sensor_manager.getSensor<vi_map::Relative6DoFPose>(&wheel_odometry)) {
    absolute_pose_topic = wheel_odometry.getHardwareId();
  }

  vi_map::GpsUtm gps_utm;
  if (sensor_manager.getSensor<vi_map::GpsUtm>(&gps_utm)) {
    gps_utm_topic = gps_utm.getHardwareId();
  }

  vi_map::GpsWgs gps_wgs;
  if (sensor_manager.getSensor<vi_map::GpsWgs>(&gps_wgs)) {
    gps_wgs_topic = gps_wgs.getHardwareId();
  }
}

void RosTopicSettings::getAllValidTopics(std::vector<std::string>* topics) {
  CHECK_NOTNULL(topics)->clear();
  makeAbsoluteTopics();

  for (const vio_common::RosTopicSettings::CameraTopicIdxMap::value_type&
           topic_idx_pair : camera_topic_cam_index_map) {
    topics->emplace_back(topic_idx_pair.first);
  }
  if (!imu_topic.empty()) {
    topics->emplace_back(imu_topic);
  }

  if (!absolute_pose_topic.empty()) {
    topics->emplace_back(absolute_pose_topic);
  }

  if (!gps_wgs_topic.empty()) {
    topics->emplace_back(gps_wgs_topic);
  }

  if (!gps_utm_topic.empty()) {
    topics->emplace_back(gps_utm_topic);
  }
}

void RosTopicSettings::makeAbsoluteTopics() {
  CameraTopicIdxMap tmp_map;
  for (const CameraTopicIdxMap::value_type& topic_idx :
       camera_topic_cam_index_map) {
    CHECK(!topic_idx.first.empty());
    topic_idx.first[0] == '/'
        ? tmp_map.emplace(topic_idx.first, topic_idx.second)
        : tmp_map.emplace("/" + topic_idx.first, topic_idx.second);
  }
  camera_topic_cam_index_map.swap(tmp_map);

  if (!imu_topic.empty() && imu_topic[0] != '/') {
    imu_topic = '/' + imu_topic;
  }

  if (!absolute_pose_topic.empty() && absolute_pose_topic[0] != '/') {
    absolute_pose_topic = '/' + absolute_pose_topic;
  }

  if (!gps_wgs_topic.empty() && gps_wgs_topic[0] != '/') {
    gps_wgs_topic = '/' + gps_wgs_topic;
  }

  if (!gps_utm_topic.empty() && gps_utm_topic[0] != '/') {
    gps_utm_topic = '/' + gps_utm_topic;
  }
}
}  // namespace vio_common
