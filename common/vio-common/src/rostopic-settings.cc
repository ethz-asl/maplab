#include "vio-common/rostopic-settings.h"

#include <string>
#include <unordered_map>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/external-features.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <sensors/lidar.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/wheel-odometry-sensor.h>
#include <vi-map/sensor-utils.h>

DEFINE_string(
    vio_camera_topic_suffix, "/image_raw",
    "Image name appended to camera topic.");
DEFINE_string(
    vio_hardware_features_topic, "hw_features",
    "Name of the hardware features topic.");

namespace vio_common {

RosTopicSettings::RosTopicSettings(
    const aslam::NCamera& camera_system, const vi_map::Imu& imu)
    : imu_topic(imu.getTopic()),
      camera_topic_suffix(FLAGS_vio_camera_topic_suffix),
      hardware_features_topic(FLAGS_vio_hardware_features_topic),
      gps_wgs_topic(""),
      gps_utm_topic(""),
      odometry_6dof_topic() {
  CHECK(!imu_topic.empty());
  std::string camera_topic_base;
  for (size_t cam_idx = 0u; cam_idx < camera_system.getNumCameras();
       ++cam_idx) {
    camera_topic_base = camera_system.getCameraShared(cam_idx)->getTopic();
    // remove spurious "/" in topic names
    camera_topic_base.erase(camera_topic_base.find_last_not_of("/") + 1);
    camera_topic_suffix.erase(0, camera_topic_suffix.find_first_not_of("/"));
    camera_topic_cam_index_map.emplace(
        std::make_pair(camera_topic_base + "/" + camera_topic_suffix, cam_idx));
  }
}

RosTopicSettings::RosTopicSettings(const vi_map::SensorManager& sensor_manager)
    : imu_topic(""),
      camera_topic_suffix(FLAGS_vio_camera_topic_suffix),
      hardware_features_topic(FLAGS_vio_hardware_features_topic),
      gps_wgs_topic(""),
      gps_utm_topic(""),
      odometry_6dof_topic() {
  aslam::NCamera::Ptr ncamera = getSelectedNCamera(sensor_manager);
  if (ncamera) {
    const size_t num_cameras = ncamera->numCameras();
    std::string camera_topic_base;
    for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
      camera_topic_base = ncamera->getCamera(cam_idx).getTopic();
      camera_topic_base.erase(camera_topic_base.find_last_not_of("/") + 1);
      camera_topic_suffix.erase(0, camera_topic_suffix.find_first_not_of("/"));

      const std::string camera_topic =
          camera_topic_base + "/" + camera_topic_suffix;

      CHECK(!camera_topic.empty())
          << "Camera " << cam_idx << " of NCamera ('" << ncamera->getId()
          << "') has an empty ROS topic!";
      CHECK(camera_topic_cam_index_map.emplace(camera_topic, cam_idx).second)
          << "Topic '" << camera_topic
          << "' already exists in the ROS topic settings, meaning another "
             "camera already subscribes to this topic!";
    }
  }

  vi_map::Imu::Ptr imu = getSelectedImu(sensor_manager);
  if (imu) {
    imu_topic = imu->getTopic();

    CHECK(!imu_topic.empty()) << "The selected IMU ('" << imu->getId()
                              << "') has an empty ROS topic!";
  }

  vi_map::Lidar::Ptr lidar = getSelectedLidar(sensor_manager);
  if (lidar) {
    const std::string lidar_topic = lidar->getTopic();

    CHECK(!lidar_topic.empty()) << "The selected LIDAR ('" << lidar->getId()
                                << "') has an empty ROS topic!";
    CHECK(
        lidar_topic_sensor_id_map.emplace(lidar_topic, lidar->getId()).second);
  }

  // TODO(mfehr): reenable multi-lidar support.
  // Set lidar_sensor_ids;
  // sensor_manager->getAllSensorIdsOfType(
  //     vi_map::SensorType::kLidar, &lidar_sensor_ids);
  //
  // for (const & lidar_sensor_id : lidar_sensor_ids) {
  //   CHECK(lidar_sensor_id.isValid());
  //   const std::string& hardware_id =
  //       sensor_manager->getSensor<vi_map::Lidar>(lidar_sensor_id).getTopic();
  //   CHECK(!hardware_id.empty());
  //   CHECK(
  //       lidar_topic_sensor_id_map.emplace(hardware_id,
  //       lidar_sensor_id).second);
  // }
  // CHECK_EQ(lidar_topic_sensor_id_map.size(), lidar_sensor_ids.size());

  vi_map::Odometry6DoF::Ptr odometry_6dof_sensor =
      getSelectedOdometry6DoFSensor(sensor_manager);
  if (odometry_6dof_sensor) {
    odometry_6dof_topic = std::make_pair(
        odometry_6dof_sensor->getTopic(), odometry_6dof_sensor->getId());

    CHECK(!odometry_6dof_topic.first.empty())
        << "The selected Odometry 6DOF sensor ('"
        << odometry_6dof_sensor->getId() << "') has an empty ROS topic!";
  }

  vi_map::Absolute6DoF::Ptr absolute_6dof_sensor =
      getSelectedAbsolute6DoFSensor(sensor_manager);
  if (absolute_6dof_sensor) {
    const std::string& absolute_6dof_topic = absolute_6dof_sensor->getTopic();

    CHECK(!absolute_6dof_topic.empty())
        << "The selected Absolute 6DOF sensor ('"
        << absolute_6dof_sensor->getId() << "') has an empty ROS topic!";
    CHECK(absolute_6dof_topic_map
              .emplace(absolute_6dof_topic, absolute_6dof_sensor->getId())
              .second);
  }

  vi_map::WheelOdometry::Ptr wheel_odometry_sensor =
      getSelectedWheelOdometrySensor(sensor_manager);
  if (wheel_odometry_sensor) {
    const std::string& wheel_odometry_topic = wheel_odometry_sensor->getTopic();

    CHECK(!wheel_odometry_topic.empty())
        << "The selected wheel odometry sensor ('"
        << wheel_odometry_sensor->getId() << "') has an empty ROS topic!";
    CHECK(wheel_odometry_topic_map
              .emplace(wheel_odometry_topic, wheel_odometry_sensor->getId())
              .second);
  }

  vi_map::LoopClosureSensor::Ptr loop_closure_sensor =
      getSelectedLoopClosureSensor(sensor_manager);
  if (loop_closure_sensor) {
    const std::string& loop_closure_topic = loop_closure_sensor->getTopic();

    CHECK(!loop_closure_topic.empty())
        << "The selected wheel odometry sensor ('"
        << loop_closure_sensor->getId() << "') has an empty ROS topic!";
    CHECK(loop_closure_topic_map
              .emplace(loop_closure_topic, loop_closure_sensor->getId())
              .second);
  }

  vi_map::GpsUtm::Ptr gps_utm = getSelectedGpsUtmSensor(sensor_manager);
  if (gps_utm) {
    gps_utm_topic = gps_utm->getTopic();

    CHECK(!gps_utm_topic.empty())
        << "The selected GPS UTM sensor ('" << gps_utm->getId()
        << "') has an empty ROS topic!";
  }

  vi_map::GpsWgs::Ptr gps_wgs = getSelectedGpsWgsSensor(sensor_manager);
  if (gps_wgs) {
    gps_wgs_topic = gps_wgs->getTopic();

    CHECK(!gps_wgs_topic.empty())
        << "The selected GPS WGS sensor ('" << gps_wgs->getId()
        << "') has an empty ROS topic!";
  }

  vi_map::PointCloudMapSensor::Ptr point_cloud_map_sensor =
      getSelectedPointCloudMapSensor(sensor_manager);
  if (point_cloud_map_sensor) {
    const std::string& pointcloud_map_topic =
        point_cloud_map_sensor->getTopic();

    CHECK(!pointcloud_map_topic.empty())
        << "The selected point cloud map sensor ('"
        << point_cloud_map_sensor->getId() << "') has an empty ROS topic!";
    CHECK(pointcloud_map_topic_map
              .emplace(pointcloud_map_topic, point_cloud_map_sensor->getId())
              .second);
  }

  aslam::SensorIdSet all_external_feature_sensor_ids;
  sensor_manager.getAllSensorIdsOfType(
      vi_map::SensorType::kExternalFeatures, &all_external_feature_sensor_ids);
  for (const aslam::SensorId sensor_id : all_external_feature_sensor_ids) {
    vi_map::ExternalFeatures::Ptr external_features_sensor =
        sensor_manager.getSensorPtr<vi_map::ExternalFeatures>(sensor_id);
    CHECK(external_features_sensor);

    const std::string& external_features_topic =
        external_features_sensor->getTopic();
    CHECK(!external_features_topic.empty())
        << "The selected external feature sensor ('"
        << external_features_sensor->getId() << "') has an empty ROS topic!";
    CHECK(
        external_features_topic_map
            .emplace(external_features_topic, external_features_sensor->getId())
            .second);
  }
}

void RosTopicSettings::getAllValidTopics(std::vector<std::string>* topics) {
  CHECK_NOTNULL(topics)->clear();
  makeAbsoluteTopics();

  for (const CameraTopicIdxMap::value_type& camera_topic_idx_pair :
       camera_topic_cam_index_map) {
    topics->emplace_back(camera_topic_idx_pair.first);
  }

  if (!imu_topic.empty()) {
    topics->emplace_back(imu_topic);
  }

  if (!gps_wgs_topic.empty()) {
    topics->emplace_back(gps_wgs_topic);
  }

  if (!gps_utm_topic.empty()) {
    topics->emplace_back(gps_utm_topic);
  }

  for (const TopicSensorIdMap::value_type& lidar_topic_sensor_id_pair :
       lidar_topic_sensor_id_map) {
    topics->emplace_back(lidar_topic_sensor_id_pair.first);
  }

  if (!odometry_6dof_topic.first.empty()) {
    topics->emplace_back(odometry_6dof_topic.first);
  }

  for (const TopicSensorIdMap::value_type& absolute_6dof_topic_sensor_id_pair :
       absolute_6dof_topic_map) {
    topics->emplace_back(absolute_6dof_topic_sensor_id_pair.first);
  }

  for (const TopicSensorIdMap::value_type& loop_closure_topic_sensor_id_pair :
       loop_closure_topic_map) {
    topics->emplace_back(loop_closure_topic_sensor_id_pair.first);
  }

  for (const TopicSensorIdMap::value_type& pointcloud_map_topic_sensor_id_pair :
       pointcloud_map_topic_map) {
    topics->emplace_back(pointcloud_map_topic_sensor_id_pair.first);
  }
}

void RosTopicSettings::makeAbsoluteTopics() {
  CameraTopicIdxMap camera_topics_tmp_map;
  for (const CameraTopicIdxMap::value_type& topic_idx :
       camera_topic_cam_index_map) {
    CHECK(!topic_idx.first.empty());
    topic_idx.first[0] == '/'
        ? camera_topics_tmp_map.emplace(topic_idx.first, topic_idx.second)
        : camera_topics_tmp_map.emplace(
              '/' + topic_idx.first, topic_idx.second);
  }
  camera_topic_cam_index_map.swap(camera_topics_tmp_map);

  if (!imu_topic.empty() && imu_topic[0] != '/') {
    imu_topic = '/' + imu_topic;
  }

  if (!gps_wgs_topic.empty() && gps_wgs_topic[0] != '/') {
    gps_wgs_topic = '/' + gps_wgs_topic;
  }

  if (!gps_utm_topic.empty() && gps_utm_topic[0] != '/') {
    gps_utm_topic = '/' + gps_utm_topic;
  }

  TopicSensorIdMap lidar_topics_tmp_map;
  for (const TopicSensorIdMap::value_type& topic_sensor_id :
       lidar_topic_sensor_id_map) {
    CHECK(!topic_sensor_id.first.empty());
    topic_sensor_id.first[0] == '/'
        ? lidar_topics_tmp_map.emplace(
              topic_sensor_id.first, topic_sensor_id.second)
        : lidar_topics_tmp_map.emplace(
              '/' + topic_sensor_id.first, topic_sensor_id.second);
  }
  lidar_topic_sensor_id_map.swap(lidar_topics_tmp_map);

  if (!odometry_6dof_topic.first.empty() &&
      odometry_6dof_topic.first[0] != '/') {
    odometry_6dof_topic.first = '/' + odometry_6dof_topic.first;
  }

  TopicSensorIdMap absolute_6dof_topic_map_tmp;
  for (const TopicSensorIdMap::value_type& topic_sensor_id :
       absolute_6dof_topic_map) {
    CHECK(!topic_sensor_id.first.empty());
    topic_sensor_id.first[0] == '/'
        ? absolute_6dof_topic_map_tmp.emplace(
              topic_sensor_id.first, topic_sensor_id.second)
        : absolute_6dof_topic_map_tmp.emplace(
              '/' + topic_sensor_id.first, topic_sensor_id.second);
  }
  absolute_6dof_topic_map.swap(absolute_6dof_topic_map_tmp);

  TopicSensorIdMap loop_closure_topic_map_tmp;
  for (const TopicSensorIdMap::value_type& topic_sensor_id :
       loop_closure_topic_map) {
    CHECK(!topic_sensor_id.first.empty());
    topic_sensor_id.first[0] == '/'
        ? loop_closure_topic_map_tmp.emplace(
              topic_sensor_id.first, topic_sensor_id.second)
        : loop_closure_topic_map_tmp.emplace(
              '/' + topic_sensor_id.first, topic_sensor_id.second);
  }
  loop_closure_topic_map.swap(loop_closure_topic_map_tmp);

  TopicSensorIdMap pointcloud_map_topic_map_tmp;
  for (const TopicSensorIdMap::value_type& topic_sensor_id :
       pointcloud_map_topic_map) {
    CHECK(!topic_sensor_id.first.empty());
    topic_sensor_id.first[0] == '/'
        ? pointcloud_map_topic_map_tmp.emplace(
              topic_sensor_id.first, topic_sensor_id.second)
        : pointcloud_map_topic_map_tmp.emplace(
              '/' + topic_sensor_id.first, topic_sensor_id.second);
  }
  pointcloud_map_topic_map.swap(pointcloud_map_topic_map_tmp);
}
}  // namespace vio_common
