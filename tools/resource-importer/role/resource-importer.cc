#include <string>
#include <vector>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map-resources/resource-conversion.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/sensor-utils.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map-serialization.h>

#include "resource-importer/message-conversion.h"
#include "resource-importer/simple-rosbag-reader.h"

DEFINE_string(map_path, "", "Map input path.");
DEFINE_string(
    mission_id, "",
    "Id of mission to attach resources to. This flag is optional if map only "
    "contains one mission.");

DEFINE_string(map_output_path, "", "Map output path.");

DEFINE_string(rosbag_path, "", "Rosbag path.");

DEFINE_string(resource_topic, "", "ROS topic of resources to import.");

DEFINE_bool(
    visualize_image_resources, false,
    "If enabled the image resources will be visualized in an OpenCV window.");

// Either we pass in the camera info topic to parse the camera calibration and
// extrinsics (from tf).
DEFINE_string(
    camera_calibration_topic, "",
    "ROS topic of camera info for the resources.");
DEFINE_string(camera_extrinsics_imu_frame, "fisheye", "TF IMU frame name.");
DEFINE_string(camera_extrinsics_camera_frame, "depth", "TF camera frame name");
DEFINE_string(
    camera_extrinsics_base_sensor_id, "",
    "Sensor id of the base sensor the camera extrinsics are relative to. Only "
    "needed if the camera calibration and extrinsics are parsed from ROS tf "
    "and CameraInfo.");
// ...or we provide an sensor calibration file.
DEFINE_string(
    sensor_calibration_file, "",
    "Path to Sensor YAML file to load sensor calibration and extrinsics.");

DEFINE_int64(
    resource_time_offset_ns, 0u,
    "Time offset between camera/sensor and the clock of the primary motion "
    "estimation camera. The offset will be added to the resource timestamps.");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Check flags.
  CHECK(!FLAGS_map_path.empty()) << "You have to provide a maplab map to "
                                 << "attach the resources to! Flag: --map_path";
  if (FLAGS_map_output_path.empty()) {
    LOG(WARNING) << "You have not provided an output map, the resources will "
                 << "be attached to the input map, otherwise abort and use "
                 << "this flag: --map_output_path";
    FLAGS_map_output_path = FLAGS_map_path;
  }

  CHECK(!FLAGS_resource_topic.empty())
      << "You have to provide a rostopic for the camera resource!";

  // Get VIMap.
  vi_map::VIMap map;
  const bool result =
      vi_map::serialization::loadMapFromFolder(FLAGS_map_path, &map);
  CHECK(result) << "Map loading failed.";

  // Set the resource folder to an external folder, such that adding resources
  // to this map does not pollute the map folder.
  const std::string kTemporaryResourceFolder = "/tmp/vi_map_resources";
  common::removePath(kTemporaryResourceFolder);
  map.useExternalResourceFolder(kTemporaryResourceFolder);

  // Get the mission that we want to add the resource to.
  CHECK_GT(map.numMissions(), 0u);
  vi_map::MissionIdList mission_ids;
  map.getAllMissionIds(&mission_ids);
  vi_map::MissionId selected_mission_id;
  if (mission_ids.size() > 1u) {
    CHECK(!FLAGS_mission_id.empty())
        << "If the map contains more than one mission, you need to provide the "
        << "mission id of the mission you want to add the resources to! Flag: "
        << "--mission_id";
    CHECK(selected_mission_id.fromHexString(FLAGS_mission_id))
        << "Invalid mission id: " << FLAGS_mission_id;
    CHECK(map.hasMission(selected_mission_id))
        << "This map does not contain a mission with id: " << FLAGS_mission_id;
  } else {
    CHECK_EQ(mission_ids.size(), 1u);
    selected_mission_id = mission_ids[0];
  }
  vi_map::VIMission& selected_mission = map.getMission(selected_mission_id);

  // Prepare camera/sensor id.
  aslam::SensorId sensor_id;
  sensor_id.setInvalid();

  SimpleRosbagSource rosbag_source(
      FLAGS_rosbag_path, FLAGS_resource_topic, FLAGS_camera_calibration_topic,
      FLAGS_camera_extrinsics_imu_frame, FLAGS_camera_extrinsics_camera_frame);

  const std::string window_name =
      "Imported image from '" + FLAGS_resource_topic + "'";
  if (FLAGS_visualize_image_resources) {
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  }

  // NOTE: We either get the camera calibration and extrinsics from the
  // CameraInfo topic and tfs or we load a sensor calibration yaml file. The
  // latter is preferred and should support other sensor types, such as lidars.

  // Camera calibration and extrinsics retrieved from CameraInfo and tf:
  bool got_calibration_from_camera_info = false;
  bool got_tf_from_extrinsics = false;
  sensor_msgs::CameraInfoConstPtr resource_camera_info_msg;
  geometry_msgs::Transform T_C_I_msg;

  // Camera calibration and extrinsics retrieved from the sensor calibration
  // yaml file.
  bool got_sensor_from_file = false;

  if (!FLAGS_sensor_calibration_file.empty()) {
    CHECK(common::fileExists(FLAGS_sensor_calibration_file))
        << "Could not find sensor calibration YAML file at: "
        << FLAGS_sensor_calibration_file << "!";

    vi_map::SensorManager sensor_manager;
    if (!sensor_manager.deserializeFromFile(FLAGS_sensor_calibration_file)) {
      LOG(FATAL) << "Failed to read the sensor calibration from '"
                 << FLAGS_sensor_calibration_file << "'!";
    }

    if (sensor_manager.getNumSensors() != 2u) {
      LOG(FATAL) << "The sensor calibration file should only contain the "
                    "resource sensor and its base sensor (=2)!";
    }

    aslam::SensorIdSet all_sensor_ids;
    sensor_manager.getAllSensorIds(&all_sensor_ids);
    CHECK_EQ(all_sensor_ids.size(), 2u);
    for (const aslam::SensorId& any_sensor_id : all_sensor_ids) {
      if (!sensor_manager.isBaseSensor(any_sensor_id)) {
        sensor_id = any_sensor_id;
      }
    }
    CHECK(sensor_id.isValid());

    // Add sensor to the maps sensor manager.
    map.getSensorManager().merge(sensor_manager);

    got_sensor_from_file = true;
  } else if (!FLAGS_camera_calibration_topic.empty()) {
    CHECK(!FLAGS_camera_extrinsics_imu_frame.empty())
        << "You have to provide a tf frame for the IMU!";
    CHECK(!FLAGS_camera_extrinsics_camera_frame.empty())
        << "You have to provide a tf frame for the depth camera!";

    aslam::generateId(&sensor_id);

    std::function<void(sensor_msgs::CameraInfoConstPtr)> camera_info_callback =
        [&](sensor_msgs::CameraInfoConstPtr camera_info_msg) {
          CHECK(camera_info_msg);
          const int64_t timestamp_ns = camera_info_msg->header.stamp.toNSec();
          CHECK_GE(timestamp_ns, 0);

          LOG(INFO) << "Found CameraInfo at " << timestamp_ns << " ns";
          resource_camera_info_msg = camera_info_msg;
          got_calibration_from_camera_info = true;
        };
    rosbag_source.setCameraInfoCallback(camera_info_callback);

    std::function<void(geometry_msgs::Transform)> camera_extrinsics_callback =
        [&](const geometry_msgs::Transform& camera_extrinsics_msg) {
          LOG(INFO) << "Found Camera extrinsics";
          T_C_I_msg = camera_extrinsics_msg;
          got_tf_from_extrinsics = true;
        };
    rosbag_source.setCameraExtrinsicsCallback(camera_extrinsics_callback);
  } else {
    LOG(ERROR) << "\n\nCannot import this resource due to the lack of camera "
               << "calibration and extrinsics.\n"
               << "There are two options:\n\t"
               << "--camera_calibration_topic\n\t"
               << "--camera_extrinsics_imu_frame\n\t"
               << "--camera_extrinsics_camera_frame\n\t"
               << "=> Import from CameraInfo topic and tf.\n"
               << "OR\n\t"
               << "--sensor_calibration_file\n\t"
               << "=> Import from calibration file.\n ";
    return -1;
  }

  std::function<void(sensor_msgs::ImageConstPtr)> image_callback =
      [&](sensor_msgs::ImageConstPtr image_message) {
        CHECK(image_message);
        int64_t timestamp_ns = image_message->header.stamp.toNSec();
        CHECK_GE(timestamp_ns, 0);

        // Apply time offset.
        timestamp_ns += FLAGS_resource_time_offset_ns;

        if (image_message->encoding ==
            sensor_msgs::image_encodings::TYPE_16UC1) {
          LOG(INFO) << "Found depth map at " << timestamp_ns << " ns";

          cv::Mat image;
          convertDepthImageMessage(image_message, &image);

          if (FLAGS_visualize_image_resources) {
            cv::imshow(window_name, image);
            constexpr int kDepthMapVisualizationWaitTimeMs = 10;
            cv::waitKey(kDepthMapVisualizationWaitTimeMs);
          }

          map.addSensorResource(
              backend::ResourceType::kRawDepthMap, sensor_id, timestamp_ns,
              image, &selected_mission);
        } else if (
            image_message->encoding ==
            sensor_msgs::image_encodings::TYPE_32FC1) {
          LOG(INFO) << "Found metric depth map at " << timestamp_ns;

          cv::Mat image;
          convertFloatDepthImageMessage(image_message, &image);

          if (FLAGS_visualize_image_resources) {
            cv::imshow(window_name, image);
            constexpr int kDepthMapVisualizationWaitTimeMs = 10;
            cv::waitKey(kDepthMapVisualizationWaitTimeMs);
          }

          map.addSensorResource(
              backend::ResourceType::kRawDepthMap, sensor_id, timestamp_ns,
              image, &selected_mission);
        } else if (
            image_message->encoding ==
                sensor_msgs::image_encodings::TYPE_8UC3 ||
            image_message->encoding == sensor_msgs::image_encodings::BGR8 ||
            image_message->encoding == sensor_msgs::image_encodings::RGB8) {
          LOG(INFO) << "Found color image at " << timestamp_ns;

          cv::Mat image;
          convertColorImageMessage(image_message, &image);

          cv::imshow(window_name, image);
          constexpr int kDepthMapVisualizationWaitTimeMs = 10;
          cv::waitKey(kDepthMapVisualizationWaitTimeMs);

          map.addSensorResource(
              backend::ResourceType::kRawColorImage, sensor_id, timestamp_ns,
              image, &selected_mission);
        } else {
          LOG(FATAL)
              << "This image resource type is currently not supported by "
                 "this importer! encoding: "
              << image_message->encoding;
        }
      };
  rosbag_source.setImageCallback(image_callback);

  std::function<void(sensor_msgs::PointCloud2ConstPtr)> pointcloud_callback =
      [&](sensor_msgs::PointCloud2ConstPtr point_cloud_msg) {
        CHECK(point_cloud_msg);
        int64_t timestamp_ns = point_cloud_msg->header.stamp.toNSec();
        CHECK_GE(timestamp_ns, 0);

        // Apply time offset.
        timestamp_ns += FLAGS_resource_time_offset_ns;

        LOG(INFO) << "Found pointcloud at " << timestamp_ns << "ns";

        resources::PointCloud maplab_pointcloud;
        backend::convertPointCloudType(*point_cloud_msg, &maplab_pointcloud);

        if (maplab_pointcloud.xyz.size() > 0u) {
          if (backend::hasColorInformation(maplab_pointcloud)) {
            map.addSensorResource(
                backend::ResourceType::kPointCloudXYZRGBN, sensor_id,
                timestamp_ns, maplab_pointcloud, &selected_mission);
          } else if (backend::hasScalarInformation(maplab_pointcloud)) {
            map.addSensorResource(
                backend::ResourceType::kPointCloudXYZI, sensor_id, timestamp_ns,
                maplab_pointcloud, &selected_mission);
          } else {
            map.addSensorResource(
                backend::ResourceType::kPointCloudXYZ, sensor_id, timestamp_ns,
                maplab_pointcloud, &selected_mission);
          }
        } else {
          LOG(WARNING) << "Received empty point cloud, ignoring...";
        }
      };
  rosbag_source.setPointcloudCallback(pointcloud_callback);

  rosbag_source.readRosbag();

  if (got_calibration_from_camera_info && got_tf_from_extrinsics) {
    // Camera with extrinsics (T_C_I, C: Camera, I: IMU frame).
    aslam::Camera::Ptr aslam_camera;
    std::pair<aslam::Camera*, aslam::Transformation> camera_with_extrinsics;
    convertCameraInfo(
        resource_camera_info_msg, T_C_I_msg, &camera_with_extrinsics);
    aslam_camera.reset(camera_with_extrinsics.first);
    CHECK(aslam_camera);

    std::vector<aslam::Camera::Ptr> camera_list = {aslam_camera};
    aslam::TransformationVector T_C_Cn_vec = {camera_with_extrinsics.second};

    aslam::NCamera::UniquePtr ncamera(
        new aslam::NCamera(sensor_id, T_C_Cn_vec, camera_list, ""));

    aslam::Transformation T_Cn_B;
    T_Cn_B.setIdentity();

    CHECK(!FLAGS_camera_extrinsics_base_sensor_id.empty())
        << "A base sensor id is needed position the new sensor with respect to "
           "the others!";
    aslam::SensorId base_sensor_id;
    base_sensor_id.fromHexString(FLAGS_camera_extrinsics_base_sensor_id);

    map.getSensorManager().addSensor<aslam::NCamera>(
        std::move(ncamera), base_sensor_id, T_Cn_B);
  } else if (got_sensor_from_file) {
    // Nothing to do. Sensor is already added to the sensor manager.
  } else {
    LOG(FATAL) << "Failed to get camera calibration and extrinsics!";
  }

  backend::SaveConfig save_config;
  save_config.overwrite_existing_files = true;
  save_config.move_resources_when_migrating = false;
  save_config.migrate_resources_settings = backend::SaveConfig::
      MigrateResourcesSettings::kMigrateResourcesToMapFolder;
  const bool write_map_result = vi_map::serialization::saveMapToFolder(
      FLAGS_map_output_path, save_config, &map);

  CHECK(write_map_result) << "Saving the output map to the file system failed.";

  return 0;
}
