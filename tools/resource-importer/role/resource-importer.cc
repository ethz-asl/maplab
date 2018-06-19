#include <string>
#include <vector>

#include <Eigen/Core>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <vi-map/check-map-consistency.h>
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
DEFINE_string(camera_calibration_topic, "",
              "ROS topic of camera info for the resources.");
DEFINE_string(camera_extrinsics_imu_frame, "fisheye", "TF IMU frame name.");
DEFINE_string(camera_extrinsics_camera_frame, "depth", "TF camera frame name");
// ...or we provide an ncamera file.
DEFINE_string(
    camera_calibration_file, "",
    "Path to NCamera YAML file to load camera calibration and extrinsics.");

DEFINE_uint64(subsample_resources_by_factor, 1,
              "Subsample the resource association by this factor. (i.e., take "
              "every Nth resource)");

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
  aslam::CameraId camera_id;
  common::generateId(&camera_id);

  SimpleRosbagSource rosbag_source(
      FLAGS_rosbag_path, FLAGS_resource_topic, FLAGS_camera_calibration_topic,
      FLAGS_camera_extrinsics_imu_frame, FLAGS_camera_extrinsics_camera_frame);

  const std::string window_name =
      "Imported image from '" + FLAGS_resource_topic + "'";
  if (FLAGS_visualize_image_resources) {
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  }

  uint64_t resource_index = 0;

  std::function<void(sensor_msgs::ImageConstPtr)> image_callback = [&](
      sensor_msgs::ImageConstPtr image_message) {
    CHECK(image_message);
    const int64_t timestamp_ns = image_message->header.stamp.toNSec();
    CHECK_GE(timestamp_ns, 0);

    if (resource_index++ % FLAGS_subsample_resources_by_factor != 0) {
      return;
    }

    if (image_message->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      LOG(INFO) << "Found depth map at " << timestamp_ns << " ns";

      cv::Mat image;
      convertDepthImageMessage(image_message, &image);

      if (FLAGS_visualize_image_resources) {
        cv::imshow(window_name, image);
        constexpr int kDepthMapVisualizationWaitTimeMs = 10;
        cv::waitKey(kDepthMapVisualizationWaitTimeMs);
      }

      map.storeOptionalRawDepthMap(camera_id, timestamp_ns, image,
                                   &selected_mission);
    } else if (image_message->encoding ==
               sensor_msgs::image_encodings::TYPE_8UC3) {
      LOG(INFO) << "Found color image at " << timestamp_ns;

      cv::Mat image;
      convertColorImageMessage(image_message, &image);

      cv::imshow(window_name, image);
      constexpr int kDepthMapVisualizationWaitTimeMs = 10;
      cv::waitKey(kDepthMapVisualizationWaitTimeMs);

      map.storeOptionalRawColorImage(camera_id, timestamp_ns, image,
                                     &selected_mission);
    } else {
      LOG(FATAL) << "This image resource type is currently not supported by "
                    "this importer! encoding: "
                 << image_message->encoding;
    }
  };
  rosbag_source.setImageCallback(image_callback);

  std::function<void(sensor_msgs::PointCloud2ConstPtr)> pointcloud_callback =
      [&](sensor_msgs::PointCloud2ConstPtr point_cloud_msg) {
        CHECK(point_cloud_msg);
        const int64_t timestamp_ns = point_cloud_msg->header.stamp.toNSec();
        CHECK_GE(timestamp_ns, 0);

        if (resource_index++ % FLAGS_subsample_resources_by_factor != 0) {
          return;
        }

        LOG(INFO) << "Found Pointcloud at " << timestamp_ns << "ns";

        resources::PointCloud maplab_pointcloud;
        convertPointCloudMessage(point_cloud_msg, &maplab_pointcloud);

        map.storeOptionalPointCloudXYZRGBN(
            camera_id, timestamp_ns, maplab_pointcloud, &selected_mission);
      };
  rosbag_source.setPointcloudCallback(pointcloud_callback);

  // NOTE: We either get the camera calibration and extrinsics from the
  // CameraInfo topic and tfs or we load the NCamera file. The latter is
  // preferred.

  // Camera calibration and extrinsics retrieved from CameraInfo and tf:
  bool got_calibration_from_camera_info = false;
  bool got_tf_from_extrinsics = false;
  sensor_msgs::CameraInfoConstPtr resource_camera_info_msg;
  geometry_msgs::Transform T_C_I_msg;

  // Camera calibration and extrinsics retrieved from NCamera yaml file.
  bool got_ncamera_from_file = false;
  aslam::NCamera::Ptr ncamera;

  if (!FLAGS_camera_calibration_file.empty()) {
    CHECK(common::fileExists(FLAGS_camera_calibration_file))
        << "Could not find NCamera YAML file at: "
        << FLAGS_camera_calibration_file << "!";

    ncamera = aslam::NCamera::loadFromYaml(FLAGS_camera_calibration_file);
    CHECK(ncamera);

    const bool has_one_camera = ncamera->getNumCameras() == 1u;
    if (has_one_camera) {
      LOG(INFO) << "Loaded camera calibration and extrinsics from: "
                << FLAGS_camera_calibration_file;
      got_ncamera_from_file = true;
    } else {
      LOG(FATAL) << "The NCamera loaded from " << FLAGS_camera_calibration_file
                 << " contains none or more than one camera!";
    }
  } else if (!FLAGS_camera_calibration_topic.empty()) {
    CHECK(!FLAGS_camera_extrinsics_imu_frame.empty())
        << "You have to provide a tf frame for the IMU!";
    CHECK(!FLAGS_camera_extrinsics_camera_frame.empty())
        << "You have to provide a tf frame for the depth camera!";

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
               << "--camera_calibration_file\n\t"
               << "=> Import from NCamera calibration file.\n ";
    return -1;
  }

  rosbag_source.readRosbag();

  aslam::Camera::UniquePtr aslam_camera;
  aslam::Transformation T_C_I;
  if (got_calibration_from_camera_info && got_tf_from_extrinsics) {
    // Camera with extrinsics (T_C_I, C: Camera, I: IMU frame).
    std::pair<aslam::Camera*, aslam::Transformation> camera_with_extrinsics;
    convertCameraInfo(resource_camera_info_msg, T_C_I_msg,
                      &camera_with_extrinsics);
    aslam_camera.reset(camera_with_extrinsics.first);
    CHECK(aslam_camera);
    T_C_I = camera_with_extrinsics.second;
  } else if (got_ncamera_from_file) {
    CHECK(ncamera);
    aslam_camera.reset(ncamera->getCamera(0).clone());
    T_C_I = ncamera->get_T_C_B(0);
  } else {
    LOG(FATAL) << "Failed to get camera calibration and extrinsics!";
  }

  // Add calibration and extrinsics to VIMap.
  CHECK(aslam_camera);
  // Replace the camera id with the id we've been using to attach the resources.
  aslam_camera->setId(camera_id);
  map.getSensorManager().addOptionalCameraWithExtrinsics(*aslam_camera, T_C_I,
                                                         selected_mission_id);

  backend::SaveConfig save_config;
  save_config.overwrite_existing_files = true;
  save_config.move_resources_when_migrating = false;
  save_config.migrate_resources_settings = backend::SaveConfig::
      MigrateResourcesSettings::kMigrateResourcesToMapFolder;
  const bool write_map_result = vi_map::serialization::saveMapToFolder(
      FLAGS_map_output_path, save_config, &map);

  CHECK(write_map_result) << "Saving the output map to the file system failed.";

  CHECK(vi_map::checkMapConsistency(map));

  LOG(INFO) << "Finished outputting map and resources.";

  return 0;
}
