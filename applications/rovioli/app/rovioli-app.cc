#include <memory>

#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <maplab-ros-common/gflags-interface.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <ros/ros.h>
#include <sensors/imu.h>
#include <sensors/sensor-types.h>
#include <signal.h>
#include <vi-map/sensor-utils.h>
#include <vi-map/vi-map-serialization.h>

#include "rovioli/rovioli-node.h"

DEFINE_string(
    vio_localization_map_folder, "",
    "Path to a localization summary map or a full VI-map used for "
    "localization.");
DEFINE_string(sensor_calibration_file, "", "Path to sensor calibration yaml.");

DEFINE_string(
    external_imu_parameters_rovio, "",
    "Optional, path to the IMU configuration yaml for ROVIO. If none is "
    "provided the maplab values will be used for ROVIO as well.");
DEFINE_string(
    save_map_folder, "", "Save map to folder; if empty nothing is saved.");
DEFINE_bool(
    overwrite_existing_map, false,
    "If set to true, an existing map will be overwritten on save. Otherwise, a "
    "number will be appended to save_map_folder to obtain an available "
    "folder.");
DEFINE_bool(
    optimize_map_to_localization_map, false,
    "Optimize and process the map into a localization map before "
    "saving it.");
DECLARE_double(rovioli_image_resize_factor);

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "rovioli");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  // Load sensors.
  CHECK(!FLAGS_sensor_calibration_file.empty())
      << "[ROVIOLI] No sensor calibration file was provided!";
  vi_map::SensorManager sensor_manager;
  if (!sensor_manager.deserializeFromFile(FLAGS_sensor_calibration_file)) {
    LOG(FATAL) << "[ROVIOLI] Failed to read the sensor calibration from '"
               << FLAGS_sensor_calibration_file << "'!";
  }
  CHECK(vi_map::getSelectedNCamera(sensor_manager))
      << "[ROVIOLI] The sensor calibration does not contain a NCamera!";
  CHECK(vi_map::getSelectedImu(sensor_manager))
      << "[ROVIOLI] The sensor calibration does not contain an IMU!";

  if (fabs(FLAGS_rovioli_image_resize_factor - 1.0) > 1e-6) {
    aslam::NCamera::Ptr ncamera = vi_map::getSelectedNCamera(sensor_manager);
    for (size_t i = 0; i < ncamera->getNumCameras(); i++) {
      // The intrinsics of the camera can just be multiplied with the resize
      // factor. Distortion parameters are agnostic to the image size
      aslam::Camera::Ptr camera = ncamera->getCameraShared(i);
      camera->setParameters(
          camera->getParameters() * FLAGS_rovioli_image_resize_factor);
      camera->setImageWidth(
          round(camera->imageWidth() * FLAGS_rovioli_image_resize_factor));
      camera->setImageHeight(
          round(camera->imageHeight() * FLAGS_rovioli_image_resize_factor));
      camera->setDescription(
          camera->getDescription() + " - resized " +
          std::to_string(FLAGS_rovioli_image_resize_factor));

      // The parameters have changed so we need to generate a new sensor id
      aslam::SensorId camera_id;
      generateId(&camera_id);
      camera->setId(camera_id);
    }
  }

  // Optionally load localization map.
  std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map;
  if (!FLAGS_vio_localization_map_folder.empty()) {
    localization_map.reset(new summary_map::LocalizationSummaryMap);
    if (!localization_map->loadFromFolder(FLAGS_vio_localization_map_folder)) {
      LOG(WARNING)
          << "[ROVIOLI] Could not load a localization summary map from "
          << FLAGS_vio_localization_map_folder
          << ". Will try to load it as a full VI map.";
      vi_map::VIMap vi_map;
      CHECK(vi_map::serialization::loadMapFromFolder(
          FLAGS_vio_localization_map_folder, &vi_map))
          << "[ROVIOLI] Loading a VI map failed. Either provide a valid "
             "localization map "
          << "or leave the map folder flag empty.";

      localization_map.reset(new summary_map::LocalizationSummaryMap);
      summary_map::createLocalizationSummaryMapForWellConstrainedLandmarks(
          vi_map, localization_map.get());
      // Make sure the localization map is not empty.
      CHECK_GT(localization_map->GLandmarkPosition().cols(), 0);
    }
  }

  // Optionally, load external values for the ROVIO sigmas; otherwise also use
  // the maplab values for ROVIO.
  vi_map::ImuSigmas rovio_imu_sigmas;
  if (!FLAGS_external_imu_parameters_rovio.empty()) {
    CHECK(rovio_imu_sigmas.loadFromYaml(FLAGS_external_imu_parameters_rovio))
        << "[ROVIOLI] Could not load IMU parameters for ROVIO from: \'"
        << FLAGS_external_imu_parameters_rovio << "\'";
    CHECK(rovio_imu_sigmas.isValid());
  } else {
    const vi_map::Imu::Ptr imu = vi_map::getSelectedImu(sensor_manager);
    CHECK(imu) << "[ROVIOLI] No imu was found in the sensor calibration!";
    rovio_imu_sigmas = imu->getImuSigmas();
  }

  // Construct the application.
  ros::AsyncSpinner ros_spinner(common::getNumHardwareThreads());
  std::unique_ptr<message_flow::MessageFlow> flow(
      message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
          common::getNumHardwareThreads()));

  if (FLAGS_map_builder_save_image_as_resources &&
      FLAGS_save_map_folder.empty()) {
    LOG(FATAL) << "If you would like to save the resources, "
               << "please also set a map folder with: --save_map_folder";
  }

  // If a map will be saved (i.e., if the save map folder is not empty), append
  // a number to the name until a name is found that is free.
  std::string save_map_folder = FLAGS_save_map_folder;
  if (!FLAGS_save_map_folder.empty()) {
    size_t counter = 0u;
    while (common::fileExists(save_map_folder) ||
           (!FLAGS_overwrite_existing_map &&
            common::pathExists(save_map_folder))) {
      save_map_folder = FLAGS_save_map_folder + "_" + std::to_string(counter++);
    }
  }

  rovioli::RovioliNode rovio_localization_node(
      sensor_manager, rovio_imu_sigmas, save_map_folder, localization_map.get(),
      flow.get());

  // Start the pipeline. The ROS spinner will handle SIGINT for us and abort
  // the application on CTRL+C.
  ros_spinner.start();
  rovio_localization_node.start();

  std::atomic<bool>& end_of_days_signal_received =
      rovio_localization_node.isDataSourceExhausted();
  while (ros::ok() && !end_of_days_signal_received.load()) {
    VLOG_EVERY_N(1, 10) << "\n" << flow->printDeliveryQueueStatistics();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  rovio_localization_node.shutdown();
  flow->shutdown();
  flow->waitUntilIdle();

  if (!save_map_folder.empty()) {
    rovio_localization_node.saveMapAndOptionallyOptimize(
        save_map_folder, FLAGS_overwrite_existing_map,
        FLAGS_optimize_map_to_localization_map);
  }
  return 0;
}
