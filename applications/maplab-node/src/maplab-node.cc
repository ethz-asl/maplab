#include <aslam/cameras/ncamera.h>
#include <atomic>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <memory>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <message-flow/message-topic-registration.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <signal.h>
#include <string>
#include <vi-map/sensor-utils.h>
#include <vi-map/vi-map-serialization.h>
#include <vio-common/vio-types.h>

#include "maplab-node/data-publisher-flow.h"
#include "maplab-node/datasource-flow.h"
#include "maplab-node/feature-tracking-flow.h"
#include "maplab-node/maplab-node.h"
#include "maplab-node/synchronizer-flow.h"

DECLARE_double(image_resize_factor);

namespace maplab {
MaplabNode::MaplabNode(
    const std::string& sensor_calibration_file,
    const std::string& save_map_folder, message_flow::MessageFlow* const flow)
    : message_flow_(CHECK_NOTNULL(flow)),
      is_datasource_exhausted_(false),
      is_running_(false),
      sensor_manager_(new vi_map::SensorManager()) {
  CHECK(
      !sensor_calibration_file.empty() &&
      common::fileExists(sensor_calibration_file))
      << "[MaplabNode] Provide a valid calibration file with the parameter "
         "'sensor_calibration_file'! file: '"
      << sensor_calibration_file << "'";

  // === SENSORS ===
  if (!sensor_manager_->deserializeFromFile(sensor_calibration_file)) {
    LOG(FATAL) << "[MaplabNode] Failed to read the sensor calibration from '"
               << sensor_calibration_file << "'!";
  }
  CHECK(sensor_manager_);

  // At the very beginning check if the main ncamera images need to be resized.
  aslam::NCamera::Ptr mapping_ncamera =
      vi_map::getSelectedNCamera(*sensor_manager_);
  if (mapping_ncamera && fabs(FLAGS_image_resize_factor - 1.0) > 1e-6) {
    for (size_t i = 0; i < mapping_ncamera->getNumCameras(); i++) {
      // The intrinsics of the camera can just be multiplied with the resize
      // factor. Distortion parameters are agnostic to the image size.
      aslam::Camera::Ptr camera = mapping_ncamera->getCameraShared(i);
      camera->setParameters(
          camera->getParameters() * FLAGS_image_resize_factor);
      camera->setImageWidth(
          round(camera->imageWidth() * FLAGS_image_resize_factor));
      camera->setImageHeight(
          round(camera->imageHeight() * FLAGS_image_resize_factor));
      camera->setDescription(
          camera->getDescription() + " - resized " +
          std::to_string(FLAGS_image_resize_factor));

      // The parameters have changed so we need to generate a new sensor id.
      aslam::SensorId camera_id;
      generateId(&camera_id);
      camera->setId(camera_id);
    }
  }

  // Check and associate external feature sensors to cameras in the main ncamera
  aslam::SensorIdSet all_external_feature_sensor_ids;
  sensor_manager_->getAllSensorIdsOfType(
      vi_map::SensorType::kExternalFeatures, &all_external_feature_sensor_ids);
  if (all_external_feature_sensor_ids.size() > 0) {
    CHECK(mapping_ncamera) << "[MaplabNode] External features require a "
                              "NCamera to add the features to.";

    for (const aslam::SensorId sensor_id : all_external_feature_sensor_ids) {
      vi_map::ExternalFeatures::Ptr external_features_sensor =
          sensor_manager_->getSensorPtr<vi_map::ExternalFeatures>(sensor_id);

      bool camera_exists = false;
      for (size_t i = 0; i < mapping_ncamera->getNumCameras(); i++) {
        if (mapping_ncamera->getCamera(i).getId() ==
            external_features_sensor->getTargetSensorId()) {
          camera_exists = true;
          external_features_sensor->setTargetNCameraId(
              mapping_ncamera->getId());
          external_features_sensor->setTargetCameraIndex(i);
          break;
        }
      }

      CHECK(camera_exists)
          << "[MaplabNode] External features enabled, but target sensor "
          << external_features_sensor->getTargetSensorId() << " not found "
          << "for the selected NCamera " << mapping_ncamera->getId() << ".";
    }
  }

  // === SYNCHRONIZER ===
  synchronizer_flow_.reset(new SynchronizerFlow(*sensor_manager_));
  synchronizer_flow_->attachToMessageFlow(message_flow_);
  // Subscribe to end of days signal from the synchronizer.
  synchronizer_flow_->registerEndOfDataCallback(
      [&]() { is_datasource_exhausted_.store(true); });

  // === DATA SOURCE ===
  datasource_flow_.reset(new DataSourceFlow(*sensor_manager_));
  datasource_flow_->attachToMessageFlow(message_flow_);
  // Subscribe to end of days signal from the datasource.
  datasource_flow_->registerEndOfDataCallback(
      [&]() { is_datasource_exhausted_.store(true); });

  // === DATA PUBLISHER ====
  data_publisher_flow_.reset(new DataPublisherFlow(*sensor_manager_));
  data_publisher_flow_->attachToMessageFlow(message_flow_);

  // === MAP BUILDER ====
  map_builder_flow_.reset(new MapBuilderFlow(
      *sensor_manager_, save_map_folder, synchronizer_flow_->T_M_B_buffer()));
  map_builder_flow_->attachToMessageFlow(message_flow_);

  // === ENABLE MAPPING COMPONENTS ===
  initializeOdometrySource();
  initializeInertialMapping();
  initializeLidarMapping();
  initializeVisualMapping();
  initializeExternalFeatures();
  initializeAbsolute6DoFSource();
  initializeWheelOdometrySource();
  initializeLoopClosureSource();
  initializePointCloudMapSource();
}

MaplabNode::~MaplabNode() {
  shutdown();
}

void MaplabNode::initializeOdometrySource() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::Odometry6DoF::Ptr external_odometry_sensor =
      vi_map::getSelectedOdometry6DoFSensor(*sensor_manager_);
  if (external_odometry_sensor) {
    LOG(INFO) << "[MaplabNode] External 6DoF Odometry sensor is ENABLED!";
    synchronizer_flow_->initializeOdometryData();
  } else {
    LOG(FATAL)
        << "[MaplabNode] This node currently relies on an external "
        << "odometry source and an IMU (for interpolation and forward "
        << "propagation) to build the map! Please ensure that all "
        << "sensors (including odometry) are registered in the yaml file.";
  }
}

void MaplabNode::initializeInertialMapping() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::Imu::Ptr mapping_imu = vi_map::getSelectedImu(*sensor_manager_);
  if (mapping_imu) {
    synchronizer_flow_->initializeInertialData();
    LOG(INFO) << "[MaplabNode] Inertial sensor is ENABLED!";
  } else {
    LOG(FATAL)
        << "[MaplabNode] This node currently relies on an external "
        << "odometry source and an IMU (for interpolation and forward "
        << "propagation) to build the map! Please ensure that all "
        << "sensors (including odometry) are registered in the yaml file!";
  }
}

void MaplabNode::initializeVisualMapping() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  aslam::NCamera::Ptr mapping_ncamera =
      vi_map::getSelectedNCamera(*sensor_manager_);
  if (mapping_ncamera) {
    synchronizer_flow_->initializeVisualData(mapping_ncamera);
    tracker_flow_.reset(new FeatureTrackingFlow(
        mapping_ncamera, synchronizer_flow_->T_M_B_buffer()));
    tracker_flow_->attachToMessageFlow(message_flow_);
    LOG(INFO) << "[MaplabNode] Visual-inertial mapping is ENABLED!";
  } else {
    LOG(WARNING) << "[MaplabNode] Visual-inertial mapping is DISABLED!";
  }
}

void MaplabNode::initializeExternalFeatures() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  aslam::SensorIdSet all_external_feature_sensor_ids;
  sensor_manager_->getAllSensorIdsOfType(
      vi_map::SensorType::kExternalFeatures, &all_external_feature_sensor_ids);

  if (all_external_feature_sensor_ids.size() > 0) {
    aslam::NCamera::Ptr mapping_ncamera =
        vi_map::getSelectedNCamera(*sensor_manager_);
    CHECK(mapping_ncamera) << "[MaplabNode] External features require a "
                              "NCamera to add the features to.";

    for (const aslam::SensorId sensor_id : all_external_feature_sensor_ids) {
      vi_map::ExternalFeatures::Ptr external_features_sensor =
          sensor_manager_->getSensorPtr<vi_map::ExternalFeatures>(sensor_id);

      // tracker_flow_.reset(new FeatureTrackingFlow(
      //    mapping_ncamera, synchronizer_flow_->T_M_B_buffer()));
      // tracker_flow_->attachToMessageFlow(message_flow_);

      LOG(INFO) << "[MaplabNode] External features ENABLED for sensor "
                << external_features_sensor->getTargetSensorId();
    }

    synchronizer_flow_->initializeExternalFeaturesData(
        all_external_feature_sensor_ids);
  }
}

void MaplabNode::initializeLidarMapping() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::Lidar::Ptr mapping_lidar = vi_map::getSelectedLidar(*sensor_manager_);
  if (mapping_lidar) {
    synchronizer_flow_->initializeLidarData();

    // TODO(LBern): initialize lidar mapping blocks here, rm imu if no imu
    // constraints are needed.

    // TODO(LBern): Currently only one lidar is supported, but there is no
    // good reason not to support N-Lidars.

    LOG(INFO) << "[MaplabNode] Lidar-inertial mapping is ENABLED!";
  } else {
    LOG(WARNING) << "[MaplabNode] Lidar-inertial mapping is DISABLED!";
  }
}

void MaplabNode::initializeAbsolute6DoFSource() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::Absolute6DoF::Ptr absolute_6dof_sensor =
      vi_map::getSelectedAbsolute6DoFSensor(*sensor_manager_);
  if (absolute_6dof_sensor) {
    synchronizer_flow_->initializeAbsolute6DoFData();

    LOG(INFO) << "[MaplabNode] External absolute 6DoF pose sensor is ENABLED!";
  } else {
    LOG(WARNING)
        << "[MaplabNode] External absolute 6DoF pose sensor is DISABLED!";
  }
}

void MaplabNode::initializeLoopClosureSource() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::LoopClosureSensor::Ptr loop_closure_sensor =
      vi_map::getSelectedLoopClosureSensor(*sensor_manager_);
  if (loop_closure_sensor) {
    synchronizer_flow_->initializeLoopClosureData();

    LOG(INFO) << "[MaplabNode] External loop closure pose sensor is ENABLED!";
  } else {
    LOG(WARNING)
        << "[MaplabNode] External loop closure pose sensor is DISABLED!";
  }
}

void MaplabNode::initializeWheelOdometrySource() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::WheelOdometry::Ptr wheel_odometry_sensor =
      vi_map::getSelectedWheelOdometrySensor(*sensor_manager_);
  if (wheel_odometry_sensor) {
    synchronizer_flow_->initializeWheelOdometryData();

    LOG(INFO) << "[MaplabNode] External wheel odometry sensor is ENABLED!";
  } else {
    LOG(WARNING) << "[MaplabNode] External wheel odometry sensor is DISABLED!";
  }
}

void MaplabNode::initializePointCloudMapSource() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::PointCloudMapSensor::Ptr pointcloud_map_sensor =
      vi_map::getSelectedPointCloudMapSensor(*sensor_manager_);
  if (pointcloud_map_sensor) {
    synchronizer_flow_->initializePointCloudMapData();

    LOG(INFO) << "[MaplabNode] External point cloud map source is ENABLED!";
  } else {
    LOG(WARNING) << "[MaplabNode] External point cloud map source is DISABLED!";
  }
}

void MaplabNode::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Starting...";

  CHECK(synchronizer_flow_);
  synchronizer_flow_->start();

  CHECK(!is_datasource_exhausted_.load())
      << "Cannot start the MaplabNode after the "
      << "end-of-data signal was received!";

  CHECK(datasource_flow_);
  datasource_flow_->startStreaming();
  VLOG(1) << "Starting data source...";

  is_running_ = true;
}

void MaplabNode::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Shutting down...";
  datasource_flow_->shutdown();
  VLOG(1) << "Closing data source...";
  is_running_ = false;
}

bool MaplabNode::saveMapAndOptionallyOptimize(
    const std::string& path, const bool overwrite_existing_map,
    const bool process_to_localization_map, const bool stop_mapping) {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Saving map to '" << path << "'.";

  if (map_builder_flow_) {
    return map_builder_flow_->saveMapAndOptionallyOptimize(
        path, overwrite_existing_map, process_to_localization_map,
        stop_mapping);
  } else {
    LOG(ERROR) << "Cannot save map, because map building has been disabled "
               << "using --run_map_builder=false";
  }
  return false;
}

bool MaplabNode::isDataSourceExhausted() {
  return is_datasource_exhausted_.load();
}

}  // namespace maplab
