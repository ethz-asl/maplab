#include "maplab-node/maplab-ros-node.h"

#include <atomic>
#include <memory>
#include <signal.h>
#include <string>

#include <diagnostic_msgs/KeyValue.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <aslam/cameras/ncamera.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <message-flow/message-topic-registration.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>

#include <std_srvs/Empty.h>
#include <vi-map/vi-map-serialization.h>
#include <vio-common/vio-types.h>

#include "maplab-node/maplab-node.h"

// === SENSORS ===

DEFINE_string(
    sensor_calibration_file, "",
    "Yaml file with all the sensor calibrations. Determines which sensors are "
    "used by MaplabNode.");

// ==== INPUT LOCALIZATION MAPS ====

DEFINE_string(
    visual_localization_map_folder, "",
    "Path to a visual localization summary map or a full VI-map used for "
    "localization.");

DEFINE_string(
    lidar_localization_map_folder, "", "Path to a lidar localization map.");

// === OUTPUT MAPS ===

DEFINE_string(
    map_output_folder, "", "Save map to folder; if empty nothing is saved.");
DEFINE_bool(
    map_overwrite_enabled, false,
    "If set to true, an existing map will be overwritten on save. Otherwise, a "
    "number will be appended to map_output_folder to obtain an available "
    "folder.");
DEFINE_bool(
    map_add_unique_timestamp_prefix_for_save_map_folder, false,
    "Adds an unique timestamp to the store location of the map folder");
DEFINE_bool(
    map_compute_localization_map, false,
    "Optimize and process the map into a localization map before "
    "saving it.");

// === MAPLAB NODE ===

DEFINE_bool(
    enable_visual_localization, true,
    "The maplab node will use vision to localize, if enabled. Requires "
    "--enable_visual_inertial_data=true");

DEFINE_bool(
    enable_lidar_localization, false,
    "The maplab node will use the lidar data to localize, if enabled. Requires "
    "--enable_lidar_data=true "
    "[NOT IMPLEMENTED YET]");

DEFINE_bool(
    enable_online_mapping, false,
    "The maplab node will try to optimize and loop close the pose graph online "
    "and provide the localization with new maps. [NOT IMPLEMENTED YET]");

DECLARE_bool(map_split_map_into_submaps_when_saving_periodically);
DECLARE_int32(map_save_every_n_sec);

namespace maplab {

MaplabRosNode::MaplabRosNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      map_output_folder_(""),
      map_counter_(0u),
      maplab_spinner_(common::getNumHardwareThreads()) {
  // Set up publishers.
  map_update_pub_ =
      nh_.advertise<std_msgs::String>("map_update_notification", 1);

  submap_counter_pub_ =
      nh_.advertise<diagnostic_msgs::KeyValue>("submap_counter", 1);

  LOG(INFO) << "[MaplabROSNode] Initializing message flow...";
  message_flow_.reset(
      message_flow::MessageFlow::create<message_flow::MessageDispatcherFifo>(
          common::getNumHardwareThreads()));

  // If a map will be saved (i.e., if the save map folder is not empty), append
  // a number to the name until a name is found that is free.
  map_output_folder_ = FLAGS_map_output_folder;
  if (!FLAGS_map_overwrite_enabled && !map_output_folder_.empty()) {
    // Add a unique mission_timestamp prefix for storing the submaps.
    if (FLAGS_map_add_unique_timestamp_prefix_for_save_map_folder) {
      map_output_folder_ +=
          "/mission_" + std::to_string(aslam::time::nanoSecondsSinceEpoch());
    }
    map_output_folder_ = common::getUniqueFolderName(map_output_folder_);
  }
  if (map_output_folder_.empty()) {
    LOG(INFO) << "[MaplabROSNode] No output map folder was provided, map "
              << "building disable.";
  } else {
    LOG(INFO) << "[MaplabROSNode] Set output map folder to: '"
              << map_output_folder_ << "', map building enabled.";
  }

  if (FLAGS_map_builder_save_image_as_resources && map_output_folder_.empty()) {
    LOG(FATAL) << "If you would like to save the resources, "
               << "please also set a map folder with: --map_output_folder";
  }

  // === MAPLAB NODE ===
  LOG(INFO) << "[MaplabROSNode] Initializing MaplabNode...";
  maplab_node_.reset(new MaplabNode(
      FLAGS_sensor_calibration_file, map_output_folder_, message_flow_.get()));

  // === LOCALIZATION MAPS ===
  // === VISION ===
  if (!FLAGS_visual_localization_map_folder.empty() &&
      FLAGS_enable_visual_localization) {
    LOG(INFO) << "[MaplabROSNode] Loading visual localization map from: "
              << FLAGS_visual_localization_map_folder << "'.";

    std::unique_ptr<summary_map::LocalizationSummaryMap> summary_map(
        new summary_map::LocalizationSummaryMap);
    summary_map::loadLocalizationSummaryMapFromAnyMapFile(
        FLAGS_visual_localization_map_folder, summary_map.get());
    maplab_node_->enableVisualLocalization(std::move(summary_map));
    CHECK(!summary_map);
  } else {
    LOG(INFO) << "[MaplabROSNode] No visual localization map provided.";
  }
  // === LIDAR ===
  if (!FLAGS_lidar_localization_map_folder.empty() &&
      FLAGS_enable_lidar_localization) {
    LOG(INFO) << "[MaplabROSNode] Loading lidar localization map from: "
              << FLAGS_lidar_localization_map_folder << "'.";

    // TODO(LBern): load lidar localization map.

    maplab_node_->enableLidarLocalization();
  } else {
    LOG(INFO) << "[MaplabROSNode] No lidar localization map provided.";
  }

  boost::function<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)>
      save_map_callback =
          boost::bind(&MaplabRosNode::saveMapCallback, this, _1, _2);
  save_map_srv_ = nh_.advertiseService("save_map", save_map_callback);

  boost::function<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)>
      go_idle_callback =
          boost::bind(&MaplabRosNode::goIdleCallback, this, _1, _2);
  go_idle_srv_ = nh_.advertiseService("go_idle", go_idle_callback);
}

bool MaplabRosNode::run() {
  LOG(INFO) << "[MaplabROSNode] Starting...";
  // Start the pipeline. The ROS spinner will handle SIGINT for us and abort
  // the application on CTRL+C.
  maplab_spinner_.start();
  maplab_node_->start();
  return true;
}

std::string MaplabRosNode::getMapFolder() const {
  return map_output_folder_;
}

bool MaplabRosNode::saveMap(
    const std::string& map_folder, const bool stop_mapping) {
  LOG(INFO) << "[MaplabROSNode] Saving map...";
  if (!map_folder.empty()) {
    CHECK(maplab_node_);

    const std::string date_time_string =
        common::generateDateStringFromCurrentTime();

    // Augment the map folder with date and time if saving periodically and add
    // the submap number if submapping is enabled.
    std::string map_folder_updated;
    if (FLAGS_map_save_every_n_sec > 0) {
      if (stop_mapping) {
        std::stringstream ss;
        if (FLAGS_map_split_map_into_submaps_when_saving_periodically) {
          ss << map_folder << "/"
             << "submap_" << map_counter_;
        } else {
          ss << map_folder << "/final_map";
        }
        map_folder_updated = ss.str();
      } else {
        std::stringstream ss;
        if (FLAGS_map_split_map_into_submaps_when_saving_periodically) {
          ss << map_folder << "/"
             << "submap_" << map_counter_;
        } else {
          ss << map_folder << "/partial_map_" << map_counter_;
        }
        map_folder_updated = ss.str();
      }
    } else {
      map_folder_updated = map_folder;
    }

    if (maplab_node_->saveMapAndOptionallyOptimize(
            map_folder_updated, FLAGS_map_overwrite_enabled,
            FLAGS_map_compute_localization_map, stop_mapping)) {
      ++map_counter_;
      std_msgs::String map_folder_msg;
      map_folder_msg.data = map_folder_updated;
      map_update_pub_.publish(map_folder_msg);
      diagnostic_msgs::KeyValue submap_counter_msg;
      if (nh_.getNamespace().size() > 1u) {
        submap_counter_msg.key = nh_.getNamespace().substr(1);
      }
      submap_counter_msg.value = std::to_string(map_counter_);
      submap_counter_pub_.publish(submap_counter_msg);

      return true;
    }
  }
  return false;
}

bool MaplabRosNode::saveMapAndFinishMapping() {
  return saveMap(map_output_folder_, true /*stop mapping*/);
}

bool MaplabRosNode::saveMapAndContinueMapping() {
  return saveMap(map_output_folder_, false /*stop mapping*/);
}

bool MaplabRosNode::isDataSourceExhausted() {
  return maplab_node_->isDataSourceExhausted();
}

void MaplabRosNode::shutdown() {
  LOG(INFO) << "[MaplabROSNode] Shutting down...";
  maplab_node_->shutdown();
  message_flow_->shutdown();
  message_flow_->waitUntilIdle();
}

// Save map over ROS service, in case save_map_on_shutdown is disabled.
bool MaplabRosNode::saveMapCallback(
    std_srvs::Empty::Request& /*request*/,      // NOLINT
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  return saveMapAndContinueMapping();
}

bool MaplabRosNode::goIdleCallback(
    std_srvs::Empty::Request& request,      // NOLINT
    std_srvs::Empty::Response& response) {  // NOLINT
  message_flow_->shutdown();
  message_flow_->waitUntilIdle();
  return true;
}

// Optional output.
std::string MaplabRosNode::printDeliveryQueueStatistics() const {
  return message_flow_->printDeliveryQueueStatistics();
}

}  // namespace maplab
