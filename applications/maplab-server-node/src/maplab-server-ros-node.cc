#include "maplab-server-node/maplab-server-ros-node.h"

#include <atomic>
#include <memory>
#include <signal.h>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <maplab-common/file-system-tools.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <maplab_msgs/MapLookupRequest.h>
#include <maplab_msgs/MapLookupResponse.h>

#include <std_srvs/Empty.h>

#include "maplab-server-node/maplab-server-config.h"
#include "maplab-server-node/maplab-server-node.h"

DEFINE_string(
    server_node_config_file, "",
    "Path the the config YAML file for the maplab server node.");

DEFINE_int32(
    map_update_notification_subscriber_queue_size, 100,
    "Size of ROS subscriber.");

namespace maplab {

MaplabServerRosNode::MaplabServerRosNode(
    const MaplabServerRosNodeConfig& config)
    : config_(config), maplab_spinner_(common::getNumHardwareThreads()) {
  LOG(INFO) << "[MaplabServerRosNode] Initializing MaplabServerNode...";
  maplab_server_node_.reset(new MaplabServerNode(config_.server_config));
}

MaplabServerRosNode::MaplabServerRosNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : config_(),
      nh_(nh),
      nh_private_(nh_private),
      maplab_spinner_(common::getNumHardwareThreads()) {
  CHECK(config_.deserializeFromFile(FLAGS_server_node_config_file))
      << "[MaplabServerRosNode] Failed to parse config from '"
      << FLAGS_server_node_config_file << "'";

  // === MAPLAB SERVER NODE ===
  LOG(INFO) << "[MaplabServerRosNode] Initializing MaplabServerNode...";
  maplab_server_node_.reset(new MaplabServerNode(config_.server_config));

  // Set up map saving service.
  boost::function<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)>
      save_map_callback =
          boost::bind(&MaplabServerRosNode::saveMapCallback, this, _1, _2);
  save_map_srv_ = nh_.advertiseService("save_map", save_map_callback);

  // Set up map saving service.
  boost::function<bool(
      maplab_msgs::BatchMapLookup::Request&,
      maplab_msgs::BatchMapLookup::Response&)>
      map_lookup_callback =
          boost::bind(&MaplabServerRosNode::mapLookupCallback, this, _1, _2);
  map_lookup_srv_ = nh_.advertiseService("map_lookup", map_lookup_callback);

  // Set up notification subscribers to transfer maps to the server machine and
  // merge them.
  for (const RobotConnectionConfig& robot_config : config_.connection_config) {
    boost::function<void(const std_msgs::StringConstPtr&)>
        submap_loading_callback = boost::bind(
            &MaplabServerRosNode::submapLoadingCallback, this, _1,
            robot_config);

    ros::Subscriber submap_loading_sub = nh_.subscribe(
        robot_config.topic, FLAGS_map_update_notification_subscriber_queue_size,
        submap_loading_callback);
    map_update_notification_subs_.push_back(submap_loading_sub);
  }
}

bool MaplabServerRosNode::start() {
  LOG(INFO) << "[MaplabServerRosNode] Starting...";
  // Start the pipeline. The ROS spinner will handle SIGINT for us and abort
  // the application on CTRL+C.
  maplab_spinner_.start();
  maplab_server_node_->start();
  return true;
}

void MaplabServerRosNode::submapLoadingCallback(
    const std_msgs::StringConstPtr& msg,
    const RobotConnectionConfig& robot_config) {
  CHECK(msg);

  std::string remote_map_path = msg->data;
  common::simplifyPath(&remote_map_path);

  if (robot_config.ip == "localhost") {
    CHECK(common::pathExists(remote_map_path))
        << "[MaplabServerRosNode] Path to submap '" << remote_map_path
        << "' does not exist despite the ip indicating the map is on the "
        << "localhost!";

    maplab_server_node_->loadAndProcessSubmap(
        robot_config.name, remote_map_path);
  } else {
    std::string map_folder_name, tmp;
    common::splitPathByLastOccurenceOf(
        remote_map_path, "/", false /*left first*/, &tmp, &map_folder_name);

    if (map_folder_name.empty()) {
      LOG(WARNING) << "[MaplabServerRosNode] Could not extract map folder from "
                   << "remote path '"
                   << "', using unique time-date string instead!";
      map_folder_name = "remote_submap_received_at_" +
                        common::generateDateStringFromCurrentTime();
    }

    const std::string local_map_path =
        "/tmp/maplab_server_node/" + map_folder_name;

    std::stringstream command_ss;
    command_ss << "rsync -r " << robot_config.user << "@" << robot_config.ip
               << ":" << remote_map_path << " " << local_map_path
               << " --progress";
    const std::string command_string = command_ss.str();
    const char* command = command_string.c_str();

    LOG(INFO) << "[MaplabServerRosNode] transfering remote map with command: '"
              << command_string << "'...";
    const int result = system(command);
    if (result == 0) {
      LOG(INFO) << "[MaplabServerRosNode] transfer complete.";
    } else {
      LOG(ERROR) << "[MaplabServerRosNode] transfer failed!.";
    }
    CHECK(common::pathExists(local_map_path));

    maplab_server_node_->loadAndProcessSubmap(
        robot_config.name, local_map_path);
  }
}

bool MaplabServerRosNode::saveMap(const std::string& map_folder) {
  CHECK(!map_folder.empty());
  CHECK(maplab_server_node_);
  LOG(INFO) << "[MaplabServerRosNode] Saving map to '" << map_folder << "'.";

  return maplab_server_node_->saveMap(map_folder);
}

bool MaplabServerRosNode::saveMap() {
  CHECK(maplab_server_node_);
  LOG(INFO) << "[MaplabServerRosNode] Saving map...";

  return maplab_server_node_->saveMap();
}

void MaplabServerRosNode::shutdown() {
  LOG(INFO) << "[MaplabServerRosNode] Shutting down...";
  maplab_server_node_->shutdown();
}

// Save map over ROS service, in case save_map_on_shutdown is disabled.
bool MaplabServerRosNode::saveMapCallback(
    std_srvs::Empty::Request& /*request*/,      // NOLINT
    std_srvs::Empty::Response& /*response*/) {  // NOLINT
  LOG(INFO) << "[MaplabServerRosNode] Received save map service call...";
  return saveMap();
}

// Look up the current global frame position of a point in sensor frame.
bool MaplabServerRosNode::mapLookupCallback(
    maplab_msgs::BatchMapLookup::Request& requests,      // NOLINT
    maplab_msgs::BatchMapLookup::Response& responses) {  // NOLINT
  for (const maplab_msgs::MapLookupRequest& request : requests.map_lookups) {
    const int64_t timestamp_ns = request.timestamp.toNSec();
    const Eigen::Vector3d p_S = {request.p_S.x, request.p_S.y, request.p_S.z};
    const std::string robot_name = request.robot_name;

    LOG(INFO) << "[MaplabServerRosNode] Received map lookup service call for "
              << "sensor frame " << request.sensor_type << " of robot "
              << request.robot_name << " at timestamp " << timestamp_ns << "ns";

    const vi_map::SensorType sensor_type =
        vi_map::convertStringToSensorType(request.sensor_type);

    Eigen::Vector3d p_G, sensor_p_G;
    maplab_msgs::MapLookupResponse response;
    response.status = static_cast<int>(maplab_server_node_->mapLookup(
        robot_name, sensor_type, timestamp_ns, p_S, &p_G, &sensor_p_G));
    response.p_G.x = p_G.x();
    response.p_G.y = p_G.y();
    response.p_G.z = p_G.z();

    response.sensor_p_G.x = sensor_p_G.x();
    response.sensor_p_G.y = sensor_p_G.y();
    response.sensor_p_G.z = sensor_p_G.z();

    responses.map_lookups.emplace_back(std::move(response));
  }

  return true;
}

void MaplabServerRosNode::visualizeMap() {
  LOG(INFO) << "[MaplabServerRosNode] Visualizing merged map.";
  maplab_server_node_->visualizeMap();
}

}  // namespace maplab
