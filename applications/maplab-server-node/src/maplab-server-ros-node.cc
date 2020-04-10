#include "maplab-server-node/maplab-server-ros-node.h"

#include <atomic>
#include <memory>
#include <signal.h>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <map-resources/resource-common.h>
#include <map-resources/resource-conversion.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <maplab_msgs/MapLookupRequest.h>
#include <maplab_msgs/MapLookupResponse.h>
#include <resources-common/point-cloud.h>

#include "maplab-server-node/maplab-server-node.h"

DEFINE_int32(
    maplab_server_map_update_topic_queue_size, 100, "Size of ROS subscriber.");

DEFINE_string(
    maplab_server_map_update_topic, "map_update_notification",
    "Topic on which the map update notification message is received, it "
    "contains the robot name and the map folder of the new map update.");

namespace maplab {

MaplabServerRosNode::MaplabServerRosNode()
    : maplab_spinner_(common::getNumHardwareThreads()) {
  LOG(INFO) << "[MaplabServerRosNode] Initializing MaplabServerNode...";
  maplab_server_node_.reset(new MaplabServerNode);
}

MaplabServerRosNode::MaplabServerRosNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      maplab_spinner_(common::getNumHardwareThreads()) {
  // === MAPLAB SERVER NODE ===
  LOG(INFO) << "[MaplabServerRosNode] Initializing MaplabServerNode...";
  maplab_server_node_.reset(new MaplabServerNode);

  boost::function<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)>
      save_map_callback =
          boost::bind(&MaplabServerRosNode::saveMapCallback, this, _1, _2);
  save_map_srv_ = nh_.advertiseService("save_map", save_map_callback);

  boost::function<bool(
      maplab_msgs::BatchMapLookup::Request&,
      maplab_msgs::BatchMapLookup::Response&)>
      map_lookup_callback =
          boost::bind(&MaplabServerRosNode::mapLookupCallback, this, _1, _2);
  map_lookup_srv_ = nh_.advertiseService("map_lookup", map_lookup_callback);

  boost::function<bool(
      maplab_msgs::DeleteMission::Request&,
      maplab_msgs::DeleteMission::Response&)>
      delete_mission_callback = boost::bind(
          &MaplabServerRosNode::deleteMissionCallback, this, _1, _2);
  delete_mission_srv_ =
      nh_.advertiseService("delete_mission", delete_mission_callback);

  boost::function<bool(
      maplab_msgs::DeleteAllRobotMissions::Request&,
      maplab_msgs::DeleteAllRobotMissions::Response&)>
      delete_all_robot_missions_callback = boost::bind(
          &MaplabServerRosNode::deleteAllRobotMissionsCallback, this, _1, _2);
  delete_all_robot_missions_srv_ = nh_.advertiseService(
      "delete_all_robot_missions", delete_all_robot_missions_callback);

  boost::function<bool(
      maplab_msgs::GetDenseMapInRange::Request&,
      maplab_msgs::GetDenseMapInRange::Response&)>
      get_dense_map_in_range_callback = boost::bind(
          &MaplabServerRosNode::getDenseMapInRangeCallback, this, _1, _2);
  get_dense_map_in_range_srv_ = nh_.advertiseService(
      "get_dense_map_in_range", get_dense_map_in_range_callback);

  boost::function<void(const diagnostic_msgs::KeyValueConstPtr&)>
      submap_loading_callback =
          boost::bind(&MaplabServerRosNode::submapLoadingCallback, this, _1);
  map_update_notification_sub_ = nh_.subscribe(
      FLAGS_maplab_server_map_update_topic,
      FLAGS_maplab_server_map_update_topic_queue_size, submap_loading_callback);

  T_G_curr_B_curr_pub_ =
      nh_.advertise<geometry_msgs::TransformStamped>("T_G_curr_B_curr", 1);
  T_G_curr_M_curr_pub_ =
      nh_.advertise<geometry_msgs::TransformStamped>("T_G_curr_M_curr", 1);

  T_G_in_B_in_pub_ =
      nh_.advertise<geometry_msgs::TransformStamped>("T_G_in_B_in", 1);
  T_G_in_M_in_pub_ =
      nh_.advertise<geometry_msgs::TransformStamped>("T_G_in_M_in", 1);

  T_G_curr_M_in_pub_ =
      nh_.advertise<geometry_msgs::TransformStamped>("T_G_curr_M_in", 1);

  status_pub_ = nh_.advertise<std_msgs::String>("status", 1);

  dense_map_query_result_ =
      nh_.advertise<sensor_msgs::PointCloud2>("dense_map_query_result", 1);

  maplab_server_node_->registerStatusCallback(
      [this](const std::string status_string) {
        std_msgs::String msg;
        msg.data = status_string;
        status_pub_.publish(msg);
      });

  maplab_server_node_->registerPoseCorrectionPublisherCallback(
      [this](
          const int64_t timestamp_ns, const std::string& robot_name,
          const aslam::Transformation& T_G_curr_B_curr,
          const aslam::Transformation& T_G_curr_M_curr,
          const aslam::Transformation& T_G_in_B_in,
          const aslam::Transformation& T_G_in_M_in) {
        publishPoseCorrection(
            timestamp_ns, robot_name, T_G_curr_B_curr, T_G_curr_M_curr,
            T_G_in_B_in, T_G_in_M_in);
      });
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
    const diagnostic_msgs::KeyValueConstPtr& msg) {
  CHECK(msg);
  const std::string robot_name = msg->key;
  std::string map_path = msg->value;
  common::simplifyPath(&map_path);

  if (!common::pathExists(map_path)) {
    LOG(ERROR) << "[MaplabServerRosNode] Received map notification for robot '"
               << robot_name << "' and local map folder '" << map_path
               << "', but the folder does not exist!";
    return;
  }

  maplab_server_node_->loadAndProcessSubmap(robot_name, map_path);
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

bool MaplabServerRosNode::publishPoseCorrection(
    const int64_t timestamp_ns, const std::string& robot_name,
    const aslam::Transformation& T_G_curr_B_curr,
    const aslam::Transformation& T_G_curr_M_curr,
    const aslam::Transformation& T_G_in_B_in,
    const aslam::Transformation& T_G_in_M_in) const {
  ros::Time timestamp_ros;
  timestamp_ros.fromNSec(timestamp_ns);

  geometry_msgs::TransformStamped T_G_curr_B_curr_msg;
  T_G_curr_B_curr_msg.child_frame_id = robot_name;
  T_G_curr_B_curr_msg.header.stamp = timestamp_ros;
  T_G_curr_B_curr_msg.header.frame_id = FLAGS_tf_map_frame;
  tf::transformKindrToMsg(T_G_curr_B_curr, &T_G_curr_B_curr_msg.transform);
  T_G_curr_B_curr_pub_.publish(T_G_curr_B_curr_msg);

  geometry_msgs::TransformStamped T_G_curr_M_curr_msgs;
  T_G_curr_M_curr_msgs.child_frame_id = robot_name;
  T_G_curr_M_curr_msgs.header.stamp = timestamp_ros;
  T_G_curr_M_curr_msgs.header.frame_id = FLAGS_tf_map_frame;
  tf::transformKindrToMsg(T_G_curr_M_curr, &T_G_curr_M_curr_msgs.transform);
  T_G_curr_M_curr_pub_.publish(T_G_curr_M_curr_msgs);

  geometry_msgs::TransformStamped T_G_in_B_in_msgs;
  T_G_in_B_in_msgs.child_frame_id = robot_name;
  T_G_in_B_in_msgs.header.stamp = timestamp_ros;
  T_G_in_B_in_msgs.header.frame_id = FLAGS_tf_map_frame;
  tf::transformKindrToMsg(T_G_in_B_in, &T_G_in_B_in_msgs.transform);
  T_G_in_B_in_pub_.publish(T_G_in_B_in_msgs);

  geometry_msgs::TransformStamped T_G_in_M_in_msgs;
  T_G_in_M_in_msgs.child_frame_id = robot_name;
  T_G_in_M_in_msgs.header.stamp = timestamp_ros;
  T_G_in_M_in_msgs.header.frame_id = FLAGS_tf_map_frame;
  tf::transformKindrToMsg(T_G_in_M_in, &T_G_in_M_in_msgs.transform);
  T_G_in_M_in_pub_.publish(T_G_in_M_in_msgs);

  const aslam::Transformation T_B_in_M_in = T_G_in_B_in.inverse() * T_G_in_M_in;
  const aslam::Transformation T_G_curr_M_in = T_G_curr_B_curr * T_B_in_M_in;

  geometry_msgs::TransformStamped T_G_curr_M_in_msgs;
  T_G_curr_M_in_msgs.child_frame_id = robot_name;
  T_G_curr_M_in_msgs.header.stamp = timestamp_ros;
  T_G_curr_M_in_msgs.header.frame_id = FLAGS_tf_map_frame;
  tf::transformKindrToMsg(T_G_curr_M_in, &T_G_curr_M_in_msgs.transform);
  T_G_curr_M_in_pub_.publish(T_G_curr_M_in_msgs);
  return true;
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

bool MaplabServerRosNode::deleteMissionCallback(
    maplab_msgs::DeleteMission::Request& request,      // NOLINT
    maplab_msgs::DeleteMission::Response& response) {  // NOLINT
  const std::string& partial_mission_id_string =
      request.mission_id_to_delete.data;

  response.success.data = maplab_server_node_->deleteMission(
      partial_mission_id_string, &response.message.data);

  return true;
}

bool MaplabServerRosNode::deleteAllRobotMissionsCallback(
    maplab_msgs::DeleteAllRobotMissions::Request& request,      // NOLINT
    maplab_msgs::DeleteAllRobotMissions::Response& response) {  // NOLINT

  const std::string& robot_name = request.robot_name.data;

  response.success.data = maplab_server_node_->deleteAllRobotMissions(
      robot_name, &response.message.data);

  return true;
}

bool MaplabServerRosNode::getDenseMapInRangeCallback(
    maplab_msgs::GetDenseMapInRange::Request& request,      // NOLINT
    maplab_msgs::GetDenseMapInRange::Response& response) {  // NOLINT
  Eigen::Vector3d center_G;
  center_G << request.center_G.x, request.center_G.y, request.center_G.z;
  const double radius_m = request.radius_m;

  if (radius_m < 1e-6) {
    LOG(ERROR) << "[MaplabServerRosNode] Received a request from robot '"
               << request.robot_name
               << "' for the dense map with invalid radius: " << radius_m;
    return true;
  }

  const uint32_t point_cloud_type = request.point_cloud_type;
  backend::ResourceType resource_type;
  switch (point_cloud_type) {
    case maplab_msgs::GetDenseMapInRange::Request::POINT_CLOUD_TYPE_XYZI:
      resource_type = backend::ResourceType::kPointCloudXYZI;
      break;
    case maplab_msgs::GetDenseMapInRange::Request::POINT_CLOUD_TYPE_XYZL:
      resource_type = backend::ResourceType::kPointCloudXYZL;
      break;
    default:
      LOG(ERROR) << "[MaplabServerRosNode] Received a request from robot '"
                 << request.robot_name
                 << "' for the dense map with invalid type: "
                 << point_cloud_type;
      return true;
  }

  LOG(INFO) << "[MaplabServerRosNode] Received a request from robot '"
            << request.robot_name << "' for the dense map of type '"
            << backend::ResourceTypeNames[static_cast<int>(resource_type)]
            << "' in a radius of " << radius_m << "m around "
            << center_G.transpose() << ".";

  resources::PointCloud point_cloud_G;
  maplab_server_node_->getDenseMapInRange(
      resource_type, center_G, radius_m, &point_cloud_G);
  backend::convertPointCloudType(point_cloud_G, &response.point_cloud_G);

  // NOTE(mfehr): remove this if this is too expensive.
  dense_map_query_result_.publish(response.point_cloud_G);

  return true;
}

void MaplabServerRosNode::visualizeMap() {
  LOG(INFO) << "[MaplabServerRosNode] Visualizing merged map.";
  maplab_server_node_->visualizeMap();
}

}  // namespace maplab
