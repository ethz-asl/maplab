#ifndef MAPLAB_SERVER_NODE_MAPLAB_SERVER_ROS_NODE_H_
#define MAPLAB_SERVER_NODE_MAPLAB_SERVER_ROS_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/KeyValue.h>
#include <maplab_msgs/BatchMapLookup.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include "maplab-server-node/maplab-server-node.h"

namespace maplab {

class MaplabServerRosNode {
 public:
  MaplabServerRosNode(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Only for test purposes.
  explicit MaplabServerRosNode(const MaplabServerNodeConfig& config);

  // Start the app.
  bool start();

  void submapLoadingCallback(const diagnostic_msgs::KeyValueConstPtr& msg);

  // Save current map.
  bool saveMap(const std::string& map_folder);
  bool saveMap();

  void shutdown();

  bool saveMapCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  // Look up the current global frame position of a point in sensor frame.
  bool mapLookupCallback(
      maplab_msgs::BatchMapLookup::Request& requests,     // NOLINT
      maplab_msgs::BatchMapLookup::Response& responses);  // NOLINT

  void visualizeMap();

 private:
  // ROS stuff.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer save_map_srv_;
  ros::ServiceServer map_lookup_srv_;

  // State for running for maplab.
  ros::AsyncSpinner maplab_spinner_;

  // One node to rule them all.
  std::unique_ptr<MaplabServerNode> maplab_server_node_;

  ros::Subscriber map_update_notification_sub_;
};

}  // namespace maplab

#endif  // MAPLAB_SERVER_NODE_MAPLAB_SERVER_ROS_NODE_H_
