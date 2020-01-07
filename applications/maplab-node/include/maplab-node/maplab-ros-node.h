#ifndef MAPLAB_NODE_MAPLAB_ROS_NODE_H_
#define MAPLAB_NODE_MAPLAB_ROS_NODE_H_

#include <atomic>
#include <memory>
#include <string>

#include <localization-summary-map/localization-summary-map.h>
#include <message-flow/message-flow.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "maplab-node/maplab-node.h"

namespace maplab {

class MaplabRosNode {
 public:
  // Initialize this node, need to be executed before run().
  //  - Load configuration from ROS parameters.
  //  - Loads localization maps and calibrations.
  //  - Creates and connect nodes in the message flow based on the
  //    configuration.
  MaplabRosNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Start the app.
  bool run();

  // Save current map.
  bool saveMap(const std::string& map_folder, const bool stop_mapping);

  // Save current map to the folder specified in the parameters and finish
  // mapping (stop adding data to the map).
  bool saveMapAndFinishMapping();
  // Save current map to the folder specified in the parameters and continue
  // mapping.
  bool saveMapAndContinueMapping();

  std::string getMapFolder() const;

  // Check if the app *should* to be stopped (i.e., finished processing
  // bag).
  bool isDataSourceExhausted();

  void shutdown();

  bool saveMapCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  // Optional output.
  std::string printDeliveryQueueStatistics() const;

 private:
  // ROS stuff.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer save_map_srv_;

  // Settings.
  std::string map_output_folder_;
  size_t map_counter_;

  // State for running for maplab.
  ros::AsyncSpinner maplab_spinner_;

  // Message flow is used to connect all the nodes in a publisher-subscriber
  // pattern.
  std::unique_ptr<message_flow::MessageFlow> message_flow_;

  // One node to rule them all.
  std::unique_ptr<MaplabNode> maplab_node_;

  // Publisher that sends the path of the map ever time it is saved.
  ros::Publisher map_update_pub_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_MAPLAB_ROS_NODE_H_
