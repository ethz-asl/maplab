#ifndef MAPLAB_SERVER_NODE_MAPLAB_SERVER_ROS_NODE_H_
#define MAPLAB_SERVER_NODE_MAPLAB_SERVER_ROS_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <maplab_msgs/BatchMapLookup.h>
#include <maplab_msgs/DeleteAllRobotMissions.h>
#include <maplab_msgs/DeleteMission.h>
#include <maplab_msgs/GetDenseMapInRange.h>
#include <maplab_msgs/Verification.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <transfolder_msgs/RobotSubfoldersArray.h>

#include "maplab-server-node/maplab-server-node.h"

namespace maplab {

class MaplabServerRosNode {
 public:
  MaplabServerRosNode(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Only for test purposes.
  MaplabServerRosNode();

  // Start the app.
  bool start();

  void submapLoadingCallback(
      const transfolder_msgs::RobotSubfoldersArrayConstPtr& msg);

  // Save current map.
  bool saveMap(const std::string& map_folder);
  bool saveMap();

  void shutdown();

  bool saveMapCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  bool reinitGflagsCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  bool whitelistAllMissionsCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  bool acceptNewSubmapsCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  bool rejectNewSubmapsCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  bool startOperatingCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  bool stopOperatingCallback(
      std_srvs::Empty::Request& request,     // NOLINT
      std_srvs::Empty::Response& response);  // NOLINT

  // Look up the current global frame position of a point in sensor frame.
  bool mapLookupCallback(
      maplab_msgs::BatchMapLookup::Request& requests,     // NOLINT
      maplab_msgs::BatchMapLookup::Response& responses);  // NOLINT

  bool deleteMissionCallback(
      maplab_msgs::DeleteMission::Request& request,     // NOLINT
      maplab_msgs::DeleteMission::Response& response);  // NOLINT

  bool deleteAllRobotMissionsCallback(
      maplab_msgs::DeleteAllRobotMissions::Request& request,     // NOLINT
      maplab_msgs::DeleteAllRobotMissions::Response& response);  // NOLINT

  bool getDenseMapInRangeCallback(
      maplab_msgs::GetDenseMapInRange::Request& request,     // NOLINT
      maplab_msgs::GetDenseMapInRange::Response& response);  // NOLINT

  bool verificationCallback(
      maplab_msgs::Verification::Request& requests,     // NOLINT
      maplab_msgs::Verification::Response& responses);  // NOLINT

  bool publishPoseCorrection(
      const int64_t timestamp_ns, const std::string& robot_name,
      const aslam::Transformation& T_G_curr_B_curr,
      const aslam::Transformation& T_G_curr_M_curr,
      const aslam::Transformation& T_G_in_B_in,
      const aslam::Transformation& T_G_in_M_in) const;

  void visualizeMap();

  void triggerSparseGraphUpdate(const ros::TimerEvent& event);

 private:
  // ROS stuff.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer save_map_srv_;
  ros::ServiceServer reinit_gflags_srv_;
  ros::ServiceServer whitelist_missions_srv_;
  ros::ServiceServer map_lookup_srv_;
  ros::ServiceServer delete_mission_srv_;
  ros::ServiceServer delete_all_robot_missions_srv_;
  ros::ServiceServer get_dense_map_in_range_srv_;
  ros::ServiceServer verification_srv_;
  ros::ServiceServer accept_new_submaps_srv_;
  ros::ServiceServer reject_new_submaps_srv_;
  ros::ServiceServer start_operating_srv_;
  ros::ServiceServer stop_operating_srv_;

  // State for running for maplab.
  ros::AsyncSpinner maplab_spinner_;

  // One node to rule them all.
  std::unique_ptr<MaplabServerNode> maplab_server_node_;

  ros::Subscriber map_update_notification_sub_;

  ros::Publisher T_G_curr_M_curr_pub_;
  ros::Publisher T_G_curr_B_curr_pub_;

  ros::Publisher T_G_in_B_in_pub_;
  ros::Publisher T_G_in_M_in_pub_;

  ros::Publisher T_G_curr_M_in_pub_;

  ros::Publisher status_pub_;

  // Publishes the result of any getDenseMapInRange service calls, in addition
  // to returning them by service. This is mainly for introspection such that
  // the operator can see whether the query makes sense.
  ros::Publisher dense_map_query_result_;

  ros::Timer sparse_graph_timer_;
  ros::Duration time_between_sparse_graph_update_requests_;
};

}  // namespace maplab

#endif  // MAPLAB_SERVER_NODE_MAPLAB_SERVER_ROS_NODE_H_
