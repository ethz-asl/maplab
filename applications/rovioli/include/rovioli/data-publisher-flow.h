#ifndef ROVIOLI_DATA_PUBLISHER_FLOW_H_
#define ROVIOLI_DATA_PUBLISHER_FLOW_H_

#include <memory>
#include <string>

#include <maplab-common/conversions.h>
#include <maplab-common/timeout-counter.h>
#include <message-flow/message-flow.h>
#include <tf/transform_broadcaster.h>
#include <visualization/common-rviz-visualization.h>
#include <visualization/rviz-visualization-sink.h>
#include <visualization/viwls-graph-plotter.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#pragma GCC diagnostic pop

#include "rovioli/flow-topics.h"

namespace rovioli {

class DataPublisherFlow {
 public:
  const std::string kRosNamespace = "maplab_rovio";
  const std::string kGeneralTopicPrefix = kRosNamespace + "/";
  const std::string kTopicPoseMission = kGeneralTopicPrefix + "T_M_I";
  const std::string kTopicPoseGlobal = kGeneralTopicPrefix + "T_G_I";
  const std::string kTopicBaseframe = kGeneralTopicPrefix + "T_G_M";
  const std::string kTopicVelocity = kGeneralTopicPrefix + "velocity_I";
  const std::string kTopicBiasAcc = kGeneralTopicPrefix + "bias_acc";
  const std::string kTopicBiasGyro = kGeneralTopicPrefix + "bias_gyro";

  DataPublisherFlow();

  void attachToMessageFlow(message_flow::MessageFlow* flow);
  void visualizeMap(const vi_map::VIMap& vi_map) const;

 private:
  void registerPublishers();
  void stateCallback(
      int64_t timestamp_ns, const vio::ViNodeState& vinode,
      const bool has_T_G_M, const aslam::Transformation& T_G_M);
  void stateDebugCallback(
      const vio::ViNodeState& vinode, const bool has_T_G_M,
      const aslam::Transformation& T_G_M);
  void localizationCallback(const Eigen::Vector3d& p_G_I_lc_pnp);

  std::unique_ptr<visualization::ViwlsGraphRvizPlotter> plotter_;
  ros::NodeHandle node_handle_;
  ros::Publisher pub_pose_T_M_I_;
  ros::Publisher pub_pose_T_G_I_;
  ros::Publisher pub_baseframe_T_G_M_;
  ros::Publisher pub_velocity_I_;
  ros::Publisher pub_imu_acc_bias_;
  ros::Publisher pub_imu_gyro_bias_;

  common::TimeoutCounter map_publisher_timeout_;

  visualization::SphereVector T_M_I_spheres_;
  visualization::SphereVector T_G_I_spheres_;
  visualization::SphereVector T_G_I_loc_spheres_;

  aslam::Transformation latest_T_G_M_;
};

}  //  namespace rovioli

#endif  // ROVIOLI_DATA_PUBLISHER_FLOW_H_
