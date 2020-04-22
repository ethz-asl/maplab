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
  const std::string kTopicPoseMission = "T_M_I";
  const std::string kTopicPoseGlobal = "T_G_I";
  const std::string kTopicTransformGlobal = "Transform_G_I";
  const std::string kTopicBaseframe = "T_G_M";
  const std::string kTopicVelocity = "velocity_I";
  const std::string kTopicBiasAcc = "bias_acc";
  const std::string kTopicBiasGyro = "bias_gyro";
  const std::string kCameraExtrinsicTopic = "cam_T_C_B";
  const std::string kTopicMaplabOdomMsg = "maplab_odom_T_M_I";
  const std::string kTopicOdomMsg = "odom_T_M_I";

  DataPublisherFlow();

  void attachToMessageFlow(message_flow::MessageFlow* flow);
  void visualizeMap(const vi_map::VIMap& vi_map) const;

 private:
  void registerPublishers();
  void publishVinsState(
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
  ros::Publisher pub_transform_T_G_I_;
  ros::Publisher pub_baseframe_T_G_M_;
  ros::Publisher pub_velocity_I_;
  ros::Publisher pub_imu_acc_bias_;
  ros::Publisher pub_imu_gyro_bias_;
  ros::Publisher pub_extrinsics_T_C_Bs_;

  // Maplab odometry message publisher (includes IMU biases).
  ros::Publisher pub_maplab_odom_T_M_I_;
  ros::Publisher pub_odom_T_M_I_;

  common::TimeoutCounter map_publisher_timeout_;

  visualization::SphereVector T_M_I_spheres_;
  visualization::SphereVector T_G_I_spheres_;
  visualization::SphereVector T_G_I_loc_spheres_;

  aslam::Transformation latest_T_G_M_;
};

}  //  namespace rovioli

#endif  // ROVIOLI_DATA_PUBLISHER_FLOW_H_
