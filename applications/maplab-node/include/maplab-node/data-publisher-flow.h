#ifndef MAPLAB_NODE_DATA_PUBLISHER_FLOW_H_
#define MAPLAB_NODE_DATA_PUBLISHER_FLOW_H_

#include <maplab-common/conversions.h>
#include <maplab-common/timeout-counter.h>
#include <memory>
#include <message-flow/message-flow.h>
#include <string>
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

#include "maplab-node/flow-topics.h"

namespace maplab {

class DataPublisherFlow {
 public:
  const std::string kTopicPoseMission = "T_M_B";
  const std::string kTopicVelocity = "velocity_I";
  const std::string kTopicBiasAcc = "bias_acc";
  const std::string kTopicBiasGyro = "bias_gyro";
  const std::string kCameraExtrinsicTopic = "cam_T_C_B";

  explicit DataPublisherFlow(const vi_map::SensorManager& sensor_manager);

  void attachToMessageFlow(message_flow::MessageFlow* flow);
  void visualizeMap(const vi_map::VIMap& vi_map) const;

 private:
  void registerPublishers();
  void publishOdometryState(
      int64_t timestamp_ns, const vio::ViNodeState& vinode);
  void publishVinsState(int64_t timestamp_ns, const vio::ViNodeState& vinode);
  void stateDebugCallback(const vio::ViNodeState& vinode);

  const vi_map::SensorManager& sensor_manager_;

  std::unique_ptr<visualization::ViwlsGraphRvizPlotter> plotter_;
  ros::NodeHandle node_handle_;
  ros::Publisher pub_pose_T_M_B_;
  ros::Publisher pub_velocity_I_;
  ros::Publisher pub_imu_acc_bias_;
  ros::Publisher pub_imu_gyro_bias_;

  common::TimeoutCounter map_publisher_timeout_;

  visualization::SphereVector T_M_B_spheres_;
};

}  //  namespace maplab

#endif  // MAPLAB_NODE_DATA_PUBLISHER_FLOW_H_
