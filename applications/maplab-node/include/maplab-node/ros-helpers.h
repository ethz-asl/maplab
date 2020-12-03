#ifndef MAPLAB_NODE_ROS_HELPERS_H_
#define MAPLAB_NODE_ROS_HELPERS_H_

#include <atomic>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop

#include <eigen_conversions/eigen_msg.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab_msgs/OdometryWithImuBiases.h>
#include <minkindr_conversions/kindr_msg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensors/absolute-6dof-pose.h>
#include <sensors/lidar.h>
#include <sensors/loop-closure-sensor.h>
#include <sensors/odometry-6dof-pose.h>
#include <sensors/pointcloud-map-sensor.h>
#include <sensors/wheel-odometry-sensor.h>
#include <vi-map/sensor-manager.h>
#include <vio-common/vio-types.h>

#ifdef VOXGRAPH
#include <voxgraph_msgs/LoopClosureEdge.h>
#include <voxgraph_msgs/LoopClosureEdgeList.h>
#include <voxgraph_msgs/MapSurface.h>
#endif  // VOXGRAPH

#include "maplab-node/odometry-estimate.h"

namespace maplab {

constexpr int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
  return aslam::time::seconds(static_cast<int64_t>(rostime.sec)) +
         static_cast<int64_t>(rostime.nsec);
}

void addRosImuMeasurementToImuMeasurementBatch(
    const sensor_msgs::Imu& imu_msg,
    vio::BatchedImuMeasurements* batched_imu_measurements_ptr);

vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx);

vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::CompressedImageConstPtr& image_message,
    size_t camera_idx);

vi_map::RosLidarMeasurement::Ptr convertRosCloudToMaplabCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const aslam::SensorId& sensor_id);

maplab::OdometryEstimate::Ptr convertRosOdometryMsgToOdometryEstimate(
    const maplab_msgs::OdometryWithImuBiasesConstPtr& msg,
    const aslam::Transformation& T_B_S, const vi_map::Odometry6DoF& sensor);

void odometryCovarianceToEigenMatrix(
    geometry_msgs::PoseWithCovariance::_covariance_type&
        odometry_msg_covariance,
    aslam::TransformationCovariance* covariance);

void eigenMatrixToOdometryCovariance(
    const aslam::TransformationCovariance& covariance,
    double* odometry_msg_covariance_data);

vi_map::Absolute6DoFMeasurement::Ptr
convertPoseWithCovarianceToAbsolute6DoFConstraint(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg,
    const vi_map::Absolute6DoF& sensor);

vi_map::WheelOdometryMeasurement::Ptr convertRosOdometryToMaplabWheelOdometry(
    const nav_msgs::OdometryConstPtr& odometry_msg, aslam::SensorId sensor_id);

#ifdef VOXGRAPH
void convertVoxgraphEdgeListToLoopClosureConstraint(
    const voxgraph_msgs::LoopClosureEdgeListConstPtr& msg,
    const vi_map::LoopClosureSensor& sensor,
    std::vector<vi_map::LoopClosureMeasurement::Ptr>* lc_edges);

vi_map::RosPointCloudMapSensorMeasurement::Ptr
convertVoxgraphMapToPointCloudMap(
    const voxgraph_msgs::MapSurfaceConstPtr& msg,
    const aslam::SensorId& sensor_id);
#else
vi_map::RosPointCloudMapSensorMeasurement::Ptr
convertRosPointCloudToPointCloudMap(
    const sensor_msgs::PointCloud2ConstPtr& msg,
    const aslam::SensorId& sensor_id);
#endif  // VOXGRAPH

}  // namespace maplab
#endif  // MAPLAB_NODE_ROS_HELPERS_H_
