#ifndef VISUALIZATION_COMMON_RVIZ_VISUALIZATION_H_
#define VISUALIZATION_COMMON_RVIZ_VISUALIZATION_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <vi-map/sensor-manager.h>
#include <visualization_msgs/Marker.h>

#include "visualization/color.h"
#include "visualization/rviz-visualization-sink.h"
#include "visualization/viz-primitives.h"

DECLARE_string(tf_map_frame);
DECLARE_string(tf_mission_frame);
DECLARE_string(tf_abs_6dof_sensor_frame);
DECLARE_string(tf_odometry_6dof_sensor_frame);
DECLARE_string(tf_wheel_odometry_sensor_frame);
DECLARE_string(tf_lc_sensor_frame);
DECLARE_string(tf_lidar_sensor_frame);
DECLARE_string(tf_pointcloud_map_frame);
DECLARE_string(tf_gps_wgs_sensor_frame);
DECLARE_string(tf_gps_utm_sensor_frame);
DECLARE_string(tf_imu_frame);
DECLARE_string(tf_camera_frame);
DECLARE_string(tf_ncamera_frame);
DECLARE_string(tf_imu_refined_frame);
DECLARE_string(vis_default_namespace);

namespace visualization {

static const std::string kViMapTopicHead = "vi_map";

typedef Aligned<std::vector, Eigen::Vector3d> Vector3dList;

void publishCoordinateFrame(
    const aslam::Transformation& T_fi_fj, const std::string& label, size_t id,
    const std::string& coordinate_frame_topic);

void publishTF(
    const aslam::Transformation& T_fi_fj, const std::string& frame,
    const std::string& child_frame);

const std::string convertSensorTypeToTfFrameId(
    const vi_map::SensorType sensor_type);

void publishSensorTFs(const vi_map::SensorManager& sensor_manager,
                      const ros::Time& ros_time);

void publishTF(
    const aslam::Transformation& T_fi_fj, const std::string& frame,
    const std::string& child_frame, const ros::Time& ros_time);

void publishLines(
    const LineSegmentVector& line_segments, size_t marker_id,
    const std::string& frame, const std::string& name_space,
    const std::string& topic, const bool wait_for_subscriber = false);

void publishLines(
    const Eigen::Matrix3Xd& points_from, const Eigen::Matrix3Xd& points_to,
    const std::vector<visualization::Color>& colors, double alpha, double scale,
    size_t marker_id, const std::string& frame, const std::string& name_space,
    const std::string& topic);

void publishLines(
    const Eigen::Vector3d& common_line_start_point,
    const Vector3dList& line_end_points,
    const std::vector<visualization::Color>& colors, double alpha, double scale,
    size_t marker_id, const std::string& frame, const std::string& name_space,
    const std::string& topic);

void publishArrow(
    const Arrow& arrow, size_t marker_id, const std::string& frame,
    const std::string& name_space, const std::string& topic);

void publishArrows(
    const ArrowVector& arrows, size_t marker_id, const std::string& frame,
    const std::string& name_space, const std::string& topic);

void publishVerticesFromPoseVector(
    const PoseVector& poses, const std::string& frame,
    const std::string& name_space, const std::string& topic);

void publish3DPointsAsPointCloud(
    const Eigen::Matrix3Xd& points, const visualization::Color& color,
    double alpha, const std::string& frame, const std::string& topic);

void publish3DPointsAsPointCloud(
    const Eigen::Matrix3Xf& points, const Eigen::VectorXf& intensities,
    const std::string& frame, const std::string& topic);

void publishPoseCovariances(
    const std::vector<aslam::Transformation>& T_G_B_vec,
    const std::vector<aslam::TransformationCovariance>& B_cov_vec,
    const Color& color, const std::string& frame, const std::string& name_space,
    const std::string& topic);

void publishSpheresAsPointCloud(
    const SphereVector& spheres, const std::string& frame,
    const std::string& topic);

void publish3DPointsAsSpheres(
    const Eigen::Matrix3Xd& points, const visualization::Color& color,
    double alpha, double scale, size_t marker_id, const std::string& frame,
    const std::string& name_space, const std::string& topic);

void publishSpheres(
    const SphereVector& spheres, size_t marker_id, const std::string& frame,
    const std::string& name_space, const std::string& topic);

void publishNormals(
    const Color& color, unsigned int marker_id, const std::string& frame,
    const std::string& name_space, const std::string& topic);

void publishFilledBoxes(
    const FilledBoxVector& boxes, const std::vector<size_t>& box_marker_ids,
    const std::string& frame, const std::string& wireframe_namespace,
    const std::string& filling_namespace, const std::string& topic);

void publishFilledBox(
    const FilledBox& box, size_t marker_id, const std::string& frame,
    const std::string& wireframe_namespace,
    const std::string& filling_namespace, const std::string& topic);

void publishMesh(
    const Aligned<std::vector, Eigen::Matrix3d>& triangles, size_t marker_id,
    double scale, const std::string& frame, const std::string name_space,
    const std::string& topic);

void publishMesh(
    const std::string& mesh_filename, const aslam::Transformation& T_G_fi,
    const aslam::Transformation& T_fi_mesh, const double scale,
    const visualization::Color color, size_t marker_id,
    const std::string& frame, const std::string& name_space,
    const std::string& topic);

void publishTransformations(
    const aslam::TransformationVector& Ts,
    const std::vector<visualization::Color>& colors, double alpha,
    const std::string& frame, const std::string& name_space,
    const std::string& topic);

template <typename PointType>
void publishPointCloud(
    const pcl::PointCloud<PointType>& point_cloud, const std::string& frame,
    const std::string& topic) {
  CHECK(!topic.empty());

  CHECK(ros::isInitialized())
      << "ROS hasn't been initialized. Call "
      << "RVizVisualizationSink::init() in your application code if you intend"
      << " to use RViz visualizations.";

  sensor_msgs::PointCloud2 point_cloud2;
  pcl::toROSMsg(point_cloud, point_cloud2);

  point_cloud2.header.frame_id = frame;
  point_cloud2.header.stamp = ros::Time::now();

  RVizVisualizationSink::publish<sensor_msgs::PointCloud2>(topic, point_cloud2);
}

void publishSpheresAsPointCloud(
    const SphereVector& spheres, const std::string& frame,
    const std::string& topic);

void deleteMarker(const std::string& topic, size_t marker_id);

///////////////////////
//// TYPE CONVERTERS
///////////////////////
void eigen3XdMatrixToSpheres(
    const Eigen::Matrix3Xd& G_points, visualization_msgs::Marker* spheres);

// Converts a 3X matrix of points into a point cloud message.
// Color and alpha are optional. If not specified, white and full alpha is used.
void eigen3XdMatrixToPointCloud(
    const Eigen::Matrix3Xd& points, const visualization::Color& color,
    unsigned char alpha, sensor_msgs::PointCloud2* point_cloud);
void eigen3XdMatrixToPointCloud(
    const Eigen::Matrix3Xd& points, const visualization::Color& color,
    sensor_msgs::PointCloud2* point_cloud);
void eigen3XdMatrixToPointCloud(
    const Eigen::Matrix3Xd& points, sensor_msgs::PointCloud2* point_cloud);
void spheresToPointCloud(
    const SphereVector& spheres, sensor_msgs::PointCloud2* point_cloud,
    const std::string& frame, const std::string& topic);

}  // namespace visualization

#endif  // VISUALIZATION_COMMON_RVIZ_VISUALIZATION_H_
