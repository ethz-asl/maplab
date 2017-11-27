#ifndef VISUALIZATION_COMMON_RVIZ_VISUALIZATION_H_
#define VISUALIZATION_COMMON_RVIZ_VISUALIZATION_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <maplab-common/macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "visualization/color.h"
#include "visualization/rviz-visualization-sink.h"
#include "visualization/viz-primitives.h"

namespace visualization {

static const std::string kViMapTopicHead = "vi_map";
static const std::string kDefaultMapFrame = "map";
static const std::string kDefaultMissionFrame = "mission";
static const std::string kDefaultImuFrame = "imu";
static const std::string kDefaultNamespace = "aslam_map_manager";

typedef Aligned<std::vector, Eigen::Vector3d> Vector3dList;

void publishCoordinateFrame(
    const aslam::Transformation& T_fi_fj, const std::string& label, size_t id,
    const std::string& coordinate_frame_topic);

void publishTF(
    const aslam::Transformation& T_fi_fj, const std::string& frame,
    const std::string& child_frame);

void publishTF(
    const aslam::Transformation& T_fi_fj, const std::string& frame,
    const std::string& child_frame, const ros::Time& ros_time);

void publishLines(
    const LineSegmentVector& line_segments, size_t marker_id,
    const std::string& frame, const std::string& name_space,
    const std::string& topic);

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

void publishVerticesFromPoseVector(
    const PoseVector& poses, const std::string& frame,
    const std::string& name_space, const std::string& topic);

void publish3DPointsAsPointCloud(
    const Eigen::Matrix3Xd& points, const visualization::Color& color,
    double alpha, const std::string& frame, const std::string& topic);

void publish3DPointsAsPointCloud(
    const Eigen::Matrix3Xf& points, const Eigen::VectorXf& intensities,
    const std::string& frame, const std::string& topic);

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
    const Eigen::Matrix3Xd& p_G_p0, const Eigen::Matrix3Xd& bearings,
    const std::string& frame, const std::string& name_space,
    const std::string& topic);

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
    const SphereVector& spheres, sensor_msgs::PointCloud2* point_cloud);

}  // namespace visualization

#endif  // VISUALIZATION_COMMON_RVIZ_VISUALIZATION_H_
