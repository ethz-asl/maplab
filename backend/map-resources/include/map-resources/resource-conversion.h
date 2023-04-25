#ifndef MAP_RESOURCES_RESOURCE_CONVERSION_H_
#define MAP_RESOURCES_RESOURCE_CONVERSION_H_

#include <aslam/cameras/camera.h>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <resources-common/point-cloud.h>

#include "map-resources/resource-common.h"
#include "map-resources/resource-typedefs.h"

namespace backend {

// Converts depth map and corresponding image to a point cloud.
// The depth map is assumed to follow the OpenNI format.
// The intensity image can either be a 8bit grayscale or 8bit BGR image.
template <typename PointCloudType>
bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    PointCloudType* point_cloud_C);

template <typename InputPointCloud, typename OutputPointCloud>
bool convertPointCloudType(
    const InputPointCloud& input_cloud, OutputPointCloud* output_cloud,
    bool with_timestamps = false, uint32_t convert_to_ns = 1,
    int64_t time_offset_ns = 0);

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    resources::PointCloud* point_cloud);

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    voxblox::Pointcloud* point_cloud);

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    resources::PointCloud* point_cloud);

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    voxblox::Pointcloud* points_C, voxblox::Colors* colors);

// In maplab we usually store the camera with the full distortion model, however
// the images that correspond to the depth maps are usually computed from
// undistorted images, therefore we need to be able to obtain a version of the
// camera that does not have a distortion.
void createCameraWithoutDistortion(
    const aslam::Camera& camera, aslam::Camera::Ptr* camera_without_distortion);

template <typename PointCloudType>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    PointCloudType* point_cloud);
template <typename PointCloudType>
void addScalarToPointCloud(
    const float scalar, const size_t index, PointCloudType* point_cloud);
template <typename PointCloudType>
void addLabelToPointCloud(
    const uint32_t label, const size_t index, PointCloudType* point_cloud);
template <typename PointCloudType>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    PointCloudType* point_cloud);
template <typename PointCloudType>
void addTimeToPointCloud(
    const uint32_t time, const size_t index, PointCloudType* point_cloud);

template <typename PointCloudType>
void getPointFromPointCloud(
    const PointCloudType& point_cloud, const size_t index,
    Eigen::Vector3d* point_C);
template <typename PointCloudType>
void getScalarFromPointCloud(
    const PointCloudType& point_cloud, const size_t index, float* scalar);
template <typename PointCloudType>
void getLabelFromPointCloud(
    const PointCloudType& point_cloud, const size_t index, uint32_t* label);
template <typename PointCloudType>
void getColorFromPointCloud(
    const PointCloudType& point_cloud, const size_t index,
    resources::RgbaColor* color);
template <typename PointCloudType>
void getTimeFromPointCloud(
    const PointCloudType& point_cloud, const size_t index, uint32_t* time,
    const uint32_t convert_to_ns, const int64_t time_offset_ns);

template <typename PointCloudType>
size_t getPointCloudSize(const PointCloudType& point_cloud);

// Tries to figure out which maplab point cloud resource type fits the fields
// present in this point cloud type.
template <typename PointCloudType>
ResourceType getResourceTypeForPointCloud(const PointCloudType& point_cloud);

template <typename PointCloudType>
bool hasColorInformation(const PointCloudType& point_cloud);
template <typename PointCloudType>
bool hasNormalsInformation(const PointCloudType& point_cloud);
template <typename PointCloudType>
bool hasLabelInformation(const PointCloudType& point_cloud);
template <typename PointCloudType>
bool hasScalarInformation(const PointCloudType& point_cloud);
template <typename PointCloudType>
bool hasTimeInformation(const PointCloudType& point_cloud);

template <typename PointCloudType>
void resizePointCloud(
    const size_t size, const bool has_color, const bool has_normals,
    const bool has_scalar, const bool has_labels, const bool has_times,
    PointCloudType* point_cloud);

}  // namespace backend

#include "map-resources/resource-conversion-inl.h"

#endif  // MAP_RESOURCES_RESOURCE_CONVERSION_H_
