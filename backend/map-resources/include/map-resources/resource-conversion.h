#ifndef MAP_RESOURCES_RESOURCE_CONVERSION_H_
#define MAP_RESOURCES_RESOURCE_CONVERSION_H_

#include <vector>

#include <aslam/cameras/camera.h>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <resources-common/point-cloud.h>
#include <voxblox/core/common.h>

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

/// Converts a point cloud to a depth map + optionally an intensity image.
/// @param[in]    point_cloud_C         Point cloud in camera coordinate frame,
///                                     which is x-right, z-front, y-down
/// @param[in]    camera                Camera model of the desired depth map
/// @param[in]    use_openni_format     If enabled, the depth map will be in the
///                                     OpenNI format ()[mm], uint16_t),
///                                     otherwise in floating point format [m].
/// @param[in]    create_range_image    If enabled, the depth map will not
///                                     contain the Z coordinate of the 3D point
///                                     but the actual ray length from the
///                                     camera center to the 3D point.
///                                     NOTE This is required for cameras of
///                                     type Camera3DLidar!
/// @param[out]   depth_map             OpenCV depth map, will be set to either
///                                     CV_32FC1 (depth in m) or CV_U16C1 (depth
///                                     in mm)
/// @param[out]   image                 OpenCV intensity image, will be set to
///                                     CV_8UC1 or CV_8UC3, depending on the
///                                     availablility of intensity vs color.It
///                                     will be unallocated if no intensity or
///                                     color is available.
template <typename PointCloudType>
bool convertPointCloudToDepthMap(
    const PointCloudType& point_cloud_C, const aslam::Camera& camera,
    const bool use_openni_format, const bool create_range_image,
    cv::Mat* depth_map, cv::Mat* image);

template <typename InputPointCloud, typename OutputPointCloud>
bool convertPointCloudType(
    const InputPointCloud& input_cloud, OutputPointCloud* output_cloud);

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
void addRingToPointCloud(
    const uint32_t ring, const size_t index, PointCloudType* point_cloud);
template <typename PointCloudType>
void addTimeToPointCloud(
    const float time, const size_t index, PointCloudType* point_cloud);
template <typename PointCloudType>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    PointCloudType* point_cloud);

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
void getRingFromPointCloud(
    const PointCloudType& point_cloud, const size_t index, uint32_t* ring);
template <typename PointCloudType>
void getTimeFromPointCloud(
    const PointCloudType& point_cloud, const size_t index, float* time_s);
template <typename PointCloudType>
void getColorFromPointCloud(
    const PointCloudType& point_cloud, const size_t index,
    resources::RgbaColor* color);

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
bool hasRingInformation(const PointCloudType& point_cloud);
template <typename PointCloudType>
bool hasTimeInformation(const PointCloudType& point_cloud);
template <typename PointCloudType>
bool hasScalarInformation(const PointCloudType& point_cloud);

template <typename PointCloudType>
void resizePointCloud(
    const size_t size, const bool has_color, const bool has_normals,
    const bool has_scalar, const bool has_labels, const bool has_rings,
    const bool has_time, PointCloudType* point_cloud);

uint32_t getPointStep(
    const bool has_color, const bool /*has_normals*/, const bool has_scalar,
    const bool has_labels, const bool has_rings, const bool has_time);

void ignoreFieldsFromPointCloud(resources::PointCloud* point_cloud);

}  // namespace backend

#include "map-resources/resource-conversion-inl.h"

#endif  // MAP_RESOURCES_RESOURCE_CONVERSION_H_
