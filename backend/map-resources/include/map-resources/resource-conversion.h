#ifndef MAP_RESOURCES_RESOURCE_CONVERSION_H_
#define MAP_RESOURCES_RESOURCE_CONVERSION_H_

#include <vector>

#include <aslam/cameras/camera.h>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <voxblox/core/common.h>

#include "map-resources/resource-typedefs.h"

namespace backend {

// Converts depth map and corresponding image to a point cloud.
// The depth map is assumed to follow the OpenNI format.
// The image can either be a 8bit grayscale or 8bit BGR image.
template <typename PointCloudType>
bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    PointCloudType* point_cloud);

template <typename InputPointCloud, typename OutputPointCloud>
bool convertPointCloudType(
    const InputPointCloud& input_cloud, OutputPointCloud* output_cloud);

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    resources::PointCloud* point_cloud);

bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const aslam::Camera& camera,
    pose::Position3DVector* point_cloud);

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    resources::PointCloud* point_cloud);

bool convertDepthMapWithImageToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    pose::Position3DVector* points_C, voxblox::Colors* colors);

// In maplab we usually store the camera with the full distortion model, however
// the images that correspond to the depth maps are usually computed from
// undistorted images, therefore we need to be able to obtain a version of the
// camera that does not have a distortion.
void createCameraWithoutDistortion(
    const aslam::Camera& camera, aslam::Camera::Ptr* camera_without_distortion);

}  // namespace backend

#include "map-resources/resource-conversion-inl.h"

#endif  // MAP_RESOURCES_RESOURCE_CONVERSION_H_
