#ifndef DENSE_RECONSTRUCTION_DISPARITY_CONVERSION_UTILS_H_
#define DENSE_RECONSTRUCTION_DISPARITY_CONVERSION_UTILS_H_

#include <aslam/cameras/camera.h>
#include <map-resources/resource-typedefs.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace dense_reconstruction {
namespace stereo {

void convertDisparityMapToPointCloud(
    const cv::Mat& input_disparity, const cv::Mat& left_image,
    const double baseline, const double focal_length, const double cx,
    const double cy, const int sad_window_size, const int min_disparity,
    const int num_disparities, resources::PointCloud* pointcloud);

void convertDisparityMapToDepthMap(
    const cv::Mat& disparity_map, const cv::Mat& first_image_undistorted,
    const double baseline, const double focal_length, const double cx,
    const double cy, const int sad_window_size, const int min_disparity,
    const int num_disparities, const aslam::Camera& target_camera,
    cv::Mat* depth_map);

}  // namespace stereo
}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_DISPARITY_CONVERSION_UTILS_H_
