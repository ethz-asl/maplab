#include "aslam/pipeline/undistorter-mapped.h"

#include <aslam/cameras/camera-factory.h>
#include <aslam/common/undistort-helpers.h>
#include <aslam/frames/visual-frame.h>
#include <glog/logging.h>
#include <opencv2/imgproc/imgproc.hpp> // cv::remap

namespace aslam {

std::unique_ptr<MappedUndistorter> createMappedUndistorterToPinhole(
    const aslam::UnifiedProjectionCamera& unified_proj_camera, float alpha,
    float scale, aslam::InterpolationMethod interpolation_type) {
  CHECK_GE(alpha, 0.0);
  CHECK_LE(alpha, 1.0);
  CHECK_GT(scale, 0.0);

  // Create a copy of the input camera.
  UnifiedProjectionCamera::Ptr input_camera(
      dynamic_cast<UnifiedProjectionCamera*>(unified_proj_camera.clone()));
  CHECK(input_camera);

  // Create the scaled output camera with removed distortion.
  const bool kUndistortToPinhole = true;
  Eigen::Matrix3d output_camera_matrix = common::getOptimalNewCameraMatrix(
      *input_camera, alpha, scale, kUndistortToPinhole);

  Eigen::Matrix<double, PinholeCamera::parameterCount(), 1> intrinsics;
  intrinsics <<  output_camera_matrix(0, 0), output_camera_matrix(1, 1),
                 output_camera_matrix(0, 2), output_camera_matrix(1, 2);

  const int output_width = static_cast<int>(scale * input_camera->imageWidth());
  const int output_height = static_cast<int>(scale * input_camera->imageHeight());
  PinholeCamera::Ptr output_camera = aslam::createCamera<aslam::PinholeCamera>(
      intrinsics, output_width, output_height);
  CHECK(output_camera);

  cv::Mat map_u, map_v;
  common::buildUndistortMap(*input_camera, *output_camera, CV_16SC2, map_u, map_v);

  return std::unique_ptr<MappedUndistorter>(
      new MappedUndistorter(input_camera, output_camera, map_u, map_v, interpolation_type));
}

MappedUndistorter::MappedUndistorter()
    : interpolation_method_(aslam::InterpolationMethod::Linear) {}

MappedUndistorter::MappedUndistorter(Camera::Ptr input_camera, Camera::Ptr output_camera,
                                     const cv::Mat& map_u, const cv::Mat& map_v,
                                     aslam::InterpolationMethod interpolation)
: Undistorter(input_camera, output_camera), map_u_(map_u), map_v_(map_v),
  interpolation_method_(interpolation) {
  CHECK_EQ(static_cast<size_t>(map_u_.rows), output_camera->imageHeight());
  CHECK_EQ(static_cast<size_t>(map_u_.cols), output_camera->imageWidth());
  CHECK_EQ(static_cast<size_t>(map_v_.rows), output_camera->imageHeight());
  CHECK_EQ(static_cast<size_t>(map_v_.cols), output_camera->imageWidth());
}

void MappedUndistorter::processImage(const cv::Mat& input_image, cv::Mat* output_image) const {
  CHECK_EQ(input_camera_->imageWidth(), static_cast<size_t>(input_image.cols));
  CHECK_EQ(input_camera_->imageHeight(), static_cast<size_t>(input_image.rows));
  CHECK_NOTNULL(output_image);
  cv::remap(input_image, *output_image, map_u_, map_v_, static_cast<int>(interpolation_method_));
}

}  // namespace aslam
