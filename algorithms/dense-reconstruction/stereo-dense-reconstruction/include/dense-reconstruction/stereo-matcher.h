#ifndef DENSE_RECONSTRUCTION_STEREO_MATCHER_H_
#define DENSE_RECONSTRUCTION_STEREO_MATCHER_H_

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <aslam/cameras/camera.h>
#include <aslam/common/pose-types.h>
#include <map-resources/resource-typedefs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "dense-reconstruction/stereo-camera-utils.h"

namespace dense_reconstruction {
namespace stereo {

struct StereoMatcherConfig {
  StereoMatcherConfig() {}

  // NOTE(mfehr): If you change these values, you need to adapt the "ground
  // truth" of the unit test in test_stereo_dense_reconstruction.cpp. This can
  // be easily done by setting kRecomputeComparisonResults to true and running
  // the unit test once. Then proceed by switching it to false again and check
  // the unit test results. Finally commit the new "ground truth" disparity
  // files. Furthermore, you need to adapt the expected size of the resulting
  // point cloud. Note that some datasets that are tested set their own
  // parameter.

  double downscaling_factor = 1.0;

  bool use_sgbm = true;

  int sgbm_min_disparity = 0;
  int sgbm_num_disparities = 128;
  int sgbm_sad_window_size = 3;
  int sgbm_p1 = 72;
  int sgbm_p2 = 288;
  int sgbm_disp12_max_diff = 1;
  int sgbm_pre_filter_cap = 31;
  int sgbm_uniqueness_ratio = 10;
  int sgbm_speckle_window_size = 500;
  int sgbm_speckle_range = 3;
  // MODE_SGBM = 0, MODE_HH = 1, MODE_SGBM_3WAY = 2, MODE_HH4 = 3
  int sgbm_mode = cv::StereoSGBM::MODE_SGBM;

  int bm_pre_filter_size = 9;
  int bm_pre_filter_cap = 31;
  std::string bm_prefilter_type = "xsobel";
  int bm_texture_threshold = 10;
  int bm_uniqueness_ratio = 15;
  int bm_sad_window_size = 9;
  int bm_min_disparity = 0;
  int bm_num_disparities = 128;
  int bm_speckle_window_size = 500;
  int bm_speckle_range = 3;
  int bm_disp12_max_diff = 1;

  static StereoMatcherConfig getFromGflags();

  void adaptParamsBasedOnImageSize(const size_t image_width);
};

// Stereo matcher convenience class that uses OpenCV stereo matches to compute
// disparity, depth maps or point cloud based on a stereo camera setup and the
// provided images.
class StereoMatcher {
 public:
  // Initialize the stereo matcher with the stereo camera intrinsics and
  // extrinsics. This will create and cache an undistorter for this stereo pair.
  StereoMatcher(
      const aslam::Camera& first_camera, const aslam::Camera& second_camera,
      const aslam::Transformation& T_C2_C1, const StereoMatcherConfig& config);

  // Compute a disparity map for the stereo pair.
  void computeDisparityMap(
      const cv::Mat& first_image, const cv::Mat& second_image,
      cv::Mat* disparity_map, cv::Mat* first_image_undistorted,
      cv::Mat* second_image_undistorted) const;

  // Compute a point cloud in the camera coordinate frame of the first camera
  // for the stereo pair.
  void computePointCloud(
      const cv::Mat& first_image, const cv::Mat& second_image,
      resources::PointCloud* point_cloud) const;

  // Compute a depth map by projecting the reconstructed pointcloud into the
  // first camera frame..
  void computeDepthMapForOriginalCamera(
      const cv::Mat& first_image, const cv::Mat& second_image,
      cv::Mat* depth_map) const;

  double baseline() const {
    return baseline_;
  }
  double cx() const {
    return cx_;
  }
  double cy() const {
    return cy_;
  }
  double focal_length() const {
    return focal_length_;
  }
  int min_disparity() const {
    return min_disparity_;
  }
  int num_disparities() const {
    return num_disparities_;
  }
  int sad_window_size() const {
    return sad_window_size_;
  }

 private:
  const StereoMatcherConfig config_;

  const aslam::Camera& first_camera_;
  const aslam::Camera& second_camera_;

  const aslam::Transformation T_C2_C1_;

  // Convenience class that stores the camera intrinsics and extrinsics as well
  // as the undistortion and rectification paramters.
  StereoCameraParameters stereo_camera_params_;

  // Convenience class to compute and cache the stereo
  // undistortion/rectification mapping.
  std::unique_ptr<Undistorter> undistorter_first_;
  std::unique_ptr<Undistorter> undistorter_second_;

  cv::Ptr<cv::StereoMatcher> stereo_matcher_;

  // Cached intrinsics:
  double focal_length_;
  double baseline_;
  double cx_;
  double cy_;

  int min_disparity_;
  int num_disparities_;
  int sad_window_size_;
};

}  // namespace stereo
}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_STEREO_MATCHER_H_
