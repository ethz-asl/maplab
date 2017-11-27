#include "dense-reconstruction/stereo-matcher.h"

#include <memory>

#include <Eigen/Dense>
#include <aslam/cameras/camera.h>
#include <aslam/common/pose-types.h>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "dense-reconstruction/aslam-cv-interface.h"
#include "dense-reconstruction/disparity-conversion-utils.h"
#include "dense-reconstruction/stereo-camera-utils.h"

DEFINE_double(
    dense_stereo_downscaling_factor, 1.0,
    "Downscaling factor applied to the images prior to stereo matching.");

DEFINE_bool(dense_stereo_use_sgbm, true, "Use SGBM if enabled, BM otherwise.");

DEFINE_int32(dense_stereo_sgbm_min_disparity, 0, "");
DEFINE_int32(dense_stereo_sgbm_num_disparities, 128, "");
DEFINE_int32(dense_stereo_sgbm_sad_window_size, 3, "");
DEFINE_int32(dense_stereo_sgbm_p1, 72, "");
DEFINE_int32(dense_stereo_sgbm_p2, 288, "");
DEFINE_int32(dense_stereo_sgbm_disp12_max_diff, 1, "");
DEFINE_int32(dense_stereo_sgbm_pre_filter_cap, 31, "");
DEFINE_int32(dense_stereo_sgbm_uniqueness_ratio, 10, "");
DEFINE_int32(dense_stereo_sgbm_speckle_window_size, 500, "");
DEFINE_int32(dense_stereo_sgbm_speckle_range, 3, "");
DEFINE_int32(
    dense_stereo_sgbm_mode, cv::StereoSGBM::MODE_SGBM,
    "MODE_SGBM = 0, MODE_HH = 1, MODE_SGBM_3WAY = 2, MODE_HH4 = 3");

DEFINE_int32(dense_stereo_bm_pre_filter_size, 9, "");
DEFINE_int32(dense_stereo_bm_pre_filter_cap, 31, "");
DEFINE_string(dense_stereo_bm_prefilter_type, "xsobel", "");
DEFINE_int32(dense_stereo_bm_texture_threshold, 10, "");
DEFINE_int32(dense_stereo_bm_uniqueness_ratio, 15, "");
DEFINE_int32(dense_stereo_bm_sad_window_size, 9, "");
DEFINE_int32(dense_stereo_bm_min_disparity, 0, "");
DEFINE_int32(dense_stereo_bm_num_disparities, 128, "");
DEFINE_int32(dense_stereo_bm_speckle_window_size, 500, "");
DEFINE_int32(dense_stereo_bm_speckle_range, 3, "");
DEFINE_int32(dense_stereo_bm_disp12_max_diff, 1, "");

namespace dense_reconstruction {
namespace stereo {

void StereoMatcherConfig::adaptParamsBasedOnImageSize(
    const size_t image_width) {
  // Source:
  // https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_match.cpp

  const int sgbm_old_num_disparities = sgbm_num_disparities;
  const int bm_old_num_disparities = bm_num_disparities;
  sgbm_num_disparities = ((image_width / 8) + 15) & -16;
  bm_num_disparities = sgbm_num_disparities;
  LOG(INFO)
      << "Adapted the number of disparities based on the image width from "
      << sgbm_old_num_disparities << "/" << bm_old_num_disparities << " to "
      << sgbm_num_disparities;

  const int old_p1 = sgbm_p1;
  const int old_p2 = sgbm_p2;
  sgbm_p1 = 8 * sgbm_sad_window_size * sgbm_sad_window_size;
  sgbm_p2 = 32 * sgbm_sad_window_size * sgbm_sad_window_size;

  LOG(INFO) << "Adapted the SGBM params p1/p2 based on sad window size from "
            << old_p1 << "/" << old_p2 << " to " << sgbm_p1 << "/" << sgbm_p2;
}

StereoMatcherConfig StereoMatcherConfig::getFromGflags() {
  StereoMatcherConfig config;

  // General config:
  config.downscaling_factor = FLAGS_dense_stereo_downscaling_factor;
  config.use_sgbm = FLAGS_dense_stereo_use_sgbm;

  // SGBM config:
  config.sgbm_min_disparity = FLAGS_dense_stereo_sgbm_min_disparity;
  config.sgbm_num_disparities = FLAGS_dense_stereo_sgbm_num_disparities;
  config.sgbm_sad_window_size = FLAGS_dense_stereo_sgbm_sad_window_size;
  config.sgbm_p1 = FLAGS_dense_stereo_sgbm_p1;
  config.sgbm_p2 = FLAGS_dense_stereo_sgbm_p2;
  config.sgbm_disp12_max_diff = FLAGS_dense_stereo_sgbm_disp12_max_diff;
  config.sgbm_pre_filter_cap = FLAGS_dense_stereo_sgbm_pre_filter_cap;
  config.sgbm_uniqueness_ratio = FLAGS_dense_stereo_sgbm_uniqueness_ratio;
  config.sgbm_speckle_window_size = FLAGS_dense_stereo_sgbm_speckle_window_size;
  config.sgbm_speckle_range = FLAGS_dense_stereo_sgbm_speckle_range;
  config.sgbm_mode = FLAGS_dense_stereo_sgbm_mode;

  // BM config:
  config.bm_pre_filter_size = FLAGS_dense_stereo_bm_pre_filter_size;
  config.bm_pre_filter_cap = FLAGS_dense_stereo_bm_pre_filter_cap;
  config.bm_prefilter_type = FLAGS_dense_stereo_bm_prefilter_type;
  config.bm_texture_threshold = FLAGS_dense_stereo_bm_texture_threshold;
  config.bm_uniqueness_ratio = FLAGS_dense_stereo_bm_uniqueness_ratio;
  config.bm_sad_window_size = FLAGS_dense_stereo_bm_sad_window_size;
  config.bm_min_disparity = FLAGS_dense_stereo_bm_min_disparity;
  config.bm_num_disparities = FLAGS_dense_stereo_bm_num_disparities;
  config.bm_speckle_window_size = FLAGS_dense_stereo_bm_speckle_window_size;
  config.bm_speckle_range = FLAGS_dense_stereo_bm_speckle_range;
  config.bm_disp12_max_diff = FLAGS_dense_stereo_bm_disp12_max_diff;

  return config;
}

StereoMatcher::StereoMatcher(
    const aslam::Camera& first_camera, const aslam::Camera& second_camera,
    const aslam::Transformation& T_C2_C1, const StereoMatcherConfig& config)
    : config_(config),
      first_camera_(first_camera),
      second_camera_(second_camera),
      T_C2_C1_(T_C2_C1),
      stereo_camera_params_(config_.downscaling_factor),
      undistorter_first_(nullptr),
      undistorter_second_(nullptr) {
  getStereoPairFromAslamCvCameras(
      first_camera_, second_camera_, T_C2_C1_, config_.downscaling_factor,
      &stereo_camera_params_);

  undistorter_first_.reset(new Undistorter(stereo_camera_params_.getFirst()));
  undistorter_second_.reset(new Undistorter(stereo_camera_params_.getSecond()));

  if (config_.use_sgbm) {
    VLOG(1) << "Stereo matching algorithm used: SGBM";
    stereo_matcher_ = cv::StereoSGBM::create(
        config_.sgbm_min_disparity, config_.sgbm_num_disparities,
        config_.sgbm_sad_window_size, config_.sgbm_p1, config_.sgbm_p2,
        config_.sgbm_disp12_max_diff, config_.sgbm_pre_filter_cap,
        config_.sgbm_uniqueness_ratio, config_.sgbm_speckle_window_size,
        config_.sgbm_speckle_range, config_.sgbm_mode);

    min_disparity_ = config_.sgbm_min_disparity;
    num_disparities_ = config_.sgbm_num_disparities;
    sad_window_size_ = config_.sgbm_sad_window_size;
  } else {
    VLOG(1) << "Stereo matching algorithm used: BM";
    stereo_matcher_ = cv::StereoBM::create(
        config_.bm_num_disparities, config_.bm_sad_window_size);

    cv::StereoBM* bm_ptr = static_cast<cv::StereoBM*>(stereo_matcher_.get());
    bm_ptr->setPreFilterCap(config_.bm_pre_filter_cap);
    bm_ptr->setPreFilterSize(config_.bm_pre_filter_size);
    bm_ptr->setMinDisparity(config_.bm_min_disparity);
    bm_ptr->setTextureThreshold(config_.bm_texture_threshold);
    bm_ptr->setUniquenessRatio(config_.bm_uniqueness_ratio);
    bm_ptr->setSpeckleRange(config_.bm_speckle_range);
    bm_ptr->setSpeckleWindowSize(config_.bm_speckle_window_size);
    bm_ptr->setDisp12MaxDiff(config_.bm_disp12_max_diff);

    min_disparity_ = config_.bm_min_disparity;
    num_disparities_ = config_.bm_num_disparities;
    sad_window_size_ = config_.bm_sad_window_size;
  }

  // Cache some intrinsics values:
  const std::shared_ptr<OutputCameraParameters> left_params =
      undistorter_first_->getCameraParametersPair().getOutputPtr();
  const std::shared_ptr<OutputCameraParameters> right_params =
      undistorter_second_->getCameraParametersPair().getOutputPtr();

  focal_length_ = left_params->P()(0, 0);
  baseline_ = (right_params->P()(0, 3) - left_params->P()(0, 3)) /
              left_params->P()(0, 0);

  cx_ = left_params->P()(0, 2);
  const double cx_right = right_params->P()(0, 2);
  CHECK_EQ(cx_, cx_right)
      << "The undistortion should have made cx_left and cx_right identical!";

  cy_ = left_params->P()(1, 2);

  CHECK_GT(std::abs(baseline_), 1e-6);
}

void StereoMatcher::computeDisparityMap(
    const cv::Mat& first_image, const cv::Mat& second_image,
    cv::Mat* disparity_map, cv::Mat* first_image_undistorted,
    cv::Mat* second_image_undistorted) const {
  CHECK_NOTNULL(disparity_map);
  CHECK_NOTNULL(first_image_undistorted);
  CHECK_NOTNULL(second_image_undistorted);
  CHECK(undistorter_first_);
  CHECK(undistorter_second_);
  CHECK(stereo_matcher_);

  VLOG(5) << "Undistorting and rectifying images...";
  undistorter_first_->undistortImage(first_image, first_image_undistorted);
  undistorter_second_->undistortImage(second_image, second_image_undistorted);

  VLOG(5) << "Computing disparity map...";
  stereo_matcher_->compute(
      *first_image_undistorted, *second_image_undistorted, *disparity_map);
  VLOG(5) << "Done.";
}

void StereoMatcher::computePointCloud(
    const cv::Mat& first_image, const cv::Mat& second_image,
    resources::PointCloud* point_cloud) const {
  CHECK_NOTNULL(point_cloud);

  cv::Mat disparity_map, first_image_undistorted, second_image_undistorted;
  computeDisparityMap(
      first_image, second_image, &disparity_map, &first_image_undistorted,
      &second_image_undistorted);

  convertDisparityMapToPointCloud(
      disparity_map, first_image_undistorted, baseline_, focal_length_, cx_,
      cy_, sad_window_size_, min_disparity_, num_disparities_, point_cloud);
}

void StereoMatcher::computeDepthMapForOriginalCamera(
    const cv::Mat& first_image, const cv::Mat& second_image,
    cv::Mat* depth_map) const {
  CHECK_NOTNULL(depth_map);

  cv::Mat disparity_map, first_image_undistorted, second_image_undistorted;
  computeDisparityMap(
      first_image, second_image, &disparity_map, &first_image_undistorted,
      &second_image_undistorted);

  convertDisparityMapToDepthMap(
      disparity_map, first_image_undistorted, baseline_, focal_length_, cx_,
      cy_, sad_window_size_, min_disparity_, num_disparities_, first_camera_,
      depth_map);
}

}  // namespace stereo
}  // namespace dense_reconstruction
