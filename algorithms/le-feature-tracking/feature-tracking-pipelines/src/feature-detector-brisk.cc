#include "feature-tracking-pipelines/feature-detector-brisk.h"

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <brisk/brisk.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "feature-tracking-pipelines/key-point-bucketing.h"
#include "feature-tracking-pipelines/helpers.h"
#include <iostream>

namespace feature_tracking_pipelines {

FeatureDetectorBrisk::FeatureDetectorBrisk(
    const FeatureDetectorBriskSettings& settings)
    : settings_(settings) {
  detector_.reset(
      new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
          settings.num_pyramid_levels, settings.uniformity_radius,
          settings.absolute_threshold, kMaxDetection));
}

void FeatureDetectorBrisk::detectFeatures(
    const cv::Mat& image, size_t num_features, const cv::Mat& mask,
    KeyframeFeatures *keyframe) {
  CHECK_NOTNULL(keyframe);
  CHECK_LE(num_features, kMaxDetection) << "Increase kMaxDetection!";

  // Detect some keypoints.
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(image, keypoints, mask);

  // Select the best keypoints in a grid.
  size_t num_buckets_u = 4u;
  size_t num_buckets_v = 4u;

  struct CvKeyPointGreaterThan {
    bool operator()(const cv::KeyPoint& a, const cv::KeyPoint& b) {
      return a.response > b.response;
    }
  };
  KeyPointBucketing<cv::KeyPoint, CvKeyPointGreaterThan>(
      image.rows, image.cols, num_features, num_buckets_u, num_buckets_v,
      &keypoints);
  CHECK_LE(keypoints.size(), num_features);

  // Convert keypoints.
  CvKeypointsToKeyframeFeatures(keypoints, keyframe);
}

}  // namespace feature_tracking_pipelines
