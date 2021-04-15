#include "feature-tracking-pipelines/feature-detector-sift.h"

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp>


#include "feature-tracking-pipelines/key-point-bucketing.h"
#include "feature-tracking-pipelines/helpers.h"
#include <iostream>

namespace feature_tracking_pipelines {

FeatureDetectorSift::FeatureDetectorSift(
    const FeatureDetectorSiftSettings& settings)
    : settings_(settings) {
  detector_ = cv::xfeatures2d::SIFT::create(
    settings_.n_features,
    settings_.n_octave_layers,
    settings_.contrast_threshold,
    settings_.edge_threshold,
    settings_.sigma);
}

void FeatureDetectorSift::detectFeatures(
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
