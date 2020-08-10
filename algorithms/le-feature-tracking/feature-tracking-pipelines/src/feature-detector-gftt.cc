#include "feature-tracking-pipelines/feature-detector-gftt.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace feature_tracking_pipelines {

FeatureDetectorGFTT::FeatureDetectorGFTT() {}

void FeatureDetectorGFTT::detectFeatures(
    const cv::Mat& image, size_t num_features, const cv::Mat& mask,
    KeyframeFeatures *keyframe) {
  CHECK_NOTNULL(keyframe);

  cv::Mat corners;
  cv::goodFeaturesToTrack(
      image, corners, num_features, /*qualityLevel=*/0.1,
      /*minDistance=*/5, mask, /*blockSize=*/3,
      /*useHarrisDetector=*/true, /*k=*/0.04);

  LOG(FATAL) << corners;
}

}  // namespace feature_tracking_pipelines
