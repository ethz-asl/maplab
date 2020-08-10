#include "feature-tracking-pipelines/feature-describer-brisk.h"

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <vector>

#include "feature-tracking-pipelines/helpers.h"
#include "feature-tracking-pipelines/keyframe-features.h"

namespace feature_tracking_pipelines {

FeatureDescriberBrisk::FeatureDescriberBrisk(
    const FeatureDescriberBriskSettings& settings)
    : settings_(settings) {
  extractor_.reset(new brisk::BriskDescriptorExtractor(
      settings_.rotation_invariant, settings_.scale_invariant));
}

void FeatureDescriberBrisk::describeFeatures(
    const cv::Mat& image, KeyframeFeatures* keyframe) {
  CHECK_NOTNULL(keyframe);

  std::vector<cv::KeyPoint> keypoints_cv;
  KeyframeFeaturesToCvPoints(*keyframe, &keypoints_cv);
  if (keypoints_cv.empty()) {
    const std::size_t n_keypoints = keyframe->keypoint_measurements.cols();
    keyframe->keypoint_descriptors.resize(2, n_keypoints);
    keyframe->keypoint_descriptors.setZero();
    return;
  }

  cv::Mat descriptors_cv;
  extractor_->compute(image, keypoints_cv, descriptors_cv);
  updateKeyframeWithDescriptors(keyframe, descriptors_cv, keypoints_cv);
}

bool FeatureDescriberBrisk::hasNonDescribableFeatures(
    const Eigen::Matrix2Xd& keypoint_measurements,
    const Eigen::RowVectorXd& keypoint_scales,
    std::vector<size_t>* non_describable_indices) {
  // LOG(WARNING) << "IMPL";
  return false;
}

void FeatureDescriberBrisk::updateKeyframeWithDescriptors(
    KeyframeFeatures* keyframe, cv::Mat& descriptors_cv,
    const std::vector<cv::KeyPoint>& keypoints_cv) {
  // TODO(mariusbr) figure out a better way to do this
  keyframe->keypoint_descriptors.resize(2, keypoints_cv.size());
  cv::cv2eigen(descriptors_cv, keyframe->keypoint_descriptors);
  keyframe->keypoint_descriptors = keyframe->keypoint_descriptors.transpose();
  CvKeypointsToKeyframeFeatures(keypoints_cv, keyframe);
  VLOG(5) << "after " << keyframe->keypoint_measurements.cols();
}

}  // namespace feature_tracking_pipelines
