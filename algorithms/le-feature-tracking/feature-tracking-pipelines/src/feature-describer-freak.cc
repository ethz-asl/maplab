#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "feature-tracking-pipelines/feature-describer-freak.h"
#include "feature-tracking-pipelines/helpers.h"

namespace feature_tracking_pipelines {

FeatureDescriberFreak::FeatureDescriberFreak(
    const FeatureDescriberFreakSettings& settings)
    : settings_(settings) {
  extractor_ = cv::xfeatures2d::FREAK::create(
      settings_.rotation_invariant, settings_.scale_invariant,
      settings_.freak_pattern_scale, settings_.orb_pyramid_levels);
}

void FeatureDescriberFreak::describeFeatures(
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

bool FeatureDescriberFreak::hasNonDescribableFeatures(
    const Eigen::Matrix2Xd& keypoint_measurements,
    const Eigen::RowVectorXd& keypoint_scales,
    std::vector<size_t>* non_describable_indices) {
  // LOG(WARNING) << "IMPL";
  return false;
}

void FeatureDescriberFreak::updateKeyframeWithDescriptors(
    KeyframeFeatures* keyframe, cv::Mat& descriptors_cv,
    const std::vector<cv::KeyPoint>& keypoints_cv) {
  // TODO(lbern) figure out a better way to do this
  keyframe->keypoint_descriptors.resize(2, keypoints_cv.size());
  cv::cv2eigen(descriptors_cv, keyframe->keypoint_descriptors);
  keyframe->keypoint_descriptors = keyframe->keypoint_descriptors.transpose();
  CvKeypointsToKeyframeFeatures(keypoints_cv, keyframe);
  VLOG(5) << "after " << keyframe->keypoint_measurements.cols();
}

}  // namespace feature_tracking_pipelines
