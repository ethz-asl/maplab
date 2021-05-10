#include "feature-tracking-pipelines/feature-describer-surf.h"
#include "feature-tracking-pipelines/helpers.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace feature_tracking_pipelines {

FeatureDescriberSurf::FeatureDescriberSurf(
    const FeatureDescriberSurfSettings& settings)
  : settings_(settings) {
  extractor_ = cv::xfeatures2d::SURF::create(
    settings_.hessian_threshold,
    settings_.n_octaves,
    settings_.octave_layers,
    settings_.extended,
    settings_.upright);
  //extractor_ = cv::xfeatures2d::SURF::create();
}

void FeatureDescriberSurf::describeFeatures(
      const cv::Mat& image, KeyframeFeatures* keyframe) {
  CHECK_NOTNULL(keyframe);

  std::vector<cv::KeyPoint> keypoints_cv;
  KeyframeFeaturesToCvPoints(*keyframe, &keypoints_cv);
  if (keypoints_cv.empty()){ 
    const std::size_t n_keypoints = keyframe->keypoint_measurements.cols();
    keyframe->keypoint_descriptors.resize(2, n_keypoints);
    keyframe->keypoint_descriptors.setZero();
    return;
  }

  cv::Mat descriptors_cv;
  extractor_->compute(image, keypoints_cv, descriptors_cv);
  updateKeyframeWithDescriptors(keyframe, descriptors_cv, keypoints_cv);
}

bool FeatureDescriberSurf::hasNonDescribableFeatures(
    const Eigen::Matrix2Xd& keypoint_measurements,
    const Eigen::RowVectorXd& keypoint_scales,
    std::vector<size_t>* non_describable_indices) {
  // LOG(WARNING) << "IMPL";
  return false;
}

void FeatureDescriberSurf::updateKeyframeWithDescriptors(
    KeyframeFeatures* keyframe,
    cv::Mat& descriptors_cv,
    const std::vector<cv::KeyPoint>& keypoints_cv) {
  // TODO(lbern) figure out a better way to do this
  keyframe->keypoint_descriptors.resize(2, keypoints_cv.size());
  cv::cv2eigen(descriptors_cv, keyframe->keypoint_descriptors);
  keyframe->keypoint_descriptors = keyframe->keypoint_descriptors.transpose();
  CvKeypointsToKeyframeFeatures(keypoints_cv, keyframe);
  VLOG(1) << "after " << keyframe->keypoint_measurements.cols();
}

std::vector<cv::KeyPoint> FeatureDescriberSurf::rawToCvKeyPoint(
    const Eigen::Matrix2Xd& keypoint_measurements,
    const Eigen::RowVectorXd& keypoint_scales,
    const Eigen::RowVectorXd& keypoint_scores,
    const Eigen::RowVectorXi& keypoint_ids,
    const Eigen::RowVectorXd& keypoint_orientations_rad) const noexcept {
  std::size_t n_keypoints = keypoint_measurements.cols();
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(n_keypoints);

  for (std::size_t i = 0u; i < n_keypoints; ++i) {
    keypoints.emplace_back(std::move(
      cv::KeyPoint ((keypoint_measurements)(0, i),  // x
        (keypoint_measurements)(1, i),               // y
        0,
        (keypoint_orientations_rad)(0, i)*180/M_PI,  // angle
        (keypoint_scores)(0, i),                    // response
        (keypoint_scales)(0, i),                      // octave
        (keypoint_ids)(0, i)                      // track id
        )
    ));
  }

  return keypoints;
}

}  // namespace feature_tracking_pipelines
