#include "feature-tracking-pipelines/feature-detector-orb.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "feature-tracking-pipelines/helpers.h"

namespace feature_tracking_pipelines {

FeatureDetectorOrbSettings InitFeatureDetectorOrbSettingsFromGFlags() {
  FeatureDetectorOrbSettings settings;
  settings.num_pyramid_levels = FLAGS_NEW_feature_tracker_num_pyramid_levels;
  settings.pyramid_scale_factor =
      FLAGS_NEW_feature_tracker_pyramid_scale_decimation;
  settings.edge_threshold =
      FLAGS_NEW_feature_tracker_detector_orb_edge_threshold;
  settings.fast_threshold =
      FLAGS_NEW_feature_tracker_detector_orb_fast_threshold;
  return settings;
}

FeatureDetectorOrb::FeatureDetectorOrb(
    const FeatureDetectorOrbSettings& settings)
    : settings_(settings) {
  orb_detector_ = cv::ORB::create(
      /*nfeatures=*/500,  // Will be overwritten during detect.
      settings.pyramid_scale_factor, settings.num_pyramid_levels,
      settings.edge_threshold,
      /*firstLevel=*/0,
      /*WTA_K=*/2,  // Unused, as we only use the detector.
      static_cast<int>(settings.score_type),
      /*patchSize=*/31,  // Unused, as we only use the detector.
      /*fastThreshold=*/20);
}

void FeatureDetectorOrb::detectFeatures(
    const cv::Mat& image, size_t num_features, const cv::Mat& mask,
    KeyframeFeatures *keyframe) {
  CHECK_NOTNULL(keyframe);

  orb_detector_->setMaxFeatures(num_features);

  std::vector<cv::KeyPoint> keypoints_cv;
  orb_detector_->detect(image, keypoints_cv, mask);
  CvKeypointsToKeyframeFeatures(keypoints_cv, keyframe);

  // TODO(schneith): Add subpixel refinement.
  LOG(ERROR) << "Check if subpixel acc: \n" << (keyframe->keypoint_measurements);
}

}  // namespace feature_tracking_pipelines
