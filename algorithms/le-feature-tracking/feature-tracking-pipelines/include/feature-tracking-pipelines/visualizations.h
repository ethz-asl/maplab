#ifndef FEATURE_TRACKING_PIPELINES_VISUALIZATIONS_H_
#define FEATURE_TRACKING_PIPELINES_VISUALIZATIONS_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>

#include "feature-tracking-pipelines/keyframe-features.h"

namespace feature_tracking_pipelines {

struct RansacSettings {};

bool performFrameToFrameFeatureRansac(
    const KeyframeFeatures& keypoint_features_current,
    const KeyframeFeatures& keypoint_features_previous,
    const RansacSettings& settings, aslam::Transformation* T_Ccurr_Cprev,
    std::vector<int>* inlier_feature_indices) {
  CHECK(keypoint_features_current.IsValid());
  CHECK(keypoint_features_previous.IsValid());
  CHECK_NOTNULL(T_Ccurr_Cprev);
  CHECK_NOTNULL(inlier_feature_indices);

  LOG(FATAL) << "IMPL.";
  return false;
}

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_VISUALIZATIONS_H_
