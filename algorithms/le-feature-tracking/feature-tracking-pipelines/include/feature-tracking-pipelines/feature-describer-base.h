#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_BASE_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_BASE_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include "feature-tracking-pipelines/keyframe-features.h"


namespace feature_tracking_pipelines {

class FeatureDescriberBase {
 public:
  FeatureDescriberBase() = default;
  virtual ~FeatureDescriberBase() {}

  virtual void describeFeatures(
      const cv::Mat& image, KeyframeFeatures* keyframe) = 0;

  virtual bool hasNonDescribableFeatures(
      const Eigen::Matrix2Xd& keypoint_measurements,
      const Eigen::RowVectorXd& keypoint_scales,
      std::vector<size_t>* non_describable_indices) {
    CHECK_NOTNULL(non_describable_indices)->clear();
    return false;
  }

  virtual std::string getDescriptorName() const = 0;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_BASE_H_
