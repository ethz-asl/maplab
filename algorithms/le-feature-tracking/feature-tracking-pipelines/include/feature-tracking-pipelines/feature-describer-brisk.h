#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_BRISK_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_BRISK_H_

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <brisk/brisk.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <string>
#include <vector>

#include "feature-tracking-pipelines/feature-describer-base.h"

namespace feature_tracking_pipelines {

struct FeatureDescriberBriskSettings {
  bool rotation_invariant;
  bool scale_invariant;
  // size_t octaves;
  // double uniformity_radius;
  // double absolute_threshold;
  // size_t max_number_of_keypoints;
};

FeatureDescriberBriskSettings InitFeatureDescriberBriskSettingsFromGFlags() {
  FeatureDescriberBriskSettings settings;

  // TODO(mariusbr): create gflags
  settings.rotation_invariant = false;
  settings.scale_invariant = true;
  return settings;
}

class FeatureDescriberBrisk : public FeatureDescriberBase {
 public:
  explicit FeatureDescriberBrisk(const FeatureDescriberBriskSettings& settings);
  virtual ~FeatureDescriberBrisk() {}

  virtual void describeFeatures(
      const cv::Mat& image, KeyframeFeatures* keyframe);

  virtual bool hasNonDescribableFeatures(
      const Eigen::Matrix2Xd& keypoint_measurements,
      const Eigen::RowVectorXd& keypoint_scales,
      std::vector<size_t>* non_describable_indices);

  virtual std::string getDescriptorName() const {
    return "brisk";
  }

 private:
  void updateKeyframeWithDescriptors(
      KeyframeFeatures* keyframe, cv::Mat& descriptors_cv,
      const std::vector<cv::KeyPoint>& keypoints_cv);

  const FeatureDescriberBriskSettings settings_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_BRISK_H_
