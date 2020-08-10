#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_SIFT_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_SIFT_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "feature-tracking-pipelines/feature-describer-base.h"

namespace feature_tracking_pipelines {

struct FeatureDescriberSiftSettings {
};

FeatureDescriberSiftSettings InitFeatureDescriberSiftSettingsFromGFlags() {
  FeatureDescriberSiftSettings settings;

  /*
  settings.rotation_invariant = false;
  settings.scale_invariant = true;
  settings.Sift_pattern_scale =  22.0f;
  settings.orb_pyramid_levels =  1u;
  */
  return settings;
}

class FeatureDescriberSift : public FeatureDescriberBase {
 public:
  explicit FeatureDescriberSift(const FeatureDescriberSiftSettings& settings);
  virtual ~FeatureDescriberSift() = default;

  virtual void describeFeatures(
      const cv::Mat& image, KeyframeFeatures* keyframe);

  virtual bool hasNonDescribableFeatures(
      const Eigen::Matrix2Xd& keypoint_measurements,
      const Eigen::RowVectorXd& keypoint_scales,
      std::vector<size_t>* non_describable_indices);

  virtual std::string getDescriptorName() const {
    return "sift";
  }

 private:
  std::vector<cv::KeyPoint> rawToCvKeyPoint(
      const Eigen::Matrix2Xd& keypoint_measurements,
      const Eigen::RowVectorXd& keypoint_scales,
      const Eigen::RowVectorXd& keypoint_scores,
      const Eigen::RowVectorXi& keypoint_ids,
      const Eigen::RowVectorXd& keypoint_orientations_rad) const noexcept;

  void updateKeyframeWithDescriptors(KeyframeFeatures* keyframe, 
      cv::Mat& descriptors_cv, 
      const std::vector<cv::KeyPoint>& keypoints_cv);

  const FeatureDescriberSiftSettings settings_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_SIFT_H_
