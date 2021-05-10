#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_SURF_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_SURF_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "feature-tracking-pipelines/feature-describer-base.h"
#include "feature-tracking-pipelines/flags.h"

namespace feature_tracking_pipelines {

struct FeatureDescriberSurfSettings {
  double hessian_threshold = 100;
  int n_octaves = 4;
  int octave_layers = 3;
  bool extended = false;
  bool upright = false;
};

FeatureDescriberSurfSettings InitFeatureDescriberSurfSettingsFromGFlags() {
  FeatureDescriberSurfSettings settings;
  settings.hessian_threshold = FLAGS_NEW_feature_tracker_surf_hessian_threshold;
  settings.n_octaves = FLAGS_NEW_feature_tracker_surf_n_octaves;
  settings.octave_layers = FLAGS_NEW_feature_tracker_surf_octave_layers;
  settings.extended = FLAGS_NEW_feature_tracker_surf_extended;
  settings.upright = FLAGS_NEW_feature_tracker_surf_upright;
  return settings;
}

class FeatureDescriberSurf : public FeatureDescriberBase {
 public:
  explicit FeatureDescriberSurf(const FeatureDescriberSurfSettings& settings);
  virtual ~FeatureDescriberSurf() = default;

  virtual void describeFeatures(
      const cv::Mat& image, KeyframeFeatures* keyframe);

  virtual bool hasNonDescribableFeatures(
      const Eigen::Matrix2Xd& keypoint_measurements,
      const Eigen::RowVectorXd& keypoint_scales,
      std::vector<size_t>* non_describable_indices);

  virtual std::string getDescriptorName() const {
    return "surf";
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

  const FeatureDescriberSurfSettings settings_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DESCRIBER_SURF_H_
