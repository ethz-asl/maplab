#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_BRISK_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_BRISK_H_
#include <memory>
#include <string>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "feature-tracking-pipelines/feature-detector-base.h"
#include "feature-tracking-pipelines/flags.h"

namespace feature_tracking_pipelines {

struct FeatureDetectorBriskSettings {
  int num_pyramid_levels = 3;
  double uniformity_radius = 5.0;
  double absolute_threshold = 35.0;
};

FeatureDetectorBriskSettings InitFeatureDetectorBriskSettingsFromGFlags() {
  FeatureDetectorBriskSettings settings;
  settings.num_pyramid_levels = FLAGS_NEW_feature_tracker_num_pyramid_levels;
  settings.uniformity_radius =
      FLAGS_NEW_feature_tracker_brisk_uniformity_radius;
  settings.absolute_threshold =
      FLAGS_NEW_feature_tracker_detector_brisk_absolute_threshold;
  return settings;
}

class FeatureDetectorBrisk : public FeatureDetectorBase {
 public:
  explicit FeatureDetectorBrisk(const FeatureDetectorBriskSettings& settings);
  virtual ~FeatureDetectorBrisk() {}

  virtual void detectFeatures(
      const cv::Mat& image, size_t num_features, const cv::Mat& mask,
      KeyframeFeatures *keyframe);

  virtual std::string getDetectorName() const {
    return "brisk";
  }

 private:
  std::shared_ptr<cv::Feature2D> detector_;

  const int kMaxDetection = 1000;
  const FeatureDetectorBriskSettings settings_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_BRISK_H_
