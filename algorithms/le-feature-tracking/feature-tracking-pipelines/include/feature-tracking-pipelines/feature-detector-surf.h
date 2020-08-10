#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_SURF_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_SURF_H_
#include <memory>
#include <string>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "feature-tracking-pipelines/feature-detector-base.h"
#include "feature-tracking-pipelines/flags.h"

namespace feature_tracking_pipelines {

struct FeatureDetectorSurfSettings {
  double hessian_threshold = 100;
  int n_octaves = 4;
  int octave_layers = 3;
  bool extended = false;
  bool upright = false;
};

FeatureDetectorSurfSettings InitFeatureDetectorSurfSettingsFromGFlags() {
  FeatureDetectorSurfSettings settings;
  settings.hessian_threshold = FLAGS_NEW_feature_tracker_surf_hessian_threshold;
  settings.n_octaves = FLAGS_NEW_feature_tracker_surf_n_octaves;
  settings.octave_layers = FLAGS_NEW_feature_tracker_surf_octave_layers;
  settings.extended = FLAGS_NEW_feature_tracker_surf_extended;
  settings.upright = FLAGS_NEW_feature_tracker_surf_upright;
  return settings;
}

class FeatureDetectorSurf : public FeatureDetectorBase {
 public:
  explicit FeatureDetectorSurf(const FeatureDetectorSurfSettings& settings);
  virtual ~FeatureDetectorSurf() {}

  virtual void detectFeatures(
      const cv::Mat& image, size_t num_features, const cv::Mat& mask,
      KeyframeFeatures *keyframe);

  virtual std::string getDetectorName() const {
    return "surf";
  }

 private:
  cv::Ptr<cv::xfeatures2d::SURF> detector_;

  const int kMaxDetection = 1000;
  const FeatureDetectorSurfSettings settings_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_SURF_H_
