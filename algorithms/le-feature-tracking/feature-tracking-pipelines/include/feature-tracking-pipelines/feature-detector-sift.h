#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_SIFT_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_SIFT_H_
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

struct FeatureDetectorSiftSettings {
  int n_features = 0;
  int n_octave_layers = 3;
  double contrast_threshold = 0.04;
  double edge_threshold = 10;
  double sigma = 1.6;
};

FeatureDetectorSiftSettings InitFeatureDetectorSiftSettingsFromGFlags() {
  FeatureDetectorSiftSettings settings;
  settings.n_features = FLAGS_NEW_feature_tracker_sift_n_features;
  settings.n_octave_layers = FLAGS_NEW_feature_tracker_sift_n_octave_layers;
  settings.contrast_threshold = FLAGS_NEW_feature_tracker_sift_contrast_threshold;
  settings.edge_threshold = FLAGS_NEW_feature_tracker_sift_edge_threshold;
  settings.sigma = FLAGS_NEW_feature_tracker_sift_sigma;
  return settings;
}

class FeatureDetectorSift : public FeatureDetectorBase {
 public:
  explicit FeatureDetectorSift(const FeatureDetectorSiftSettings& settings);
  virtual ~FeatureDetectorSift() {}

  virtual void detectFeatures(
      const cv::Mat& image, size_t num_features, const cv::Mat& mask,
      KeyframeFeatures *keyframe);

  virtual std::string getDetectorName() const {
    return "sift";
  }

 private:
  cv::Ptr<cv::xfeatures2d::SIFT> detector_;

  const int kMaxDetection = 1000;
  const FeatureDetectorSiftSettings settings_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_SIFT_H_
