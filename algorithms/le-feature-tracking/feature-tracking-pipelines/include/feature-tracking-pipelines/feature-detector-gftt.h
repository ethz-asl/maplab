#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_GFTT_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_GFTT_H_

#include <string>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "feature-tracking-pipelines/feature-detector-base.h"
#include "feature-tracking-pipelines/flags.h"

namespace feature_tracking_pipelines {

class FeatureDetectorGFTT : public FeatureDetectorBase {
 public:
  FeatureDetectorGFTT();
  virtual ~FeatureDetectorGFTT() {}

  virtual void detectFeatures(
      const cv::Mat& image, size_t num_features, const cv::Mat& mask,
      KeyframeFeatures *keyframe);

  virtual std::string getDetectorName() const {
    return "gfft";
  }
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_GFTT_H_
