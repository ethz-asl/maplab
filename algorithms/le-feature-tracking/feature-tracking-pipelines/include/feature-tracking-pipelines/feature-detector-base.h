#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_BASE_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_BASE_H_
#include <string>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <feature-tracking-pipelines/keyframe-features.h>

namespace feature_tracking_pipelines {

class FeatureDetectorBase {
 public:
  FeatureDetectorBase() = default;
  virtual ~FeatureDetectorBase() = default;

  virtual void detectFeatures(
      const cv::Mat& image, size_t num_features, const cv::Mat& mask,
      KeyframeFeatures* keyframe) = 0;

  virtual std::string getDetectorName() const = 0;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_BASE_H_
