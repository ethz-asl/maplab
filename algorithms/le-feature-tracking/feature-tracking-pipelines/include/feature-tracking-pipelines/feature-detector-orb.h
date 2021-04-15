#ifndef FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_ORB_H_
#define FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_ORB_H_

#include <string>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "feature-tracking-pipelines/feature-detector-base.h"
#include "feature-tracking-pipelines/flags.h"

namespace feature_tracking_pipelines {

struct FeatureDetectorOrbSettings {
  // The number of pyramid levels. The smallest level will have linear size
  // equal to input_image_linear_size/pow(scaleFactor, nlevels).
  int num_pyramid_levels = 3;

  // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the
  // classical pyramid, where each next level has 4x less pixels than the
  // previous, but such a big scale factor will degrade feature matching scores
  // dramatically. On the other hand, too close to 1 scale factor will mean
  // that to cover certain scale range you will need more pyramid levels and so
  // the speed will suffer.
  float pyramid_scale_factor = 1.5f;

  // This is size of the border where the features are not detected. It should
  // roughly match the descriptor patch size.
  int edge_threshold = 31;

  // Algortihm used to rank features during selection. Written to the score
  // output field.
  enum class ScoreType : int { Harris = 0, Fast = 1 };
  ScoreType score_type = ScoreType::Harris;

  // Lower threshold for FAST selection.
  int fast_threshold = 20;
};

FeatureDetectorOrbSettings InitFeatureDetectorOrbSettingsFromGFlags();

class FeatureDetectorOrb : public FeatureDetectorBase {
 public:
  explicit FeatureDetectorOrb(const FeatureDetectorOrbSettings& settings);
  virtual ~FeatureDetectorOrb() {}

  virtual void detectFeatures(
      const cv::Mat& image, size_t num_features, const cv::Mat& mask,
      KeyframeFeatures *keyframe);

  virtual std::string getDetectorName() const {
    return "orb";
  }

 private:
  const FeatureDetectorOrbSettings settings_;

  cv::Ptr<cv::ORB> orb_detector_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_FEATURE_DETECTOR_ORB_H_
