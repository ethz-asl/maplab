#ifndef FEATURE_TRACKING_FEATURE_DETECTION_EXTRACTION_H_
#define FEATURE_TRACKING_FEATURE_DETECTION_EXTRACTION_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera.h>
#include <aslam/frames/visual-frame.h>
#include <opencv2/features2d/features2d.hpp>

#include "feature-tracking/feature-tracking-types.h"

namespace feature_tracking {

class FeatureDetectorExtractor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FeatureDetectorExtractor(
      const aslam::Camera& camera,
      const FeatureTrackingExtractorSettings& extractor_settings,
      const FeatureTrackingDetectorSettings& detector_settings);

  void detectAndExtractFeatures(aslam::VisualFrame* frame) const;
  cv::Ptr<cv::DescriptorExtractor> getExtractorPtr() const;

 private:
  void initialize();

  const aslam::Camera& camera_;
  const FeatureTrackingExtractorSettings extractor_settings_;
  const FeatureTrackingDetectorSettings detector_settings_;

  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
};

}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_FEATURE_DETECTION_EXTRACTION_H_
