#ifndef FEATURE_TRACKING_FEATURE_DETECTION_EXTRACTION_H_
#define FEATURE_TRACKING_FEATURE_DETECTION_EXTRACTION_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera.h>
#include <aslam/common/memory.h>
#include <aslam/common/occupancy-grid.h>
#include <aslam/frames/visual-nframe.h>
#include <aslam/matcher/match.h>
#include <aslam/tracker/feature-tracker.h>
#include <gflags/gflags.h>
#include <opencv2/video/tracking.hpp>

#include "feature-tracking/feature-tracking-types.h"

namespace feature_tracking {

class FeatureDetectorExtractor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FeatureDetectorExtractor(const aslam::Camera& camera);
  void detectAndExtractFeatures(aslam::VisualFrame* frame) const;
  cv::Ptr<cv::DescriptorExtractor> getExtractorPtr() const;

 private:
  void initialize();

  /// \brief  A simple non-maximum suppression algorithm that erases keypoints
  ///         in a specified radius around a queried keypoint if their response
  ///         is below ratio_threshold times the response of the queried
  ///         keypoint.
  /// @param[in] radius:  Radius around queried keypoint that is searched for
  ///                     other keypoints to potentially suppress them.
  /// @param[in] ratio_threshold: Suppress keypoints if their response is lower
  ///                             than this threshold times the response of
  ///                             the queried keypoint. A lower threshold should
  ///                             make the suppression more robust. However,
  ///                             this will result in more keypoints remaining
  ///                             close to each other.
  /// @param[out] keypoints:  A subset of the keypoints will be erased according
  ///                         to above criteria.
  const aslam::Camera& camera_;

  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;

 public:
  // Descriptor extractor settings are stored in this struct.
  const SweFeatureTrackingExtractorSettings extractor_settings_;
  // Feature detector settings are stored in this struct.
  const SweFeatureTrackingDetectorSettings detector_settings_;
};

}  // namespace feature_tracking

#endif  // FEATURE_TRACKING_FEATURE_DETECTION_EXTRACTION_H_
