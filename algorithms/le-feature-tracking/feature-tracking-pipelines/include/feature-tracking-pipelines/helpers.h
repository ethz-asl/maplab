#ifndef FEATURE_TRACKING_PIPELINES_HELPERS_H_
#define FEATURE_TRACKING_PIPELINES_HELPERS_H_

#include <mutex>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

#include "feature-tracking-pipelines/keyframe-features.h"

namespace feature_tracking_pipelines {

void KeyframeFeaturesToCvPoints(
    const Eigen::Matrix2Xd& keypoint_measurements,
    std::vector<cv::Point2f>* cv_features);
void CvKeypointsToKeyframeFeatures(
    const std::vector<cv::Point2f>& cv_features,
    Eigen::Matrix2Xd* keypoint_measurements);

void CvKeypointsToKeyframeFeatures(
    const std::vector<cv::KeyPoint>& keypoints_cv,
    KeyframeFeatures* keyframe);
void KeyframeFeaturesToCvPoints(
    const KeyframeFeatures& keyframe,
    std::vector<cv::KeyPoint>* keypoints_cv);

template <typename ContainerType>
void GetElementIndicesInVector(
    const ContainerType& input_vector,
    const typename ContainerType::value_type& value_to_copy,
    std::vector<size_t>* indices) {
  CHECK_NOTNULL(indices)->clear();
  indices->reserve(input_vector.size());
  for (size_t index = 0u; index < input_vector.size(); ++index) {
    if (input_vector[index] == value_to_copy) {
      indices->emplace_back(index);
    }
  }
}

class TrackIdProvider {
 public:
  TrackIdProvider() : counter_(0u) {}

  size_t getId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return counter_++;
  }

  // Get a range of IDs: [start, end)
  std::pair</*start*/ size_t, /*end*/ size_t> getIds(size_t count) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t start = counter_;
    counter_ += count;
    return std::pair</*start*/ size_t, /*end*/ size_t>(start, counter_);
  }

 private:
  mutable std::mutex mutex_;
  mutable size_t counter_;
};

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_HELPERS_H_
