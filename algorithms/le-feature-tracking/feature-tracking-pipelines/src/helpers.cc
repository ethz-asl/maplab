#include "feature-tracking-pipelines/helpers.h"

#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace feature_tracking_pipelines {

void KeyframeFeaturesToCvPoints(
    const Eigen::Matrix2Xd& keypoint_measurements,
    std::vector<cv::Point2f>* cv_features) {
  CHECK_NOTNULL(cv_features)->clear();
  for (size_t i = 0u; i < static_cast<size_t>(keypoint_measurements.cols());
       ++i) {
    cv_features->emplace_back(
        cv::Point2f(keypoint_measurements(0, i), keypoint_measurements(1, i)));
  }
}

void CvKeypointsToKeyframeFeatures(
    const std::vector<cv::Point2f>& cv_features,
    Eigen::Matrix2Xd* keypoint_measurements) {
  CHECK_NOTNULL(keypoint_measurements);

  keypoint_measurements->resize(Eigen::NoChange, cv_features.size());
  for (size_t i = 0u; i < cv_features.size(); ++i) {
    (*keypoint_measurements)(0, i) = cv_features[i].x;
    (*keypoint_measurements)(1, i) = cv_features[i].y;
  }
}

static void resizeKeyframeFeatures(const std::size_t n_keypoints,
    KeyframeFeatures* keyframe) {
  CHECK_NOTNULL(keyframe);
  keyframe->keypoint_measurements.resize(Eigen::NoChange, n_keypoints);
  keyframe->keypoint_scales.resize(Eigen::NoChange, n_keypoints);
  keyframe->keypoint_orientations_rad.resize(Eigen::NoChange, n_keypoints);
  keyframe->keypoint_scores.resize(Eigen::NoChange, n_keypoints);
  keyframe->keypoint_sizes.resize(Eigen::NoChange, n_keypoints);
  keyframe->keypoint_track_ids.resize(Eigen::NoChange, n_keypoints);
}

static float deg2rad(const float deg) {
  return deg * M_PI / 180;
}

static float rad2deg(const float rad) {
  return rad * 180 / M_PI;
}

void CvKeypointsToKeyframeFeatures(
    const std::vector<cv::KeyPoint> &keypoints_cv,
    KeyframeFeatures* keyframe) {
  CHECK_NOTNULL(keyframe);

  const std::size_t n_keypoints = keypoints_cv.size();
  resizeKeyframeFeatures(n_keypoints, keyframe);

  for (std::size_t idx = 0u; idx < n_keypoints; ++idx) {
    const cv::KeyPoint &keypoint = keypoints_cv[idx];
    keyframe->keypoint_measurements(0, idx) = keypoint.pt.x;
    keyframe->keypoint_measurements(1, idx) = keypoint.pt.y;
    keyframe->keypoint_scales(0, idx) = keypoint.octave;
    keyframe->keypoint_orientations_rad(0, idx) = deg2rad(keypoint.angle);
    keyframe->keypoint_scores(0, idx) = keypoint.response;
    keyframe->keypoint_sizes(0, idx) = keypoint.size;
    keyframe->keypoint_track_ids(0, idx) = keypoint.class_id;
  }
}

void KeyframeFeaturesToCvPoints(
    const KeyframeFeatures& keyframe,
    std::vector<cv::KeyPoint>* keypoints_cv) {
  CHECK_NOTNULL(keypoints_cv);
  std::size_t n_keypoints = keyframe.keypoint_measurements.cols();
  keypoints_cv->reserve(n_keypoints);
 
  for (std::size_t i = 0u; i < n_keypoints; ++i) {
    keypoints_cv->emplace_back(std::move(
      cv::KeyPoint (
        keyframe.keypoint_measurements(0, i),  // x
        keyframe.keypoint_measurements(1, i),               // y
        keyframe.keypoint_sizes(0, i),                    // size
        rad2deg(keyframe.keypoint_orientations_rad(0, i)),  // angle
        keyframe.keypoint_scores(0, i),                    // response
        keyframe.keypoint_scales(0, i),                      // octave
        keyframe.keypoint_track_ids(0, i)                      // track id
        )
    ));
  }
}

}  // namespace feature_tracking_pipelines
