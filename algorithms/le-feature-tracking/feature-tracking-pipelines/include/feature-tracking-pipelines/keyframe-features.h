#ifndef FEATURE_TRACKING_PIPELINES_KEYFRAME_FEATURES_H_
#define FEATURE_TRACKING_PIPELINES_KEYFRAME_FEATURES_H_

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <maplab-common/eigen-helpers.h>
#include <opencv2/core/core.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace feature_tracking_pipelines {

struct KeyframeFeatures {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  size_t frame_idx;

  // Raw keypoint measurements.
  Eigen::Matrix2Xd keypoint_measurements;
  Eigen::RowVectorXd keypoint_scales;
  Eigen::RowVectorXd keypoint_orientations_rad;
  Eigen::RowVectorXd keypoint_scores;  // higher, better.
  Eigen::RowVectorXd keypoint_measurement_uncertainties;
  Eigen::RowVectorXd keypoint_sizes;
  Eigen::Matrix3Xd keypoint_vectors;

  // Temporal tracking information.
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
      keypoint_descriptors;

  // Temporal tracking information.
  Eigen::RowVectorXi keypoint_track_ids;

  // Informaton of the sources.
  std::string keypoint_detector_name = "unset";
  std::string keypoint_descriptor_name = "unset";
  std::string keypoint_tracker_name = "unset";
};

void Print(const KeyframeFeatures& features, std::ostream& ss);

void RemoveKeypoints(
    const std::vector<size_t>& indices_to_remove,
    KeyframeFeatures* keyframe_features);
void AppendKeypoints(
    const KeyframeFeatures& data_to_append, KeyframeFeatures* keyframe_merged);

void ApplyLidarKeypointFeaturesToVisualNFrame(
    const std::vector<KeyframeFeatures>& keypoint_features,
    aslam::VisualNFrame* nframe);

void ApplyKeypointFeaturesToVisualNFrame(
    const std::vector<KeyframeFeatures>& keypoint_features,
    aslam::VisualNFrame* nframe);

void GetKeypointIndicesSortedByTrackId(
    const KeyframeFeatures& keyframe_features,
    std::vector<size_t>* keypoint_indices_sorted_by_track_id);

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_KEYFRAME_FEATURES_H_
