#include "feature-tracking-pipelines/keyframe-features.h"

#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <maplab-common/eigen-helpers.h>

namespace feature_tracking_pipelines {

void Print(const KeyframeFeatures& features, std::ostream& ss) {
  ss << std::endl;
  ss << "Detector name: \t" << features.keypoint_detector_name << std::endl;
  ss << "Descriptor name: \t" << features.keypoint_descriptor_name << std::endl;
  ss << "Tracker name: \t" << features.keypoint_tracker_name << std::endl;
  ss.flush();

  ss << "Keypoint measurements: \n"
     << features.keypoint_measurements << std::endl;
  ss.flush();
  ss << "Keypoint scales: \n" << features.keypoint_scales << std::endl;
  ss << "Keypoint orientations (rad): \n"
     << features.keypoint_orientations_rad << std::endl;
  ss << "Keypoint scores: \n" << features.keypoint_scores << std::endl;

  ss.flush();
  ss << "Feature Track IDs: \n" << features.keypoint_track_ids << std::endl;

  ss << "Descriptor: \n"
     << "  # rows: " << features.keypoint_descriptors.rows() << std::endl
     << "  # cols: " << features.keypoint_descriptors.cols() << std::endl;
}

void RemoveKeypoints(
    const std::vector<size_t>& indices_to_remove,
    KeyframeFeatures* keyframe_features) {
  CHECK_NOTNULL(keyframe_features);

  if (indices_to_remove.empty()) {
    return;
  }
  CHECK_GT(keyframe_features->keypoint_measurements.cols(), 0);

  // Keypoints are always expected to be present.
  common::RemoveColsFromEigen(
      indices_to_remove, &keyframe_features->keypoint_measurements);
  const int num_keypoints = keyframe_features->keypoint_measurements.cols();

  // The other fields are optional and only shrinked if they are present.
  if (keyframe_features->keypoint_scales.cols() > 0) {
    common::RemoveColsFromEigen(
        indices_to_remove, &keyframe_features->keypoint_scales);
    CHECK_EQ(keyframe_features->keypoint_scales.cols(), num_keypoints);
  }
  if (keyframe_features->keypoint_orientations_rad.cols() > 0) {
    common::RemoveColsFromEigen(
        indices_to_remove, &keyframe_features->keypoint_orientations_rad);
    CHECK_EQ(
        keyframe_features->keypoint_orientations_rad.cols(), num_keypoints);
  }
  if (keyframe_features->keypoint_scores.cols() > 0) {
    common::RemoveColsFromEigen(
        indices_to_remove, &keyframe_features->keypoint_scores);
    CHECK_EQ(keyframe_features->keypoint_scores.cols(), num_keypoints);
  }
  if (keyframe_features->keypoint_descriptors.cols() > 0) {
    common::RemoveColsFromEigen(
        indices_to_remove, &keyframe_features->keypoint_descriptors);
    CHECK_EQ(keyframe_features->keypoint_descriptors.cols(), num_keypoints);
  }
  if (keyframe_features->keypoint_track_ids.cols() > 0) {
    common::RemoveColsFromEigen(
        indices_to_remove, &keyframe_features->keypoint_track_ids);
    CHECK_EQ(keyframe_features->keypoint_track_ids.cols(), num_keypoints);
  }
}

void AppendKeypoints(
    const KeyframeFeatures& data_to_append, KeyframeFeatures* keyframe_merged) {
  CHECK_NOTNULL(keyframe_merged);

  common::AppendColsRightToEigen(
      data_to_append.keypoint_measurements,
      &keyframe_merged->keypoint_measurements);
  const int num_keypoints = keyframe_merged->keypoint_measurements.cols();

  // The other fields are optional and are only added if they already exist in
  // the destination container.
  common::AppendColsRightToEigen(
      data_to_append.keypoint_scales, &keyframe_merged->keypoint_scales);
  if (keyframe_merged->keypoint_scales.cols() > 0) {
    CHECK_EQ(keyframe_merged->keypoint_scales.cols(), num_keypoints);
  }

  common::AppendColsRightToEigen(
      data_to_append.keypoint_orientations_rad,
      &keyframe_merged->keypoint_orientations_rad);
  if (keyframe_merged->keypoint_orientations_rad.cols() > 0) {
    CHECK_EQ(keyframe_merged->keypoint_orientations_rad.cols(), num_keypoints);
  }

  common::AppendColsRightToEigen(
      data_to_append.keypoint_scores, &keyframe_merged->keypoint_scores);
  if (keyframe_merged->keypoint_scores.cols() > 0) {
    CHECK_EQ(keyframe_merged->keypoint_scores.cols(), num_keypoints);
  }

  common::AppendColsRightToEigen(
      data_to_append.keypoint_track_ids, &keyframe_merged->keypoint_track_ids);
  if (keyframe_merged->keypoint_track_ids.cols() > 0) {
    CHECK_EQ(keyframe_merged->keypoint_track_ids.cols(), num_keypoints);
  }

  common::AppendColsRightToEigen(
      data_to_append.keypoint_descriptors,
      &keyframe_merged->keypoint_descriptors);
  if (keyframe_merged->keypoint_descriptors.cols() > 0) {
    CHECK_EQ(keyframe_merged->keypoint_descriptors.cols(), num_keypoints);
  }

  common::AppendColsRightToEigen(
      data_to_append.keypoint_sizes, &keyframe_merged->keypoint_sizes);
  //TODO(lbern): something is broken here
  /*
  if (keyframe_merged->keypoint_sizes.cols() > 0) {
    CHECK_EQ(keyframe_merged->keypoint_sizes.cols(), num_keypoints);
  }*/

  common::AppendColsRightToEigen(
      data_to_append.keypoint_measurement_uncertainties, 
      &keyframe_merged->keypoint_measurement_uncertainties);
  if (keyframe_merged->keypoint_measurement_uncertainties.cols() > 0) {
    CHECK_EQ(keyframe_merged->keypoint_measurement_uncertainties.cols(), num_keypoints);
  }
}

void ApplyKeypointFeaturesToVisualNFrame(
    const std::vector<KeyframeFeatures>& keypoint_features,
    aslam::VisualNFrame* nframe) {
  CHECK_NOTNULL(nframe);
  CHECK_EQ(keypoint_features.size(), nframe->getNumFrames());
  for (size_t i = 0; i < nframe->getNumFrames(); ++i) {
    aslam::VisualFrame& frame = *CHECK_NOTNULL(nframe->getFrameShared(i).get());
    CHECK_EQ(keypoint_features[i].frame_idx, i);

    frame.setKeypointMeasurements(keypoint_features[i].keypoint_measurements);
    frame.setKeypointMeasurementUncertainties(
        keypoint_features[i].keypoint_measurement_uncertainties);
    frame.setKeypointScales(keypoint_features[i].keypoint_scales);
    frame.setKeypointOrientations(
        keypoint_features[i].keypoint_orientations_rad);
    frame.setKeypointScores(keypoint_features[i].keypoint_scores);
    frame.setTrackIds(keypoint_features[i].keypoint_track_ids);
    frame.setDescriptors(keypoint_features[i].keypoint_descriptors);
    frame.setKeypointVectors(keypoint_features[i].keypoint_vectors);
  }
}

void GetKeypointIndicesSortedByTrackId(
    const KeyframeFeatures& keyframe_features,
    std::vector<size_t>* keypoint_indices_sorted_by_track_id) {
  CHECK_NOTNULL(keypoint_indices_sorted_by_track_id)->clear();
  keypoint_indices_sorted_by_track_id->reserve(
      keyframe_features.keypoint_track_ids.size());

  *keypoint_indices_sorted_by_track_id = common::SortPermutationEigenVector(
      keyframe_features.keypoint_track_ids, std::less<int>());

  // Also move all untracked features (negative track ids) to the far right.
  std::partition(
      keypoint_indices_sorted_by_track_id->begin(),
      keypoint_indices_sorted_by_track_id->end(), [&](const int& kp_idx) {
        CHECK_LT(kp_idx, keyframe_features.keypoint_track_ids.cols());
        return keyframe_features.keypoint_track_ids(0, kp_idx) >= 0;
      });
}

}  // namespace feature_tracking_pipelines
