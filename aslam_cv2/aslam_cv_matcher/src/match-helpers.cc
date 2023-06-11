#include "aslam/matcher/match-helpers.h"

#include <aslam/cameras/camera.h>
#include <aslam/common/stl-helpers.h>
#include <Eigen/Core>
#include <glog/logging.h>

namespace aslam {

/// Return the normalized bearing vectors for a list of single camera matches.
void getBearingVectorsFromMatches(
    const VisualFrame& frame_kp1, const VisualFrame& frame_k,
    const FrameToFrameMatches& matches_kp1_k, int descriptor_type,
    Aligned<std::vector, Eigen::Vector3d>* bearing_vectors_kp1,
    Aligned<std::vector, Eigen::Vector3d>* bearing_vectors_k) {
  CHECK_NOTNULL(bearing_vectors_kp1);
  CHECK_NOTNULL(bearing_vectors_k);

  const size_t num_matches = matches_kp1_k.size();
  std::vector<size_t> keypoint_indices_kp1;
  keypoint_indices_kp1.reserve(num_matches);
  std::vector<size_t> keypoint_indices_k;
  keypoint_indices_k.reserve(num_matches);

  keypoint_indices_kp1.reserve(matches_kp1_k.size());
  keypoint_indices_k.reserve(matches_kp1_k.size());
  for (const FrameToFrameMatch& match_kp1_k : matches_kp1_k) {
    keypoint_indices_kp1.emplace_back(match_kp1_k.first);
    keypoint_indices_k.emplace_back(match_kp1_k.second);
  }

  std::vector<unsigned char> success;
  aslam::common::convertEigenToStlVector(frame_kp1.getNormalizedBearingVectors(
      keypoint_indices_kp1, descriptor_type, &success), bearing_vectors_kp1);
  aslam::common::convertEigenToStlVector(frame_k.getNormalizedBearingVectors(
      keypoint_indices_k, descriptor_type, &success), bearing_vectors_k);
}


void predictKeypointsByRotation(const VisualFrame& frame_k,
                                const aslam::Quaternion& q_Ckp1_Ck,
                                Eigen::Matrix2Xd* predicted_keypoints_kp1,
                                std::vector<unsigned char>* prediction_success,
                                int descriptor_type) {
  CHECK_NOTNULL(predicted_keypoints_kp1);
  CHECK_NOTNULL(prediction_success)->clear();
  CHECK(frame_k.hasKeypointMeasurements());
  CHECK(frame_k.hasDescriptorType(descriptor_type));

  const aslam::Camera& camera =
      *CHECK_NOTNULL(frame_k.getCameraGeometry().get());

  const Eigen::Block<const Eigen::Matrix2Xd> keypoints_k = 
      frame_k.getKeypointMeasurementsOfType(descriptor_type);

  if (keypoints_k.cols() == 0u) {
    return;
  }

  // Early exit for identity rotation.
  if (std::abs(q_Ckp1_Ck.w() - 1.0) < 1e-8) {
    *predicted_keypoints_kp1 = keypoints_k;
    prediction_success->resize(predicted_keypoints_kp1->size(), true);
  }

  // Backproject the keypoints to bearing vectors.
  Eigen::Matrix3Xd bearing_vectors_k;
  camera.backProject3Vectorized(keypoints_k, &bearing_vectors_k,
                                prediction_success);
  CHECK_EQ(static_cast<int>(prediction_success->size()), bearing_vectors_k.cols());
  CHECK_EQ(keypoints_k.cols(), bearing_vectors_k.cols());

  // Rotate the bearing vectors into the keypoints_kp1 coordinates.
  const Eigen::Matrix3Xd bearing_vectors_kp1 = q_Ckp1_Ck.rotateVectorized(bearing_vectors_k);

  // Project the bearing vectors to the keypoints_kp1.
  std::vector<ProjectionResult> projection_results;
  camera.project3Vectorized(bearing_vectors_kp1, predicted_keypoints_kp1, &projection_results);
  CHECK_EQ(predicted_keypoints_kp1->cols(), bearing_vectors_k.cols());
  CHECK_EQ(static_cast<int>(projection_results.size()), bearing_vectors_k.cols());

  // Set the success based on the backprojection and projection results and output the initial
  // unrotated keypoint for failed predictions.
  CHECK_EQ(keypoints_k.cols(), predicted_keypoints_kp1->cols());

  for (size_t idx = 0u; idx < projection_results.size(); ++idx) {
    (*prediction_success)[idx] = (*prediction_success)[idx] &&
                                 projection_results[idx].isKeypointVisible();

    // Set the initial keypoint location for failed predictions.
    if (!(*prediction_success)[idx]) {
      predicted_keypoints_kp1->col(idx) = keypoints_k.col(idx);
    }
  }
}

}  // namespace aslam
