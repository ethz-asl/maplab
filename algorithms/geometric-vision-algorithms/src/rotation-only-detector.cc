#include "geometric-vision/rotation-only-detector.h"

#include <aslam/matcher/match-helpers.h>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>

namespace geometric_vision {
bool RotationOnlyDetector::isRotationOnlyMotion(
    const aslam::VisualNFrame::Ptr& nframe_kp1,
    const aslam::VisualNFrame::Ptr& nframe_k,
    const aslam::FrameToFrameMatchesList& matches_kp1_k,
    const aslam::Quaternion& q_Bkp1_Bk, const bool remove_outliers) {
  const aslam::NCamera::Ptr& ncamera_ptr = nframe_k->getNCameraShared();
  CHECK(ncamera_ptr);
  const size_t num_cameras = ncamera_ptr->getNumCameras();
  CHECK_GT(num_cameras, 0u);
  CHECK_EQ(matches_kp1_k.size(), num_cameras);
  // Ensure that the camera_rig does not change from viewpoint k to viewpoint
  // k+1.
  CHECK_EQ(ncamera_ptr.get(), nframe_kp1->getNCameraShared().get());

  // Iterate through the camera rig.
  deviation_from_parallel_bearing_vector_ = 0.0;
  for (size_t camera_index = 0; camera_index < num_cameras; ++camera_index) {
    const aslam::Transformation& T_C_B = nframe_k->get_T_C_B(camera_index);
    const aslam::Quaternion q_Ckp1_Ck =
        T_C_B.getRotation() * q_Bkp1_Bk * T_C_B.inverse().getRotation();

    // Remove outliers if requested.
    opengv::bearingVectors_t bearing_vectors_kp1, bearing_vectors_k;
    size_t num_matches = 0;
    if (remove_outliers) {
      aslam::FrameToFrameMatches inlier_matches_kp1_k;
      getInlierMatches(
          nframe_kp1, nframe_k, matches_kp1_k, q_Ckp1_Ck, camera_index,
          &inlier_matches_kp1_k);
      aslam::getBearingVectorsFromMatches(
          nframe_kp1->getFrame(camera_index), nframe_k->getFrame(camera_index),
          inlier_matches_kp1_k, &bearing_vectors_kp1, &bearing_vectors_k);
      num_matches = inlier_matches_kp1_k.size();
    } else {
      aslam::getBearingVectorsFromMatches(
          nframe_kp1->getFrame(camera_index), nframe_k->getFrame(camera_index),
          matches_kp1_k[camera_index], &bearing_vectors_kp1,
          &bearing_vectors_k);
      num_matches = matches_kp1_k[camera_index].size();
    }

    // Iterate through the matches of the camera pair.
    double deviation_from_parallel_bearing_vector_current_camera = 0.0;
    CHECK_GT(num_matches, 0u);
    for (size_t match_index = 0; match_index < num_matches; ++match_index) {
      // Equations (27) and (28) of referenced paper.
      deviation_from_parallel_bearing_vector_current_camera +=
          (bearing_vectors_kp1[match_index].normalized() -
           (q_Ckp1_Ck.getRotationMatrix() * bearing_vectors_k[match_index])
               .normalized())
              .norm();
    }
    // Ensure that the threshold is independent of the number of matches.
    deviation_from_parallel_bearing_vector_ +=
        deviation_from_parallel_bearing_vector_current_camera /
        static_cast<double>(num_matches);
  }
  // Ensure the threshold is independent of the number of cameras.
  deviation_from_parallel_bearing_vector_ /= static_cast<double>(num_cameras);

  // Classification based on value at current(k+1) and last(k) timestep to avoid
  // flickering.
  // TODO(hitimo): Use more robust classification (ML: unsupervised learning, 2
  // clusters ...).
  const bool is_only_rotated_kp1 = is_only_rotated_k_ &&
                                   deviation_from_parallel_bearing_vector_ <
                                       motion_classification_threshold_;
  is_only_rotated_k_ = deviation_from_parallel_bearing_vector_ <
                       motion_classification_threshold_;
  return is_only_rotated_kp1;
}

void RotationOnlyDetector::getInlierMatches(
    const aslam::VisualNFrame::Ptr& nframe_kp1,
    const aslam::VisualNFrame::Ptr& nframe_k,
    const aslam::FrameToFrameMatchesList& matches_kp1_k,
    const aslam::Quaternion& q_Ckp1_Ck, const size_t camera_index,
    aslam::FrameToFrameMatches* inlier_matches_kp1_k) const {
  CHECK_NOTNULL(inlier_matches_kp1_k);
  // Get bearing vectors.
  opengv::bearingVectors_t bearing_vectors_kp1, bearing_vectors_k;
  aslam::getBearingVectorsFromMatches(
      nframe_kp1->getFrame(camera_index), nframe_k->getFrame(camera_index),
      matches_kp1_k[camera_index], &bearing_vectors_kp1, &bearing_vectors_k);
  // Solve 2-point RANSAC problem.
  using opengv::relative_pose::CentralRelativeAdapter;
  CentralRelativeAdapter adapter(
      bearing_vectors_kp1, bearing_vectors_k, q_Ckp1_Ck.getRotationMatrix());

  using opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
  std::shared_ptr<TranslationOnlySacProblem> twopt_problem(
      new TranslationOnlySacProblem(adapter));

  opengv::sac::Ransac<TranslationOnlySacProblem> ransac;
  ransac.sac_model_ = twopt_problem;
  ransac.threshold_ = kTwoPointRansacThreshold;
  ransac.max_iterations_ = kTwoPointRansacMaxIterations;
  ransac.computeModel();

  // Find the inliers.
  for (size_t inlier_index = 0; inlier_index < ransac.inliers_.size();
       ++inlier_index) {
    inlier_matches_kp1_k->emplace_back(
        matches_kp1_k[camera_index][ransac.inliers_[inlier_index]]);
  }
}
}  // namespace geometric_vision
