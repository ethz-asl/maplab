#include "aslam/geometric-vision/match-outlier-rejection-twopt.h"

#include <memory>

#include <aslam/common/pose-types.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/matcher/match-helpers.h>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/RotationOnlySacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>

namespace aslam {
namespace geometric_vision {

bool rejectOutlierFeatureMatchesTranslationRotationSAC(
    const aslam::VisualFrame& frame_kp1, const aslam::VisualFrame& frame_k,
    const aslam::Quaternion& q_Ckp1_Ck, int descriptor_type,
    const aslam::FrameToFrameMatches& matches_kp1_k,
    bool fix_random_seed, double ransac_threshold, size_t ransac_max_iterations,
    aslam::FrameToFrameMatches* inlier_matches_kp1_k,
    aslam::FrameToFrameMatches* outlier_matches_kp1_k) {

  BearingVectors bearing_vectors_kp1;
  BearingVectors bearing_vectors_k;
  aslam::getBearingVectorsFromMatches(
      frame_kp1, frame_k, matches_kp1_k, descriptor_type,
      &bearing_vectors_kp1, &bearing_vectors_k);

  std::unordered_set<int> inlier_indices;
  const bool success = rejectOutlierFeatureMatchesTranslationRotationSAC(
        bearing_vectors_kp1, bearing_vectors_k, q_Ckp1_Ck, fix_random_seed,
        ransac_threshold, ransac_max_iterations, &inlier_indices);

  // Remove the outliers from the matches list.
  int match_index = 0;
  for (const aslam::FrameToFrameMatch& match : matches_kp1_k) {
    if (inlier_indices.count(match_index)) {
      inlier_matches_kp1_k->emplace_back(match);
    } else {
      outlier_matches_kp1_k->emplace_back(match);
    }
    ++match_index;
  }
  CHECK_EQ(inlier_matches_kp1_k->size() + outlier_matches_kp1_k->size(),
           matches_kp1_k.size());
  return success;
}

bool rejectOutlierFeatureMatchesTranslationRotationSAC(
    const BearingVectors& bearing_vectors_kp1,
    const BearingVectors& bearing_vectors_k,
    const aslam::Quaternion& q_Ckp1_Ck, bool fix_random_seed,
    double ransac_threshold, size_t ransac_max_iterations,
    std::unordered_set<int>* inlier_indices) {
  CHECK_GT(ransac_threshold, 0.0);
  CHECK_GT(ransac_max_iterations, 0u);
  CHECK_NOTNULL(inlier_indices)->clear();
  CHECK_EQ(bearing_vectors_kp1.size(), bearing_vectors_kp1.size());

  // Handle the case with too few matches to distinguish between out-/inliers.
  static constexpr size_t kMinKeypointCorrespondences = 6u;
  if (bearing_vectors_kp1.size() < kMinKeypointCorrespondences) {
    VLOG(1) << "Too few matches to run RANSAC.";
    inlier_indices->clear();  // Treat all as outliers.
    return false;
  }

  using opengv::relative_pose::CentralRelativeAdapter;
  CentralRelativeAdapter adapter(bearing_vectors_kp1, bearing_vectors_k,
                                 q_Ckp1_Ck.getRotationMatrix());

  typedef opengv::sac_problems::relative_pose::RotationOnlySacProblem RotationOnlySacProblem;
  std::shared_ptr<RotationOnlySacProblem> rotation_sac_problem(
      new RotationOnlySacProblem(adapter, !fix_random_seed));

  opengv::sac::Ransac<RotationOnlySacProblem> rotation_ransac;
  rotation_ransac.sac_model_ = rotation_sac_problem;
  rotation_ransac.threshold_ = ransac_threshold;
  rotation_ransac.max_iterations_ = ransac_max_iterations;
  rotation_ransac.computeModel();

  typedef opengv::sac_problems::relative_pose::TranslationOnlySacProblem TranslationOnlySacProblem;
  std::shared_ptr<TranslationOnlySacProblem> translation_sac_problem(
      new TranslationOnlySacProblem(adapter, !fix_random_seed));
  opengv::sac::Ransac<TranslationOnlySacProblem> translation_ransac;
  translation_ransac.sac_model_ = translation_sac_problem;
  translation_ransac.threshold_ = ransac_threshold;
  translation_ransac.max_iterations_ = ransac_max_iterations;
  translation_ransac.computeModel();

  // Take the union of both inlier sets as final inlier set.
  // This is done because translation only ransac erroneously discards many
  // matches in the center of the image as outliers but it is reliable
  // closer to the boundary of the image. On the contrary, rotation only
  // ransac erroneously discards many matches close to the border of the image
  // but it correctly classifies matches in the center of the image.
  inlier_indices->insert(rotation_ransac.inliers_.begin(),
                         rotation_ransac.inliers_.end());
  inlier_indices->insert(translation_ransac.inliers_.begin(),
                         translation_ransac.inliers_.end());

  if (inlier_indices->size() < kMinKeypointCorrespondences) {
    VLOG(1) << "Too few inliers to reliably classify outlier matches.";
    inlier_indices->clear();  // Treat all as outliers.
    return false;
  }

  return true;
}

}  // namespace geometric_vision
}  // namespace aslam
