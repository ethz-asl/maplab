#ifndef ASLAM_MATCH_OUTLIER_REJECTION_TWOPT_H_
#define ASLAM_MATCH_OUTLIER_REJECTION_TWOPT_H_

#include <unordered_set>
#include <Eigen/Core>
#include <glog/logging.h>

#include <aslam/common/pose-types.h>
#include <aslam/matcher/match.h>

namespace aslam {
class VisualFrame;

namespace geometric_vision {
using BearingVectors =
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

// RANSAC threshold can be defined as:  1 - cos(max_ray_disparity_angle).
bool rejectOutlierFeatureMatchesTranslationRotationSAC(
    const aslam::VisualFrame& frame_kp1, const aslam::VisualFrame& frame_k,
    const aslam::Quaternion& q_Ckp1_Ck, int descriptor_type,
    const aslam::FrameToFrameMatches& matches_kp1_k,
    bool fix_random_seed, double ransac_threshold, size_t ransac_max_iterations,
    aslam::FrameToFrameMatches* inlier_matches_kp1_k,
    aslam::FrameToFrameMatches* outlier_matches_kp1_k);

bool rejectOutlierFeatureMatchesTranslationRotationSAC(
    const BearingVectors& bearing_vectors_kp1,
    const BearingVectors& bearing_vectors_k,
    const aslam::Quaternion& q_Ckp1_Ck, bool fix_random_seed,
    double ransac_threshold, size_t ransac_max_iterations,
    std::unordered_set<int>* inlier_indices);

}  // namespace geometric_vision

}  // namespace aslam
#endif  // ASLAM_MATCH_OUTLIER_REJECTION_TWOPT_H_
