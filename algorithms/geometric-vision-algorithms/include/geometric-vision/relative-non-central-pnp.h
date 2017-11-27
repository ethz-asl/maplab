#ifndef GEOMETRIC_VISION_RELATIVE_NON_CENTRAL_PNP_H_
#define GEOMETRIC_VISION_RELATIVE_NON_CENTRAL_PNP_H_

#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <aslam/matcher/match.h>
#include <glog/logging.h>
#include <maplab-common/pose_types.h>
#include <opengv/relative_pose/NoncentralRelativeMultiAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/MultiRansac.hpp>
#include <opengv/sac_problems/relative_pose/MultiNoncentralRelativePoseSacProblem.hpp>

namespace geometric_vision {
typedef opengv::sac_problems::relative_pose::
    MultiNoncentralRelativePoseSacProblem MultiNonCentralRelativeSacPnp;

class RelativeNonCentralPnp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit RelativeNonCentralPnp(bool run_nonlinear_refinement)
      : random_seed_(true),
        solver_(MultiNonCentralRelativeSacPnp::Algorithm::SEVENTEENPT),
        minimum_required_matches_(setup(solver_)),
        run_nonlinear_refinement_(run_nonlinear_refinement) {}

  /// Set the random seed and solver.
  RelativeNonCentralPnp(
      bool run_nonlinear_refinement,
      MultiNonCentralRelativeSacPnp::Algorithm solver, bool random_seed)
      : random_seed_(random_seed),
        solver_(solver),
        minimum_required_matches_(setup(solver_)),
        run_nonlinear_refinement_(run_nonlinear_refinement) {}

  int getMinRequiredMatches() const {
    return minimum_required_matches_;
  }

  /// \brief Cast to pinhole type, calculate ransac threshold
  /// and call relative non-central PnP.
  /// @param[in] nframe_0 Contains camera_rig and observations
  /// from viewpoint 0.
  /// @param[in] nframe_1 Contains camera_rig and observations
  /// from viewpoint 1.
  /// @param[in] matches_0_1 Matches from camera i in viewpoint 0
  /// to camera i in viewpoint 1 where i=[0,num_cameras).
  /// E.g. matches[camera_idx][0].first contains the index of the first
  /// keypoint of the first nframe/viewpoint.
  /// @param[in] pixel_sigma Image noise standard deviation [pixels].
  /// @param[in] ransac_max_iters Maximum number of RANSAC iterations.
  /// @param[out] T_I0_I1 Estimated RELATIVE camera translation and camera
  /// rotation transformation matrix between the two viewpoints
  /// (body frames).
  /// @param[out] num_inliers Number of inliers calculated by RANSAC.
  /// @return Success of relative pose ransac.
  bool computePinhole(
      const aslam::VisualNFrame::ConstPtr& nframe_0,
      const aslam::VisualNFrame::ConstPtr& nframe_1,
      const aslam::FrameToFrameMatchesList& matches_0_1, double pixel_sigma,
      size_t ransac_max_iters, pose::Transformation* T_I0_I1,
      size_t* num_inliers) const;

  /// \brief Compute relative non-central PNP using openGV and run RANSAC.
  bool compute(
      const aslam::VisualNFrame::ConstPtr& nframe_0,
      const aslam::VisualNFrame::ConstPtr& nframe_1,
      const aslam::FrameToFrameMatchesList& matches_0_1,
      double ransac_threshold, size_t ransac_max_iters,
      pose::Transformation* T_I0_I1, size_t* num_inliers) const;

 private:
  // TODO(hitimo): Check if these values are actually correct.
  static size_t setup(MultiNonCentralRelativeSacPnp::Algorithm solver) {
    switch (solver) {
      case MultiNonCentralRelativeSacPnp::Algorithm::SIXPT:
        return 6;
        break;
      case MultiNonCentralRelativeSacPnp::Algorithm::GE:
        return 2;
        break;
      case MultiNonCentralRelativeSacPnp::Algorithm::SEVENTEENPT:
        return 17;
        break;
      default:
        LOG(FATAL) << "Unsupported PnP-solver.";
    }
    return 0;
  }
  // Whether to let RANSAC pick a timestamp-based random seed or not. If false,
  // a seed can be set with srand().
  const bool random_seed_;

  // Solve problem with only one camera?
  static constexpr bool kAsCentral = false;

  /// PnP solver method.
  const MultiNonCentralRelativeSacPnp::Algorithm solver_;

  /// Minimum number of matches required to solve the problem.
  const int minimum_required_matches_;

  static constexpr double kRansacInlierRatioThreshold = 0.7;

  /// Run nonlinear refinement over all inliers.
  const bool run_nonlinear_refinement_;
};
}  // namespace geometric_vision

#endif  // GEOMETRIC_VISION_RELATIVE_NON_CENTRAL_PNP_H_
