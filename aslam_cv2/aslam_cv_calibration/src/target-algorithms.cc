#include "aslam/calibration/target-algorithms.h"

#include <Eigen/Core>
#include <aslam/geometric-vision/pnp-pose-estimator.h>

namespace aslam {
namespace calibration {

bool estimateTargetTransformation(
    const TargetObservation& target_observation,
    const aslam::Camera::ConstPtr& camera_ptr, aslam::Transformation* T_G_C) {
  CHECK(camera_ptr);
  CHECK_NOTNULL(T_G_C);
  constexpr bool kRunNonlinearRefinement = true;
  constexpr double kRansacPixelSigma = 1.0;
  constexpr int kRansacMaxIters = 200;
  return estimateTargetTransformation(
      target_observation, camera_ptr, T_G_C, kRunNonlinearRefinement,
      kRansacPixelSigma, kRansacMaxIters);
}

bool estimateTargetTransformation(
    const TargetObservation& target_observation,
    const aslam::Camera::ConstPtr& camera_ptr, aslam::Transformation* T_G_C,
    const bool run_nonlinear_refinement, const double ransac_pixel_sigma,
    const int ransac_max_iters) {
  CHECK(camera_ptr);
  CHECK_GT(ransac_pixel_sigma, 0.0);
  CHECK_GT(ransac_max_iters, 0);
  CHECK_NOTNULL(T_G_C);
  const Eigen::Matrix2Xd& observed_corners =
      target_observation.getObservedCorners();
  // Corner positions in target coordinates (global frame).
  const Eigen::Matrix3Xd corner_positions_G =
      target_observation.getCorrespondingTargetPoints();
  aslam::geometric_vision::PnpPoseEstimator pnp(run_nonlinear_refinement);
  std::vector<int> inliers;
  int num_iters = 0;
  bool pnp_success = pnp.absolutePoseRansacPinholeCam(
      observed_corners, corner_positions_G, ransac_pixel_sigma,
      ransac_max_iters, camera_ptr, T_G_C, &inliers, &num_iters);
  if (pnp_success) {
    VLOG(4) << "Found " << inliers.size() << "/" << observed_corners.cols()
            << " inliers in" << num_iters << " iterations.";
  } else {
    LOG(WARNING) << "Target transformation estimation failed.";
  }
  return pnp_success;
}

}  // namespace calibration
}  // namespace aslam
