#ifndef ASLAM_CALIBRATION_TARGET_ALGORITHMS_H
#define ASLAM_CALIBRATION_TARGET_ALGORITHMS_H

#include <aslam/cameras/camera.h>
#include <aslam/common/pose-types.h>

#include "aslam/calibration/target-observation.h"

namespace aslam {
namespace calibration {

// Estimates the target transform from the given corner observations.
bool estimateTargetTransformation(
    const TargetObservation& target_observation,
    const aslam::Camera::ConstPtr& camera_ptr, aslam::Transformation* T_G_C);

// Estimates the target transform from the given corner observations (with
// additional options).
bool estimateTargetTransformation(
    const TargetObservation& target_observation,
    const aslam::Camera::ConstPtr& camera_ptr, aslam::Transformation* T_G_C,
    const bool run_nonlinear_refinement, const double ransac_pixel_sigma,
    const int ransac_max_iters);

}  // namespace calibration
}  // namespace aslam

#endif  // ASLAM_CALIBRATION_TARGET_ALGORITHMS_H
