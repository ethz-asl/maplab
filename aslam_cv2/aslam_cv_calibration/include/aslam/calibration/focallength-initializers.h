#ifndef ASLAM_CALIBRATION_CAMERA_INITIALIZER_H
#define ASLAM_CALIBRATION_CAMERA_INITIALIZER_H

#include <vector>

#include <Eigen/Core>

#include "aslam/calibration/target-observation.h"
#include "aslam/calibration/helpers.h"

namespace aslam {
namespace calibration {

// Initializes the intrinsics vector based on one views of a calibration targets.
// On success it returns true. These functions are based on functions from Lionel Heng and
// the excellent camodocal: https://github.com/hengli/camodocal.
// This algorithm can be used with high distortion lenses.
//
// C. Hughes, P. Denny, M. Glavin, and E. Jones,
// Equidistant Fish-Eye Calibration and Rectification by Vanishing Point
// Extraction, PAMI 2010
// Find circles from rows of chessboard corners, and for each pair
// of circles, find vanishing points: v1 and v2.
// f = ||v1 - v2|| / PI;
bool initFocalLengthVanishingPoints(
    const std::vector<TargetObservation::Ptr>& observations,
    Eigen::VectorXd* intrinsics);

// Initializes the intrinsics vector based on one view of a gridded calibration target.
// On success it returns true. These functions are based on functions from Lionel Heng
// and the excellent camodocal https://github.com/hengli/camodocal.
//
// Z. Zhang
// A Flexible New Technique for Camera Calibration,
// Extraction, PAMI 2000
// Intrinsics estimation with image of absolute conic;
bool initFocalLengthAbsoluteConic(
  const std::vector<TargetObservation::Ptr>& observations,
  Eigen::VectorXd* intrinsics);

}  // namespace calibration
}  // namespace aslam

#include "internal/focallength-initializers-traits-inl.h"

#endif  // ASLAM_CALIBRATION_CAMERA_INITIALIZER_H
