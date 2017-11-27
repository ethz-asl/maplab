#include <ceres-error-terms/position-error-term.h>
#include <glog/logging.h>

#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

bool PositionErrorTerm::Evaluate(
    double const* const* parameters, double* residuals,
    double** jacobians) const {
  CHECK_NOTNULL(parameters);
  CHECK_NOTNULL(residuals);

  Eigen::Map<const Eigen::Vector3d> position_current(
      parameters[kIdxPose] + poseblocks::kOrientationBlockSize);

  // Calculate residuals.
  Eigen::Map<Eigen::Matrix<double, positionblocks::kResidualSize, 1> >
      residual_vector(residuals);
  residual_vector = position_current - position_prior_;

  // Weight according to the square root of information matrix.
  residual_vector = sqrt_information_matrix_ * residual_vector;

  if (jacobians) {
    // Jacobian w.r.t. current pose.
    if (jacobians[kIdxPose]) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxPose]);

      J.setZero();
      J.block<3, 3>(0, 4) = Eigen::Matrix3d::Identity();

      // Add the weighting according to the square root of information matrix.
      J = sqrt_information_matrix_ * J;
    }
  }
  return true;
}

}  // namespace ceres_error_terms
