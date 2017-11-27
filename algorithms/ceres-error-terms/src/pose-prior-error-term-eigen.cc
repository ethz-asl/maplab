#include <ceres-error-terms/pose-prior-error-term-eigen.h>
#include <glog/logging.h>

#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

bool EigenPosePriorErrorTerm::Evaluate(
    double const* const* parameters, double* residuals,
    double** jacobians) const {
  CHECK_NOTNULL(parameters);
  CHECK_NOTNULL(residuals);

  Eigen::Map<const Eigen::Quaterniond> orientation_current(parameters[kIdxQ]);
  Eigen::Map<const Eigen::Vector3d> position_current(parameters[kIdxP]);

  Eigen::Vector3d delta_orientation;
  common::eigen_quaternion_helpers::Minus(
      orientation_current, orientation_prior_, &delta_orientation);

  // Calculate residuals.
  Eigen::Map<Eigen::Matrix<double, poseblocks::kResidualSize, 1>>
      residual_vector(residuals);
  residual_vector.head<3>() = delta_orientation;
  residual_vector.tail<3>() = position_current - position_prior_;

  // Weight according to the square root of information matrix.
  residual_vector = sqrt_information_matrix_ * residual_vector;

  if (jacobians) {
    // Jacobian w.r.t. current pose.
    if (jacobians[kIdxQ]) {
      Eigen::Map<OrientationJacobian> J(jacobians[kIdxQ]);

      // Jacobian w.r.t elements of quaternion parameterization
      EigenQuaternionParameterization::LiftJacobian lift_jacobian;
      EigenQuaternionParameterization parameterization;
      parameterization.ComputeLiftJacobian(
          orientation_current.coeffs().data(), lift_jacobian.data());

      J.setZero();
      J.block<3, 4>(0, 0) = (Eigen::Matrix3d::Identity() +
                             0.5 * common::skew(delta_orientation)) *
                            lift_jacobian;

      // Add the weighting according to the square root of information matrix.
      J = sqrt_information_matrix_ * J;
    }
    if (jacobians[kIdxP]) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxP]);

      J.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();

      // Add the weighting according to the square root of information matrix.
      J = sqrt_information_matrix_ * J;
    }
  }

  return true;
}

}  // namespace ceres_error_terms
