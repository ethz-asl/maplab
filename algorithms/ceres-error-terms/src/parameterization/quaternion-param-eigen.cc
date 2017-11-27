#include <ceres-error-terms/parameterization/quaternion-param-eigen.h>
#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

bool EigenQuaternionParameterization::Plus(
    const double* x, const double* delta, double* x_plus_delta) const {
  const Eigen::Map<const Eigen::Vector4d> q_current(x);
  const Eigen::Map<const Eigen::Vector3d> delta_current(delta);
  Eigen::Map<Eigen::Quaterniond> q_next(x_plus_delta);

  Eigen::Quaterniond q_temp;
  common::eigen_quaternion_helpers::Plus(q_current, delta_current, &q_temp);
  q_next = q_temp;
  return true;
}

bool EigenQuaternionParameterization::ComputeJacobian(
    const double* x, double* jacobian) const {
  //  Eigen::Map<Jacobian> J(jacobian);
  //  J.setZero();
  //  J(0, 0) = 1.0;
  //  J(1, 1) = 1.0;
  //  J(2, 2) = 1.0;
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(jacobian);
  const Eigen::Map<const Eigen::Quaterniond> quat_x(x);

  // Jacobian for Hamilton [w,x,y,z] convention multiplication
  // and JPL [x,y,z,w] convention memory layout
  //
  // 80-chars convention violated to keep readability
  jacobian[0] = quat_x.w() * 0.5; jacobian[1] = quat_x.z() * 0.5; jacobian[2] = -quat_x.y() * 0.5;  // NOLINT
  jacobian[3] = -quat_x.z() * 0.5; jacobian[4] = quat_x.w() * 0.5; jacobian[5] = quat_x.x() * 0.5;  // NOLINT
  jacobian[6] = quat_x.y() * 0.5; jacobian[7] = -quat_x.x() * 0.5; jacobian[8] = quat_x.w() * 0.5;  // NOLINT
  jacobian[9] = -quat_x.x() * 0.5; jacobian[10] = -quat_x.y() * 0.5; jacobian[11] = -quat_x.z() * 0.5;  // NOLINT

  return true;
}

bool EigenQuaternionParameterization::Minus(
    const double* x, const double* y, double* x_minus_y) const {
  const Eigen::Quaterniond p_current(x);
  const Eigen::Quaterniond q_current(y);
  Eigen::Map<Eigen::Vector3d> difference(x_minus_y);

  Eigen::Vector3d difference_temp;
  common::eigen_quaternion_helpers::Minus(
      p_current, q_current, &difference_temp);
  difference = difference_temp;
  return true;
}

bool EigenQuaternionParameterization::ComputeLiftJacobian(
    const double* x, double* jacobian) const {
  //  Eigen::Map<LiftJacobian> J(jacobian);
  //  J.setZero();
  //  J(0, 0) = 1.0;
  //  J(1, 1) = 1.0;
  //  J(2, 2) = 1.0;

  CHECK_NOTNULL(x);
  CHECK_NOTNULL(jacobian);
  const Eigen::Map<const Eigen::Quaterniond> quat_x(x);

  // Jacobian for Hamilton [w,x,y,z] convention multiplication
  // and JPL [x,y,z,w] convention memory layout
  //
  // 80-chars convention violated to keep readability
  jacobian[0] = quat_x.w() * 2.0; jacobian[4] = quat_x.z() * 2.0; jacobian[8] = -quat_x.y() * 2.0;  // NOLINT
  jacobian[1] = -quat_x.z() * 2.0; jacobian[5] = quat_x.w() * 2.0; jacobian[9] = quat_x.x() * 2.0;  // NOLINT
  jacobian[2] = quat_x.y() * 2.0; jacobian[6] = -quat_x.x() * 2.0; jacobian[10] = quat_x.w() * 2.0;  // NOLINT
  jacobian[3] = -quat_x.x() * 2.0; jacobian[7] = -quat_x.y() * 2.0; jacobian[11] = -quat_x.z() * 2.0;  // NOLINT

  return true;
}

}  // namespace ceres_error_terms
