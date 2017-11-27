#include <ceres-error-terms/parameterization/unit3-param.h>

namespace ceres_error_terms {

bool Unit3Parameterization::Plus(
    const double* x, const double* delta, double* x_plus_delta) const {
  const Eigen::Quaterniond q_current(x);
  Eigen::Vector2d delta_current(delta);
  Eigen::Map<Eigen::Quaterniond> q_next(x_plus_delta);

  Eigen::Quaterniond q_temp;
  Unit3::Plus(q_current, delta_current, &q_temp);
  q_next = q_temp;
  return true;
}

bool Unit3Parameterization::ComputeJacobian(
    const double* x, double* jacobian) const {
  //  Eigen::Map<Jacobian> J(jacobian);
  //  J.setZero();
  //  J(0, 0) = 1.0;
  //  J(1, 1) = 1.0;

  CHECK_NOTNULL(x);
  CHECK_NOTNULL(jacobian);
  const Eigen::Map<const Eigen::Quaterniond> quat_x(x);

  Eigen::Quaterniond quat_x_copy = quat_x;
  if (quat_x_copy.w() < 0.) {
    quat_x_copy.coeffs() = quat_x_copy.coeffs();
  }
  // CHECK_GE(quat_x_copy.w(), 0);

  // Jacobian for Hamilton [w,x,y,z] convention multiplication
  // and JPL [x,y,z,w] convention memory layout
  //
  // 80-chars convention violated to keep readability

  jacobian[0] = quat_x_copy.w() * 0.5; jacobian[1] = -quat_x_copy.z() * 0.5;
  jacobian[2] = quat_x_copy.z() * 0.5; jacobian[3] = quat_x_copy.w() * 0.5;
  jacobian[4] = -quat_x_copy.y() * 0.5; jacobian[5] = quat_x_copy.x() * 0.5;
  jacobian[6] = -quat_x_copy.x() * 0.5; jacobian[7] = -quat_x_copy.y() * 0.5;
  return true;
}

bool Unit3Parameterization::Minus(
    const double* x, const double* y, double* x_minus_y) const {
  const Eigen::Quaterniond unit1_current(x);
  const Eigen::Quaterniond unit2_current(y);
  Eigen::Map<Eigen::Vector2d> difference(x_minus_y);

  Eigen::Vector2d difference_temp;
  Unit3::Minus(unit1_current, unit2_current, &difference_temp);
  difference = difference_temp;
  return true;
}

bool Unit3Parameterization::ComputeLiftJacobian(
    const double* x, double* jacobian) const {
  //  Eigen::Map<LiftJacobian> J(jacobian);
  //  J.setZero();
  //  J(0, 0) = 1.0;
  //  J(1, 1) = 1.0;

  CHECK_NOTNULL(x);
  CHECK_NOTNULL(jacobian);
  const Eigen::Map<const Eigen::Quaterniond> quat_x(x);

  Eigen::Quaterniond quat_x_copy = quat_x;
  if (quat_x_copy.w() < 0.) {
    quat_x_copy.coeffs() = quat_x_copy.coeffs();
  }
  // CHECK_GE(quat_x_copy.w(), 0);

  // Jacobian for Hamilton [w,x,y,z] convention multiplication
  // and JPL [x,y,z,w] convention memory layout
  //
  // 80-chars convention violated to keep readability

  jacobian[0] = quat_x_copy.w() * 2.0; jacobian[4] = -quat_x_copy.z() * 2.0;
  jacobian[1] = quat_x_copy.z() * 2.0; jacobian[5] = quat_x_copy.w() * 2.0;
  jacobian[2] = -quat_x_copy.y() * 2.0; jacobian[6] = quat_x_copy.x() * 2.0;
  jacobian[3] = -quat_x_copy.x() * 2.0; jacobian[7] = -quat_x_copy.y() * 2.0;
  return true;
}

}  // namespace ceres_error_terms
