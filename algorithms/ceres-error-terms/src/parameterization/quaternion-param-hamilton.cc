#include <ceres-error-terms/parameterization/quaternion-param-hamilton.h>

#include <glog/logging.h>

#include <maplab-common/pose_types.h>

namespace ceres_error_terms {

bool HamiltonQuaternionParameterization::Plus(
    const double* x, const double* delta, double* x_plus_delta) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(delta);
  CHECK_NOTNULL(x_plus_delta);

  Eigen::Map<const Eigen::Vector3d> vector_delta(delta);
  double square_norm_delta = vector_delta.squaredNorm();
  if (square_norm_delta > 1.0) {
    // This delta is too large -- not a valid error quaternion.
    square_norm_delta = 1.0;
  }
  if (square_norm_delta > 0.0) {
    Eigen::Quaterniond delta_quat(
        sqrt(1.0 - 0.25 * square_norm_delta), 0.5 * vector_delta[0],
        0.5 * vector_delta[1], 0.5 * vector_delta[2]);
    delta_quat.normalize();

    const pose::Quaternion quaternion_delta(delta_quat);

    const Eigen::Map<const Eigen::Quaterniond> quaternion_x(x);
    Eigen::Map<Eigen::Quaterniond> quaternion_x_plus_delta(x_plus_delta);
    quaternion_x_plus_delta =
        common::positiveQuaternionProductHamilton(
            quaternion_delta, pose::Quaternion(quaternion_x))
            .toImplementation();
    quaternion_x_plus_delta.normalize();
    CHECK_GE(quaternion_x_plus_delta.w(), 0.);
  } else {
    memcpy(x_plus_delta, x, 4 * sizeof(*x));
  }
  return true;
}

bool HamiltonQuaternionParameterization::ComputeJacobian(
    const double* x, double* jacobian) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(jacobian);
  const Eigen::Map<const Eigen::Quaterniond> quat_x(x);

  Eigen::Quaterniond quat_x_copy = quat_x;
  if (quat_x_copy.w() < 0.) {
    quat_x_copy.coeffs() = -quat_x_copy.coeffs();
  }
  CHECK_GE(quat_x_copy.w(), 0);

  // Jacobian for Hamilton [w,x,y,z] convention multiplication
  // and JPL [x,y,z,w] convention memory layout
  //
  // 80-chars convention violated to keep readability
  jacobian[0] = quat_x_copy.w() * 0.5;
  jacobian[1] = quat_x_copy.z() * 0.5;
  jacobian[2] = -quat_x_copy.y() * 0.5;  // NOLINT
  jacobian[3] = -quat_x_copy.z() * 0.5;
  jacobian[4] = quat_x_copy.w() * 0.5;
  jacobian[5] = quat_x_copy.x() * 0.5;  // NOLINT
  jacobian[6] = quat_x_copy.y() * 0.5;
  jacobian[7] = -quat_x_copy.x() * 0.5;
  jacobian[8] = quat_x_copy.w() * 0.5;  // NOLINT
  jacobian[9] = -quat_x_copy.x() * 0.5;
  jacobian[10] = -quat_x_copy.y() * 0.5;
  jacobian[11] = -quat_x_copy.z() * 0.5;  // NOLINT

  return true;
}

bool HamiltonYawOnlyQuaternionParameterization::Plus(
    const double* x, const double* delta, double* x_plus_delta) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(delta);
  CHECK_NOTNULL(x_plus_delta);

  double delta_yaw = delta[0];
  double square_norm_delta = delta_yaw * delta_yaw;
  if (square_norm_delta > 1.0) {
    // This delta is too large -- not a valid error quaternion.
    square_norm_delta = 1.0;
  }
  if (square_norm_delta > 0.0) {
    Eigen::Quaterniond tmp(
        sqrt(1.0 - 0.25 * square_norm_delta), 0, 0, 0.5 * delta_yaw);
    tmp.normalize();

    const pose::Quaternion quaternion_delta(tmp);

    const Eigen::Map<const Eigen::Quaterniond> quaternion_x(x);
    Eigen::Map<Eigen::Quaterniond> quaternion_x_plus_delta(x_plus_delta);
    quaternion_x_plus_delta =
        common::signedQuaternionProductHamilton(
            quaternion_delta, pose::Quaternion(quaternion_x))
            .toImplementation();
    quaternion_x_plus_delta.normalize();

    if (quaternion_x_plus_delta.w() < 0.) {
      quaternion_x_plus_delta.coeffs() = -quaternion_x_plus_delta.coeffs();
    }
    CHECK_GE(quaternion_x_plus_delta.w(), 0.);
  } else {
    memcpy(x_plus_delta, x, 4 * sizeof(*x));
  }
  return true;
}

bool HamiltonYawOnlyQuaternionParameterization::ComputeJacobian(
    const double* x, double* jacobian) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(jacobian);
  const Eigen::Map<const Eigen::Quaterniond> quat_x(x);

  Eigen::Quaterniond quat_x_copy = quat_x;
  if (quat_x_copy.w() < 0.) {
    quat_x_copy.coeffs() = -quat_x_copy.coeffs();
  }
  CHECK_GE(quat_x_copy.w(), 0);

  // Jacobian for Hamilton [w,x,y,z] convention multiplication
  // and JPL [x,y,z,w] convention memory layout
  //
  // 4x1 Jacobian, mapping from Quaternion ambient space to yaw only tangent
  // space.
  jacobian[0] = -quat_x_copy.y() * 0.5;
  jacobian[1] = quat_x_copy.x() * 0.5;
  jacobian[2] = quat_x_copy.w() * 0.5;
  jacobian[3] = -quat_x_copy.z() * 0.5;

  return true;
}

}  // namespace ceres_error_terms
