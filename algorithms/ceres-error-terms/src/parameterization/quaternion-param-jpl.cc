#include "ceres-error-terms/parameterization/quaternion-param-jpl.h"

#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {
namespace {
inline void get_dQuaternionJpl_dTheta(
    const double* q_ptr, double* jacobian_row_major) {
  CHECK_NOTNULL(q_ptr);
  CHECK_NOTNULL(jacobian_row_major);

  const Eigen::Map<const Eigen::Matrix<double, 4, 1>> q(q_ptr);
  CHECK_GE(q(3), 0.);

  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jacobian(
      jacobian_row_major);
  jacobian.setZero();
  jacobian(0, 0) = q(3);
  jacobian(0, 1) = -q(2);
  jacobian(0, 2) = q(1);
  jacobian(1, 0) = q(2);
  jacobian(1, 1) = q(3);
  jacobian(1, 2) = -q(0);
  jacobian(2, 0) = -q(1);
  jacobian(2, 1) = q(0);
  jacobian(2, 2) = q(3);
  jacobian(3, 0) = -q(0);
  jacobian(3, 1) = -q(1);
  jacobian(3, 2) = -q(2);
  jacobian *= 0.5;
}
}  // namespace

bool JplQuaternionParameterization::Plus(
    const double* x, const double* delta, double* x_plus_delta) const {
  CHECK_NOTNULL(x);
  CHECK_NOTNULL(delta);
  CHECK_NOTNULL(x_plus_delta);

  Eigen::Map<const Eigen::Vector3d> rot_vector_delta(delta);
  Eigen::Map<const Eigen::Vector4d> q(x);
  Eigen::Map<Eigen::Vector4d> q_product(x_plus_delta);
  CHECK_GE(q(3), 0.);

  double square_norm_delta = rot_vector_delta.squaredNorm();
  // 0.262rad is ~15deg which keeps the small angle approximation error limited
  // to about 1%.
  static constexpr double kSmallAngleApproxThresholdDoubleSquared =
      (0.262 * 2) * (0.262 * 2);

  Eigen::Vector4d q_delta;
  if (square_norm_delta < kSmallAngleApproxThresholdDoubleSquared) {
    // The delta theta norm is below the threshold so we can use the small
    // angle approximation.
    q_delta << 0.5 * rot_vector_delta, 1.0;
    q_delta.normalize();
  } else {
    const double norm_delta_half = 0.5 * sqrt(square_norm_delta);
    q_delta.head<3>() =
        0.5 * rot_vector_delta / norm_delta_half * std::sin(norm_delta_half);
    q_delta(3) = std::cos(norm_delta_half);
  }
  common::positiveQuaternionProductJPL(q_delta, q, q_product);
  CHECK_GE(q_product(3), 0.);

  return true;
}

bool JplQuaternionParameterization::ComputeJacobian(
    const double* quat, double* jacobian_row_major) const {
  CHECK_NOTNULL(quat);
  CHECK_NOTNULL(jacobian_row_major);
  get_dQuaternionJpl_dTheta(quat, jacobian_row_major);
  return true;
}

bool JplYawQuaternionParameterization::Plus(
    const double* q_A_B_ptr, const double* B_delta_yaw_ptr,
    double* q_A_B_plus_delta_ptr) const {
  CHECK_NOTNULL(q_A_B_ptr);
  CHECK_NOTNULL(B_delta_yaw_ptr);
  CHECK_NOTNULL(q_A_B_plus_delta_ptr);

  const double delta_half = 0.5 * *B_delta_yaw_ptr;
  Eigen::Vector4d q_AhatA_delta;
  q_AhatA_delta << 0.0, 0.0, std::sin(delta_half),
      std::cos(std::abs(delta_half));

  Eigen::Map<const Eigen::Vector4d> q_AB(q_A_B_ptr);
  Eigen::Map<Eigen::Vector4d> q_AB_plus_delta(q_A_B_plus_delta_ptr);
  common::positiveQuaternionProductJPL(q_AhatA_delta, q_AB, q_AB_plus_delta);
  return true;
}

bool JplYawQuaternionParameterization::ComputeJacobian(
    const double* q_A_B_ptr, double* jacobian) const {
  CHECK_NOTNULL(q_A_B_ptr);
  CHECK_NOTNULL(jacobian);
  const Eigen::Map<const Eigen::Quaternion<double>> q_AB(q_A_B_ptr);
  CHECK_GT(q_AB.w(), 0.0);

  jacobian[0] = q_AB.y() * 0.5;
  jacobian[1] = -q_AB.x() * 0.5;
  jacobian[2] = q_AB.w() * 0.5;
  jacobian[3] = -q_AB.z() * 0.5;
  return true;
}

JplRollPitchQuaternionParameterization::JplRollPitchQuaternionParameterization(
    const Eigen::Matrix<double, 4, 1>& q_GM_JPL) {
  CHECK_GE(q_GM_JPL(3, 0), 0.0);
  common::toRotationMatrixJPL(common::quaternionInverseJPL(q_GM_JPL), &R_M_G_);
}

bool JplRollPitchQuaternionParameterization::Plus(
    const double* q_I_M_ptr, const double* G_delta_ptr,
    double* q_I_M_plus_M_delta_ptr) const {
  CHECK_NOTNULL(q_I_M_ptr);
  CHECK_NOTNULL(G_delta_ptr);
  CHECK_NOTNULL(q_I_M_plus_M_delta_ptr);

  // Frames:
  //  G: global, gravity-aligned frame.
  //     rotations around G_[0,0,1] are locked.
  //  M: mission-frame
  //  I: body-fixed frame.

  // q_IM = (q_MI)^-1 = (q_MMrp(R_MG * G_delta) * q_MrpI)^-1
  Eigen::Map<const Eigen::Vector4d> q_I_Mrp(q_I_M_ptr);
  Eigen::Map<const Eigen::Vector2d> G_delta(G_delta_ptr);
  Eigen::Map<Eigen::Vector4d> q_I_M_plus_M_delta(q_I_M_plus_M_delta_ptr);
  CHECK_GE(q_I_Mrp(3), 0.);
  const Eigen::Vector3d M_delta =
      R_M_G_ * Eigen::Vector3d(G_delta(0), G_delta(1), 0.0);
  const Eigen::Vector4d q_M_Mrp =
      common::rotationVectorToQuaternionJPL(M_delta);

  Eigen::Matrix<double, 4, 1> q_M_I_plus_M_delta;
  common::positiveQuaternionProductJPL(
      q_M_Mrp, common::quaternionInverseJPL(q_I_Mrp), q_M_I_plus_M_delta);
  q_I_M_plus_M_delta = common::quaternionInverseJPL(q_M_I_plus_M_delta);
  CHECK_GE(q_I_M_plus_M_delta(3), 0.);
  return true;
}

bool JplRollPitchQuaternionParameterization::ComputeJacobian(
    const double* q_I_M_ptr, double* jacobian_row_major_ptr) const {
  CHECK_NOTNULL(q_I_M_ptr);
  CHECK_NOTNULL(jacobian_row_major_ptr);

  const Eigen::Map<const Eigen::Matrix<double, 4, 1>> q_I_M(q_I_M_ptr);
  Eigen::Map<Eigen::Matrix<double, 4, 2, Eigen::RowMajor>> jacobian(
      jacobian_row_major_ptr);

  Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dqJPL_dtheta;
  get_dQuaternionJpl_dTheta(q_I_M_ptr, dqJPL_dtheta.data());

  Eigen::Matrix3d R_I_M;
  common::toRotationMatrixJPL(q_I_M, &R_I_M);

  // q_IM = (q_MI)^-1 = (q_MMrp(G_delta) * q_MrpI)^-1
  //                  = (q_MMrp(R_MG * G_delta) * q_MrpI)^-1
  jacobian = (-dqJPL_dtheta * R_I_M * R_M_G_).leftCols<2>();
  return true;
}

}  // namespace ceres_error_terms
