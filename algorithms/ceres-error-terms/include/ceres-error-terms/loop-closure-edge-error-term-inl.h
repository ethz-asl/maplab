#ifndef CERES_ERROR_TERMS_LOOP_CLOSURE_EDGE_ERROR_TERM_INL_H_
#define CERES_ERROR_TERMS_LOOP_CLOSURE_EDGE_ERROR_TERM_INL_H_

#include "maplab-common/quaternion-math.h"

namespace ceres_error_terms {

template <typename T>
bool LoopClosureEdgeErrorTerm::operator()(
    const T* const T_G_MA, const T* const T_MA_IA, const T* const T_G_MB,
    const T* const T_MB_IB, const T* const switch_variable,
    T* residuals) const {
  // Maps memory regions to Eigen:: vectors.
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_G_MA_map(
      T_G_MA + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_MA_IA_map(
      T_MA_IA + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_G_MB_map(
      T_G_MB + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_MB_IB_map(
      T_MB_IB + kOrientationBlockSize);

  typedef Eigen::Matrix<T, 4, 1> Vector4T;

  const Eigen::Map<const Vector4T> q_G_MA_map(T_G_MA);
  const Eigen::Map<const Vector4T> q_IA_MA_map(T_MA_IA);
  const Eigen::Map<const Vector4T> q_G_MB_map(T_G_MB);
  const Eigen::Map<const Vector4T> q_IB_MB_map(T_MB_IB);

  Eigen::Map<Eigen::Matrix<T, 6, 1>> error(residuals);

  Eigen::Matrix<T, 3, 3> R_G_MA;
  common::toRotationMatrixJPL(q_G_MA_map, &R_G_MA);

  Eigen::Matrix<T, 3, 3> R_G_MB;
  common::toRotationMatrixJPL(q_G_MB_map, &R_G_MB);

  Eigen::Matrix<T, 3, 1> p_G_IA = p_G_MA_map + R_G_MA * p_MA_IA_map;
  Eigen::Matrix<T, 3, 1> p_G_IB = p_G_MB_map + R_G_MB * p_MB_IB_map;

  Vector4T q_G_IA;
  common::positiveQuaternionProductJPL(
      q_G_MA_map, common::quaternionInverseJPL(q_IA_MA_map), q_G_IA);

  Vector4T q_G_IB;
  common::positiveQuaternionProductJPL(
      q_G_MB_map, common::quaternionInverseJPL(q_IB_MB_map), q_G_IB);

  Vector4T q_IA_IB_estimated;
  common::positiveQuaternionProductJPL(
      common::quaternionInverseJPL(q_G_IA), q_G_IB, q_IA_IB_estimated);

  Vector4T q_IA_IB_lc = q_AB__A_p_AB_.head<kOrientationBlockSize>().cast<T>();

  Vector4T q_IB_lc_IB_estimated;
  common::positiveQuaternionProductJPL(
      common::quaternionInverseJPL(q_IA_IB_lc), q_IA_IB_estimated,
      q_IB_lc_IB_estimated);

  Eigen::Matrix<T, 3, 3> R_IA_G;
  common::toRotationMatrixJPL(common::quaternionInverseJPL(q_G_IA), &R_IA_G);

  // Residual for translation.
  error.head(3) = R_IA_G * (p_G_IB - p_G_IA) -
                  q_AB__A_p_AB_.tail<kPositionBlockSize>().cast<T>();
  // Residual for orientation (using small angle approximation).
  error.tail(3) = Eigen::Matrix<T, 3, 1>(
      static_cast<T>(2.0) * q_IB_lc_IB_estimated.x(),
      static_cast<T>(2.0) * q_IB_lc_IB_estimated.y(),
      static_cast<T>(2.0) * q_IB_lc_IB_estimated.z());
  error = (*switch_variable) * error.transpose() *
          sqrt_information_matrix_.cast<T>();

  return true;
}

template <typename T>
bool LoopClosureEdgeErrorTerm::operator()(
    const T* const T_M_IA, const T* const T_M_IB,
    const T* const switch_variable, T* residuals) const {
  Eigen::Matrix<double, 7, 1> identity;
  identity << 0, 0, 0, 1, 0, 0, 0;
  Eigen::Matrix<T, 7, 1> T_G_MA = identity.cast<T>();
  Eigen::Matrix<T, 7, 1> T_G_MB = identity.cast<T>();

  return this->operator()(
      T_G_MA.data(), T_M_IA, T_G_MB.data(), T_M_IB, switch_variable, residuals);
}

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_LOOP_CLOSURE_EDGE_ERROR_TERM_INL_H_
