#ifndef CERES_ERROR_TERMS_LOOP_CLOSURE_BLOCK_POSE_ERROR_TERM_INL_H_
#define CERES_ERROR_TERMS_LOOP_CLOSURE_BLOCK_POSE_ERROR_TERM_INL_H_

namespace ceres_error_terms {

template <typename T>
bool LoopClosureBlockPoseErrorTerm::operator()(const T* const T_G_MA,
                                               const T* const T_MA_IA,
                                               const T* const T_G_MB,
                                               const T* const T_MB_IB,
                                               const T* const switch_variable,
                                               T* residuals) const {
  // Define Quaternion and Position of type T.
  typedef pose::QuaternionTemplate<T> QuaternionT;
  typedef pose::PositionTemplate<T> PositionT;

  // Maps memory regions to Eigen:: vectors.
  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_G_MA_map(
      T_G_MA + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_MA_IA_map(
      T_MA_IA + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_G_MB_map(
      T_G_MB + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_MB_IB_map(
      T_MB_IB + kOrientationBlockSize);

  const Eigen::Map<const Eigen::Quaternion<T> > q_MA_G_map(T_G_MA);
  const Eigen::Map<const Eigen::Quaternion<T> > q_MA_IA_map(T_MA_IA);
  const Eigen::Map<const Eigen::Quaternion<T> > q_MB_G_map(T_G_MB);
  const Eigen::Map<const Eigen::Quaternion<T> > q_MB_IB_map(T_MB_IB);

  Eigen::Map<Eigen::Matrix<T, 6, 1> > error(residuals);

  QuaternionT q_G_IA =
      QuaternionT(q_MA_G_map.inverse()) * QuaternionT(q_MA_IA_map);
  QuaternionT q_G_IB =
      QuaternionT(q_MB_G_map.inverse()) * QuaternionT(q_MB_IB_map);
  PositionT p_G_IA = p_G_MA_map + q_MA_G_map.inverse() * p_MA_IA_map;
  PositionT p_G_IB = p_G_MB_map + q_MB_G_map.inverse() * p_MB_IB_map;

  QuaternionT q_IA_IB_estimated =
      QuaternionT(q_G_IA).inverse() * QuaternionT(q_G_IB);
  QuaternionT q_IA_IB_measured(
      delta_pose_.getRotation().toImplementation().cast<T>());
  QuaternionT q_IB_measured_IB_estimated =
      q_IA_IB_measured.inverse() * q_IA_IB_estimated;

  if (q_IA_IB_measured.w() < static_cast<T>(0.)) {
    q_IA_IB_measured.toImplementation().coeffs() =
        -q_IA_IB_measured.toImplementation().coeffs();
  }
  CHECK_GE(q_IA_IB_measured.w(), static_cast<T>(0.));

  Eigen::Matrix<T, 3, 3> R_IA_G = q_G_IA.getRotationMatrix().transpose();

  // Residual for translation.
  error.head(3) =
      R_IA_G * (p_G_IB - p_G_IA) - delta_pose_.getPosition().cast<T>();
  // Residual for orientation (using small angle approximation).
  error.tail(3) = Eigen::Matrix<T, 3, 1>(
      static_cast<T>(2.0) * q_IB_measured_IB_estimated.x(),
      static_cast<T>(2.0) * q_IB_measured_IB_estimated.y(),
      static_cast<T>(2.0) * q_IB_measured_IB_estimated.z());
  error = (*switch_variable) * error.transpose() *
          sqrt_information_matrix_.cast<T>();

  return true;
}

template <typename T>
bool LoopClosureBlockPoseErrorTerm::operator()(const T* const T_M_IA,
                                               const T* const T_M_IB,
                                               const T* const switch_variable,
                                               T* residuals) const {
  // Define Quaternion of scalar type T.
  typedef pose::QuaternionTemplate<T> QuaternionT;

  // Maps memory regions to Eigen:: vectors.
  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_M_IA_map(
      T_M_IA + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_M_IB_map(
      T_M_IB + kOrientationBlockSize);

  const Eigen::Map<const Eigen::Quaternion<T> > q_M_IA_map(T_M_IA);
  const Eigen::Map<const Eigen::Quaternion<T> > q_M_IB_map(T_M_IB);

  Eigen::Map<Eigen::Matrix<T, 6, 1> > error(residuals);

  QuaternionT q_A_B_estimated =
      QuaternionT(q_M_IA_map).inverse() * QuaternionT(q_M_IB_map);
  QuaternionT q_A_B_jet(delta_pose_.getRotation().toImplementation().cast<T>());
  QuaternionT q_A_A_estimated = q_A_B_jet.inverse() * q_A_B_estimated;

  if (q_A_A_estimated.w() < static_cast<T>(0.)) {
    q_A_A_estimated.toImplementation().coeffs() =
        -q_A_A_estimated.toImplementation().coeffs();
  }
  CHECK_GE(q_A_A_estimated.w(), static_cast<T>(0.));

  Eigen::Matrix<T, 3, 3> R_IA_M = q_M_IA_map.toRotationMatrix().transpose();

  // Residual for translation.
  error.head(3) =
      R_IA_M * (p_M_IB_map - p_M_IA_map) - delta_pose_.getPosition().cast<T>();
  // Residual for orientation (using small angle approximation).
  error.tail(3) = Eigen::Matrix<T, 3, 1>(
      static_cast<T>(2.0) * q_A_A_estimated.x(),
      static_cast<T>(2.0) * q_A_A_estimated.y(),
      static_cast<T>(2.0) * q_A_A_estimated.z());
  error = (*switch_variable) * error.transpose() *
          sqrt_information_matrix_.cast<T>();
  return true;
}

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_LOOP_CLOSURE_BLOCK_POSE_ERROR_TERM_INL_H_
