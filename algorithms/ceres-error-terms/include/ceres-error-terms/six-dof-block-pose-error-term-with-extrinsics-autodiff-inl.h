#ifndef CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_WITH_EXTRINSICS_AUTODIFF_INL_H_
#define CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_WITH_EXTRINSICS_AUTODIFF_INL_H_
namespace ceres_error_terms {

template <typename T>
bool SixDoFBlockPoseErrorTermWithExtrinsics::operator()(
    const T* const T_G_Ik, const T* const T_G_Ikp1, const T* const q_I_B,
    const T* const p_B_I, T* residuals) const {
  // Define Quaternion and Position of type T.
  typedef pose::QuaternionTemplate<T> QuaternionT;
  typedef pose::PositionTemplate<T> PositionT;
  typedef pose::TransformationTemplate<T> TransformationT;

  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_G_Ik_map(
      T_G_Ik + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_G_Ikp1_map(
      T_G_Ikp1 + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_B_I_map(p_B_I);

  const Eigen::Map<const Eigen::Quaternion<T>> q_G_Ik_map(T_G_Ik);
  const Eigen::Map<const Eigen::Quaternion<T>> q_G_Ikp1_map(T_G_Ikp1);
  const Eigen::Map<const Eigen::Quaternion<T>> q_I_B_map(q_I_B);

  const TransformationT T_G_Ik_estimated(QuaternionT(q_G_Ik_map), p_G_Ik_map);
  const TransformationT T_G_Ikp1_estimated(
      QuaternionT(q_G_Ikp1_map), p_G_Ikp1_map);
  const TransformationT T_B_I_estimated(
      QuaternionT(q_I_B_map).inverse(), p_B_I_map);
  const TransformationT T_Bk_Bkp1_estimated =
      T_B_I_estimated * T_G_Ik_estimated.inverse() * T_G_Ikp1_estimated *
      T_B_I_estimated.inverse();

  QuaternionT q_Bk_Bkp1_estimated = T_Bk_Bkp1_estimated.getRotation();
  QuaternionT q_Bk_Bkp1_jet(
      T_Bk_Bkp1_.getRotation().toImplementation().cast<T>());

  // Get quaternion difference.
  QuaternionT q_Bk_measurement_Bk_estimated =
      q_Bk_Bkp1_jet.inverse() * q_Bk_Bkp1_estimated;

  // Flips the quaternion, if necessary.
  if (q_Bk_measurement_Bk_estimated.w() < static_cast<T>(0.)) {
    q_Bk_measurement_Bk_estimated.toImplementation().coeffs() =
        -q_Bk_measurement_Bk_estimated.toImplementation().coeffs();
  }
  CHECK_GE(q_Bk_measurement_Bk_estimated.w(), static_cast<T>(0.));

  const PositionT p_BkBkp1_measurement_BkBkp1_estimated =
      T_Bk_Bkp1_.getPosition().cast<T>() - T_Bk_Bkp1_estimated.getPosition();

  Eigen::Map<Eigen::Matrix<T, 6, 1>> error(residuals);
  // The residual for translation.
  error.head(3) = p_BkBkp1_measurement_BkBkp1_estimated;
  // The residual for orientation (using small angle approximation).
  error.tail(3) = Eigen::Matrix<T, 3, 1>(
      static_cast<T>(2.0) * q_Bk_measurement_Bk_estimated.x(),
      static_cast<T>(2.0) * q_Bk_measurement_Bk_estimated.y(),
      static_cast<T>(2.0) * q_Bk_measurement_Bk_estimated.z());
  error = error.transpose() * T_Bk_Bkp1_sqrt_information_matrix_.cast<T>();
  return true;
}

}  // namespace ceres_error_terms
#endif  // CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_WITH_EXTRINSICS_AUTODIFF_INL_H_
