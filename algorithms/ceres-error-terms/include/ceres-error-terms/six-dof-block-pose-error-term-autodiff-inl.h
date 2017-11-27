#ifndef CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_AUTODIFF_INL_H_
#define CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_AUTODIFF_INL_H_
namespace ceres_error_terms {

template <typename T>
bool SixDoFBlockPoseErrorTerm::operator()(
    const T* const T_G_A, const T* const T_G_B, T* residuals) const {
  // Define Quaternion and Position of type T.
  typedef pose::QuaternionTemplate<T> QuaternionT;
  typedef pose::PositionTemplate<T> PositionT;

  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_G_A_map(
      T_G_A + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_G_B_map(
      T_G_B + kOrientationBlockSize);

  const Eigen::Map<const Eigen::Quaternion<T> > q_G_A_map(T_G_A);
  const Eigen::Map<const Eigen::Quaternion<T> > q_G_B_map(T_G_B);

  Eigen::Map<Eigen::Matrix<T, 6, 1> > error(residuals);

  QuaternionT q_A_B_estimated =
      QuaternionT(q_G_A_map).inverse() * QuaternionT(q_G_B_map);
  QuaternionT q_A_B_jet(delta_pose_.getRotation().toImplementation().cast<T>());

  // Get quaternion difference.
  QuaternionT q_A_A_estimated = q_A_B_jet.inverse() * q_A_B_estimated;

  if (q_A_A_estimated.w() < static_cast<T>(0.)) {
    q_A_A_estimated.toImplementation().coeffs() =
        -q_A_A_estimated.toImplementation().coeffs();
  }
  CHECK_GE(q_A_A_estimated.w(), static_cast<T>(0.));

  Eigen::Matrix<T, 3, 3> R_A_G = q_G_A_map.toRotationMatrix().transpose();

  PositionT p_G_AB_est = PositionT(p_G_B_map - p_G_A_map);
  PositionT p_A_B_est(R_A_G * p_G_AB_est);

  // The residual for translation.
  error.head(3) = (p_A_B_est - PositionT(delta_pose_.getPosition().cast<T>()));
  // The residual for orientation (using small angle approximation).
  error.tail(3) = Eigen::Matrix<T, 3, 1>(
      static_cast<T>(2.0) * q_A_A_estimated.x(),
      static_cast<T>(2.0) * q_A_A_estimated.y(),
      static_cast<T>(2.0) * q_A_A_estimated.z());
  error = error.transpose() * sqrt_information_matrix_.cast<T>();

  return true;
}

}  // namespace ceres_error_terms
#endif  // CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_AUTODIFF_INL_H_
