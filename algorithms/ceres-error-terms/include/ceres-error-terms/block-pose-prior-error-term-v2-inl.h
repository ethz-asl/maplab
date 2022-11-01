#ifndef CERES_ERROR_TERMS_BLOCK_POSE_PRIOR_ERROR_TERM_V2_INL_H_
#define CERES_ERROR_TERMS_BLOCK_POSE_PRIOR_ERROR_TERM_V2_INL_H_

namespace ceres_error_terms {

template <typename T>
bool BlockPosePriorErrorTermV2::operator()(
    const T* const q_G_M_jpl_p_G_M, const T* const q_B_M_jpl_p_M_B,
    const T* const q_S_B_jpl_p_S_B, T* residuals) const {
  // Define Quaternion and Position of type T.
  typedef typename Eigen::Matrix<T, 3, 3> RotationMatrixT;
  typedef aslam::PositionTemplate<T> PositionT;

  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_G_M_map(
      q_G_M_jpl_p_G_M + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_M_B_map(
      q_B_M_jpl_p_M_B + kOrientationBlockSize);
  const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_S_B_map(
      q_S_B_jpl_p_S_B + kOrientationBlockSize);

  const Eigen::Map<const Eigen::Matrix<T, 4, 1>> q_G_M_jpl_map(q_G_M_jpl_p_G_M);
  RotationMatrixT R_G_M_jpl;
  common::toRotationMatrixJPL(q_G_M_jpl_map, &R_G_M_jpl);
  const RotationMatrixT R_G_M = R_G_M_jpl;

  const Eigen::Map<const Eigen::Matrix<T, 4, 1>> q_B_M_jpl_map(q_B_M_jpl_p_M_B);
  RotationMatrixT R_B_M_jpl;
  common::toRotationMatrixJPL(q_B_M_jpl_map, &R_B_M_jpl);
  const RotationMatrixT R_M_B = R_B_M_jpl.transpose();

  const Eigen::Map<const Eigen::Matrix<T, 4, 1>> q_S_B_jpl_map(q_S_B_jpl_p_S_B);
  RotationMatrixT R_S_B_jpl;
  common::toRotationMatrixJPL(q_S_B_jpl_map, &R_S_B_jpl);
  const RotationMatrixT R_B_S = R_S_B_jpl.transpose();

  const RotationMatrixT R_G_S = R_G_M * R_M_B * R_B_S;

  // TODO(mfehr): DOUBLE CHECK IF THIS IS CORRECT!!
  const Eigen::Matrix<T, 3, 1> p_B_S = -(R_B_S * p_S_B_map);

  const PositionT p_G_S = R_G_M * R_M_B * p_B_S + R_G_M * p_M_B_map + p_G_M_map;

  const Eigen::Quaternion<T> q_G_S_estimated(R_G_S);
  const Eigen::Quaternion<T> q_S_G_measured(q_S_G_measured_);
  const Eigen::Quaternion<T> q_diff = q_G_S_estimated * q_S_G_measured;

  Eigen::Map<Eigen::Matrix<T, kResidualBlockSize, 1>> error(residuals);
  error.head(3) = p_G_S - p_G_S_measured_.cast<T>();
  error.tail(3) = Eigen::Matrix<T, 3, 1>(
      static_cast<T>(2.0) * q_diff.x(), static_cast<T>(2.0) * q_diff.y(),
      static_cast<T>(2.0) * q_diff.z());
  error = error.transpose() * sqrt_information_matrix_.cast<T>();

  return true;
}

}  // namespace ceres_error_terms
#endif  // CERES_ERROR_TERMS_BLOCK_POSE_PRIOR_ERROR_TERM_V2_INL_H_
