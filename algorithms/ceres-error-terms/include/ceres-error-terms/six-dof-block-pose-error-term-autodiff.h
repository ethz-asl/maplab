#ifndef CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_AUTODIFF_H_
#define CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_AUTODIFF_H_

#include <Eigen/Dense>

#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

class SixDoFBlockPoseErrorTerm {
 public:
  SixDoFBlockPoseErrorTerm(
      const pose::Transformation& T_A_B,
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance)
      : delta_pose_(T_A_B) {
    Eigen::Matrix<double, 6, 6> L = T_A_B_covariance.llt().matrixL();
    Eigen::Matrix<double, 6, 6> inv_L = Eigen::Matrix<double, 6, 6>::Identity();

    L.template triangularView<Eigen::Lower>().solveInPlace(inv_L);

    sqrt_information_matrix_ = inv_L;
  }

  template <typename T>
  bool operator()(
      const T* const T_G_A, const T* const T_G_B, T* residuals) const;

  static constexpr int residualBlockSize = 6;
  static constexpr int kOrientationBlockSize = 4;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  pose::Transformation delta_pose_;
  Eigen::Matrix<double, 6, 6> sqrt_information_matrix_;
};

}  // namespace ceres_error_terms

#include "./ceres-error-terms/six-dof-block-pose-error-term-autodiff-inl.h"

#endif  // CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_AUTODIFF_H_
