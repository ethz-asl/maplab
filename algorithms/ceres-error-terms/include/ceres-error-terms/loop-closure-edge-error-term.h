#ifndef CERES_ERROR_TERMS_LOOP_CLOSURE_EDGE_ERROR_TERM_H_
#define CERES_ERROR_TERMS_LOOP_CLOSURE_EDGE_ERROR_TERM_H_

#include <Eigen/Dense>

#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

class LoopClosureEdgeErrorTerm {
 public:
  LoopClosureEdgeErrorTerm(
      const Eigen::Matrix<double, 7, 1>& q_AB__A_p_AB,
      const Eigen::Matrix<double, 6, 6>& T_A_B_covariance)
      : q_AB__A_p_AB_(q_AB__A_p_AB) {
    Eigen::Matrix<double, 6, 6> L = T_A_B_covariance.llt().matrixL();
    Eigen::Matrix<double, 6, 6> inv_L = Eigen::Matrix<double, 6, 6>::Identity();

    L.template triangularView<Eigen::Lower>().solveInPlace(inv_L);
    sqrt_information_matrix_ = inv_L;
  }

  // Version where the vertices are in different mission frame of reference.
  template <typename T>
  bool operator()(
      const T* const T_G_MA, const T* const T_MA_IA, const T* const T_G_MB,
      const T* const T_MB_IB, const T* const switch_variable,
      T* residuals) const;

  // Version where both vertices are in the same mission frame of reference.
  template <typename T>
  bool operator()(
      const T* const T_M_IA, const T* const T_M_IB,
      const T* const switch_variable, T* residuals) const;

  static constexpr int kResidualBlockSize = 6;
  static constexpr int kSwitchVariableBlockSize = 1;
  static constexpr int kOrientationBlockSize = 4;
  static constexpr int kPositionBlockSize = 3;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Matrix<double, 7, 1> q_AB__A_p_AB_;
  Eigen::Matrix<double, 6, 6> sqrt_information_matrix_;
};

}  // namespace ceres_error_terms
#include "ceres-error-terms/loop-closure-edge-error-term-inl.h"
#endif  // CERES_ERROR_TERMS_LOOP_CLOSURE_EDGE_ERROR_TERM_H_
