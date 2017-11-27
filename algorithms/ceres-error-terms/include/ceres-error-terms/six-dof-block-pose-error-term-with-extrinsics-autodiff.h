#ifndef CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_WITH_EXTRINSICS_AUTODIFF_H_
#define CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_WITH_EXTRINSICS_AUTODIFF_H_

#include <Eigen/Dense>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

// Coordinate frames used:
// I: IMU body frame,
// B: Sensor body frame.
// This error-term models a residual for the following situation:
// A sensor measures the relative transformation between time k and
// time kp1, expressed in the sensor body frame B.
// The sensor is rigidly attached to the robot, with T_B_I denoting the
// transformation between the sensor body frame B and the IMU body frame I.
class SixDoFBlockPoseErrorTermWithExtrinsics {
 public:
  SixDoFBlockPoseErrorTermWithExtrinsics(
      const pose::Transformation& T_Bk_Bkp1,
      const Eigen::Matrix<double, 6, 6>& T_Bk_Bkp1_covariance)
      : T_Bk_Bkp1_(T_Bk_Bkp1) {
    const Eigen::Matrix<double, 6, 6> L_Bk_Bkp1 =
        T_Bk_Bkp1_covariance.llt().matrixL();
    Eigen::Matrix<double, 6, 6> inv_L_Bk_Bkp1 =
        Eigen::Matrix<double, 6, 6>::Identity();
    L_Bk_Bkp1.template triangularView<Eigen::Lower>().solveInPlace(
        inv_L_Bk_Bkp1);
    T_Bk_Bkp1_sqrt_information_matrix_ = inv_L_Bk_Bkp1;
  }

  template <typename T>
  bool operator()(
      const T* const T_G_Ik, const T* const T_G_Ikp1, const T* const q_I_B,
      const T* const p_B_I, T* residuals) const;

  static constexpr int kResidualBlockSize = 6;
  static constexpr int kOrientationBlockSize = 4;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  pose::Transformation T_Bk_Bkp1_;
  Eigen::Matrix<double, 6, 6> T_Bk_Bkp1_sqrt_information_matrix_;
};

}  // namespace ceres_error_terms

#include "./ceres-error-terms/six-dof-block-pose-error-term-with-extrinsics-autodiff-inl.h"

#endif  // CERES_ERROR_TERMS_SIX_DOF_BLOCK_POSE_ERROR_TERM_WITH_EXTRINSICS_AUTODIFF_H_
