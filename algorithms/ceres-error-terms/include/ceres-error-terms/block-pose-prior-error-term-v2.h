#ifndef CERES_ERROR_TERMS_BLOCK_POSE_PRIOR_ERROR_TERM_V2_H_
#define CERES_ERROR_TERMS_BLOCK_POSE_PRIOR_ERROR_TERM_V2_H_

#include <memory>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <ceres-error-terms/common.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>
#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

// Note: this error term accepts rotations expressed as quaternions
// in JPL convention [x, y, z, w]. This convention corresponds to the internal
// coefficient storage of Eigen so you can directly pass pointer to your
// Eigen quaternion data, e.g. your_eigen_quaternion.coeffs().data().
class BlockPosePriorErrorTermV2 {
 public:
  BlockPosePriorErrorTermV2(
      const aslam::Transformation& T_G_S_measured,
      const Eigen::Matrix<double, 6, 6>& covariance);

  ~BlockPosePriorErrorTermV2() {}

  template <typename T>
  bool operator()(
      const T* const q_G_M_jpl_p_G_M, const T* const q_B_M_jpl_p_M_B,
      const T* const q_S_B_jpl_p_S_B, T* residuals) const;

  static constexpr int kResidualBlockSize = 6;
  static constexpr int kOrientationBlockSize = 4;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Don't change the ordering of the enum elements, they have to be the
  // same as the order of the parameter blocks.
  enum {
    kIdxBaseframePose,
    kIdxVertexPose,
    kIdxSensorPose,
  };

  Eigen::Matrix<double, 6, 6> sqrt_information_matrix_;

  Eigen::Quaterniond q_S_G_measured_;
  Eigen::Vector3d p_G_S_measured_;
};

}  // namespace ceres_error_terms

#include "ceres-error-terms/block-pose-prior-error-term-v2-inl.h"

#endif  // CERES_ERROR_TERMS_BLOCK_POSE_PRIOR_ERROR_TERM_V2_H_
