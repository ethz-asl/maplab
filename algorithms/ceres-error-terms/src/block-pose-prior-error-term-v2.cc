#include <memory>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <ceres-error-terms/common.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>

#include "ceres-error-terms/block-pose-prior-error-term-v2.h"

namespace ceres_error_terms {

BlockPosePriorErrorTermV2::BlockPosePriorErrorTermV2(
    const aslam::Transformation& T_G_S_measured,
    const Eigen::Matrix<double, 6, 6>& covariance)
    : q_S_G_measured_(
          T_G_S_measured.getRotation().toImplementation().inverse()),
      p_G_S_measured_(T_G_S_measured.getPosition()) {
  // Getting inverse square root of covariance matrix.
  Eigen::Matrix<double, 6, 6> L = covariance.llt().matrixL();
  sqrt_information_matrix_.setIdentity();
  L.triangularView<Eigen::Lower>().solveInPlace(sqrt_information_matrix_);

  // inverse_orientation_prior_ =
  //    Eigen::Quaterniond(orientation_prior_).inverse().coeffs();
}

}  // namespace ceres_error_terms
