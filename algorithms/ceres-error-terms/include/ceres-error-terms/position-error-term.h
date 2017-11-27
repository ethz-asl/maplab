#ifndef CERES_ERROR_TERMS_POSITION_ERROR_TERM_H_
#define CERES_ERROR_TERMS_POSITION_ERROR_TERM_H_

#include <memory>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>

#include <ceres-error-terms/common.h>
#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <maplab-common/pose_types.h>

namespace ceres_error_terms {

// Error term taking in a full pose (position + orientation) parametrization
// but only operates on the position block.
class PositionErrorTerm
    : public ceres::SizedCostFunction<positionblocks::kResidualSize,
                                      poseblocks::kPoseSize> {
 public:
  PositionErrorTerm(
      const Eigen::Vector3d& position_prior,
      const Eigen::Matrix<double, 3, 3>& covariance)
      : position_prior_(position_prior) {
    // Getting inverse square root of covariance matrix.
    Eigen::Matrix<double, 3, 3> L = covariance.llt().matrixL();
    sqrt_information_matrix_.setIdentity();
    L.triangularView<Eigen::Lower>().solveInPlace(sqrt_information_matrix_);
  }

  virtual ~PositionErrorTerm() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Don't change the ordering of the enum elements, they have to be the
  // same as the order of the parameter blocks.
  enum { kIdxPose };

  // The representation for Jacobian computed by this object.
  typedef Eigen::Matrix<double, positionblocks::kResidualSize,
                        poseblocks::kPoseSize, Eigen::RowMajor>
      PositionJacobian;

  Eigen::Matrix<double, 3, 3> sqrt_information_matrix_;
  Eigen::Vector3d position_prior_;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_POSITION_ERROR_TERM_H_
