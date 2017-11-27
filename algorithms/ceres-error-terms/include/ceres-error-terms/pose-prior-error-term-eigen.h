#ifndef CERES_ERROR_TERMS_POSE_PRIOR_ERROR_TERM_EIGEN_H_
#define CERES_ERROR_TERMS_POSE_PRIOR_ERROR_TERM_EIGEN_H_

#include <memory>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>

#include <ceres-error-terms/common.h>
#include <ceres-error-terms/parameterization/quaternion-param-eigen.h>
#include <maplab-common/pose_types.h>

namespace ceres_error_terms {

// Note: this error term accepts rotations expressed as quaternions
// in JPL convention [x, y, z, w]. This convention corresponds to the internal
// coefficient storage of Eigen so you can directly pass pointer to your
// Eigen quaternion data, e.g. your_eigen_quaternion.coeffs().data().
class EigenPosePriorErrorTerm
    : public ceres::SizedCostFunction<
          poseblocks::kResidualSize, poseblocks::kOrientationBlockSize,
          poseblocks::kPositionBlockSize> {
 public:
  EigenPosePriorErrorTerm(
      const Eigen::Quaterniond& orientation_prior,
      const Eigen::Vector3d& position_prior,
      const Eigen::Matrix<double, 6, 6>& covariance)
      : orientation_prior_(orientation_prior), position_prior_(position_prior) {
    if (orientation_prior.norm() > 1.0000001) {
      LOG(WARNING) << "Orientation norm larger than 1: "
                   << orientation_prior.norm();
    }

    // Getting inverse square root of covariance matrix.
    Eigen::Matrix<double, 6, 6> L = covariance.llt().matrixL();
    sqrt_information_matrix_.setIdentity();
    L.triangularView<Eigen::Lower>().solveInPlace(sqrt_information_matrix_);

    inverse_orientation_prior_ =
        Eigen::Quaterniond(orientation_prior_).inverse();
  }

  virtual ~EigenPosePriorErrorTerm() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Don't change the ordering of the enum elements, they have to be the
  // same as the order of the parameter blocks.
  enum { kIdxQ, kIdxP };

  // The representation for Jacobian computed by this object.
  typedef Eigen::Matrix<
      double, poseblocks::kResidualSize, poseblocks::kOrientationBlockSize,
      Eigen::RowMajor>
      OrientationJacobian;

  typedef Eigen::Matrix<
      double, poseblocks::kResidualSize, poseblocks::kPositionBlockSize,
      Eigen::RowMajor>
      PositionJacobian;
  typedef Eigen::Matrix<
      double, poseblocks::kResidualSize, poseblocks::kPoseSize, Eigen::RowMajor>
      PoseJacobian;

  Eigen::Matrix<double, 6, 6> sqrt_information_matrix_;
  Eigen::Quaterniond orientation_prior_;
  Eigen::Quaterniond inverse_orientation_prior_;
  Eigen::Vector3d position_prior_;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_POSE_PRIOR_ERROR_TERM_EIGEN_H_
