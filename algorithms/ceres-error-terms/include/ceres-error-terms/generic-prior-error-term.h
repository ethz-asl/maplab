#ifndef CERES_ERROR_TERMS_GENERIC_PRIOR_ERROR_TERM_H_
#define CERES_ERROR_TERMS_GENERIC_PRIOR_ERROR_TERM_H_

#include <memory>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>

namespace ceres_error_terms {

template <int ParameterBlockSize, int PriorBlockIndex, int PriorBlockSize>
class GenericPriorErrorTerm
    : public ceres::SizedCostFunction<ParameterBlockSize, ParameterBlockSize> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GenericPriorErrorTerm(
      const Eigen::Matrix<double, PriorBlockSize, 1>& prior_mean,
      const Eigen::Matrix<double, PriorBlockSize, PriorBlockSize>&
          prior_covariance)
      : prior_mean_(prior_mean) {
    static_assert(ParameterBlockSize > 1, "Requires dimension to be >= 2.");
    static_assert(
        PriorBlockIndex >= 0, "Prior block index must be non-negative.");
    static_assert(
        PriorBlockSize > 0, "Prior block size must be larger than zero.");

    // Getting inverse square root of covariance matrix.
    Eigen::Matrix<double, PriorBlockSize, PriorBlockSize> L =
        prior_covariance.llt().matrixL();
    sqrt_information_matrix_.setIdentity();
    L.template triangularView<Eigen::Lower>().solveInPlace(
        sqrt_information_matrix_);
  }

  virtual ~GenericPriorErrorTerm() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const {
    CHECK_NOTNULL(parameters);
    CHECK_NOTNULL(residuals);

    Eigen::Map<const Eigen::Matrix<double, ParameterBlockSize, 1>>
        current_value(parameters[kIdxVector]);

    // Calculate the whitened residuals.
    Eigen::Map<Eigen::Matrix<double, ParameterBlockSize, 1>> residual_vector(
        residuals);
    residual_vector.setZero();
    residual_vector.segment(PriorBlockIndex, PriorBlockSize) =
        sqrt_information_matrix_ *
        (current_value.segment(PriorBlockIndex, PriorBlockSize) - prior_mean_);

    if (jacobians) {
      if (jacobians[kIdxVector]) {
        // J = sqrt_information_matrix_ * Jb, with Jb = Identity
        Eigen::Map<PriorJacobian> J(jacobians[kIdxVector]);
        J.setZero();
        J.block(
            PriorBlockIndex, PriorBlockIndex, PriorBlockSize, PriorBlockSize) =
            sqrt_information_matrix_;
      }
    }
    return true;
  }

 private:
  // Parameter ordering.
  enum { kIdxVector };

  // The representation for Jacobian computed by this object.
  typedef Eigen::Matrix<double, ParameterBlockSize, ParameterBlockSize,
                        Eigen::RowMajor>
      PriorJacobian;

  const Eigen::Matrix<double, PriorBlockSize, 1> prior_mean_;
  Eigen::Matrix<double, PriorBlockSize, PriorBlockSize>
      sqrt_information_matrix_;
};

}  // namespace ceres_error_terms
#endif  // CERES_ERROR_TERMS_GENERIC_PRIOR_ERROR_TERM_H_
