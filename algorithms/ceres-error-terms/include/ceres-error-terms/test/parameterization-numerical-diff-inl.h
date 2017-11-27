#ifndef CERES_ERROR_TERMS_TEST_PARAMETERIZATION_NUMERICAL_DIFF_INL_H_
#define CERES_ERROR_TERMS_TEST_PARAMETERIZATION_NUMERICAL_DIFF_INL_H_

#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace ceres_error_terms {
// Ceres does not provide interfaces to numerically differentiate local
// parameterization. This class wraps the parameterization in a cost function
// to provide this functionality. The linearization point is fixed to the origin
// of the global space.
template <int kDimGlobal, int kDimLocal>
class ParameterizationCostWrapper {
 public:
  ParameterizationCostWrapper(
      const ceres::LocalParameterization& parameterization,
      const Eigen::Matrix<double, kDimGlobal, 1>& linearization_point_global)
      : parameterization_(parameterization),
        linearization_point_global_(linearization_point_global) {
    CHECK_EQ(kDimLocal, parameterization.LocalSize());
    CHECK_EQ(kDimGlobal, parameterization.GlobalSize());
  }
  bool operator()(const double* const delta_tocal, double* q_global) const {
    CHECK_NOTNULL(delta_tocal);
    CHECK_NOTNULL(q_global);
    parameterization_.Plus(
        linearization_point_global_.data(), delta_tocal, q_global);
    return true;
  }

 private:
  const ceres::LocalParameterization& parameterization_;
  const Eigen::Matrix<double, kDimGlobal, 1> linearization_point_global_;
};

template <typename LocalParameterization, int kDimGlobal, int kDimLocal,
          int EigenJacobianOptions>
bool EvaluateNumericalJacobianOfParameterization(
    const LocalParameterization& parameterization,
    const Eigen::Matrix<double, kDimGlobal, 1>& linearization_point_global,
    Eigen::Matrix<double, kDimGlobal, kDimLocal, EigenJacobianOptions>*
        dLocal_dGlobal) {
  CHECK_NOTNULL(dLocal_dGlobal);
  CHECK_LT(kDimLocal, kDimGlobal);

  // Ceres requires RowMajor format for Jacobians but Eigen does not support
  // this flag for Vector (any dim equal 1). Therefore we template on the
  // Storage type and check that it is RowMajor in case the Jacobian is a proper
  // matrix.
  if (kDimGlobal != 1 && kDimLocal != 1) {
    CHECK_EQ(EigenJacobianOptions, Eigen::RowMajor);
  }

  typedef ParameterizationCostWrapper<kDimGlobal, kDimLocal>
      WrappedParameterization;
  WrappedParameterization wrapped_parameterization(
      parameterization, linearization_point_global);

  ceres::NumericDiffCostFunction<WrappedParameterization, ceres::RIDDERS,
                                 kDimGlobal, kDimLocal>
      cost_function(&wrapped_parameterization, ceres::DO_NOT_TAKE_OWNERSHIP);

  Eigen::Matrix<double, kDimLocal, 1> delta_tocal =
      Eigen::Matrix<double, kDimLocal, 1>::Zero();
  std::vector<double*> parameters{delta_tocal.data()};
  Eigen::Matrix<double, kDimGlobal, 1> residual;

  Eigen::Matrix<double, kDimGlobal, kDimLocal, EigenJacobianOptions>
      dLocal_dGlobal_rowmajor;
  std::vector<double*> jacs{dLocal_dGlobal_rowmajor.data()};

  const bool success =
      cost_function.Evaluate(parameters.data(), residual.data(), jacs.data());

  // Convert ceres rowmajor to output format.
  *dLocal_dGlobal = dLocal_dGlobal_rowmajor;
  return success;
}
}  // namespace ceres_error_terms
#endif  // CERES_ERROR_TERMS_TEST_PARAMETERIZATION_NUMERICAL_DIFF_INL_H_
