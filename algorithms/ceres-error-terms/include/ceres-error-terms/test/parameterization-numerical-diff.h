#ifndef CERES_ERROR_TERMS_TEST_PARAMETERIZATION_NUMERICAL_DIFF_H_
#define CERES_ERROR_TERMS_TEST_PARAMETERIZATION_NUMERICAL_DIFF_H_

#include <memory>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <maplab-common/pose_types.h>

namespace ceres_error_terms {

template <typename LocalParameterization, int kDimGlobal, int kDimLocal,
          int EigenJacobianOptions>
bool EvaluateNumericalJacobianOfParameterization(
    const LocalParameterization& parameterization,
    const Eigen::Matrix<double, kDimGlobal, 1>& linearization_point_global,
    Eigen::Matrix<double, kDimGlobal, kDimLocal, EigenJacobianOptions>*
        dLocal_dGlobal);

}  // namespace ceres_error_terms
#include "ceres-error-terms/test/parameterization-numerical-diff-inl.h"
#endif  // CERES_ERROR_TERMS_TEST_PARAMETERIZATION_NUMERICAL_DIFF_H_
