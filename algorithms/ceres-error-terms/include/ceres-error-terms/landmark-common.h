#ifndef CERES_ERROR_TERMS_LANDMARK_COMMON_H_
#define CERES_ERROR_TERMS_LANDMARK_COMMON_H_

#include <vector>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

void replaceUnusedArgumentsOfLandmarkCostFunctionWithDummies(
    ceres_error_terms::LandmarkErrorType error_term_type,
    std::vector<double*>* error_term_argument_list,
    std::vector<double*>* dummies_to_set_constant);

}  // namespace ceres_error_terms

#include "ceres-error-terms/landmark-common-inl.h"

#endif  // CERES_ERROR_TERMS_LANDMARK_COMMON_H_
