#ifndef CERES_ERROR_TERMS_VISUAL_ERROR_TERM_FACTORY_H_
#define CERES_ERROR_TERMS_VISUAL_ERROR_TERM_FACTORY_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera.h>

namespace ceres_error_terms {

template <template <typename, typename> class ErrorTerm>
ceres::CostFunction* createVisualCostFunction(
    const Eigen::Vector2d& measurement, double pixel_sigma,
    ceres_error_terms::LandmarkErrorType error_term_type,
    aslam::Camera* camera);

}  // namespace ceres_error_terms

#include "ceres-error-terms/visual-error-term-factory-inl.h"

#endif  // CERES_ERROR_TERMS_VISUAL_ERROR_TERM_FACTORY_H_
