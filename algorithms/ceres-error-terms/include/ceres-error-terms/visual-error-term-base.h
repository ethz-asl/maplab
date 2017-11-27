#ifndef CERES_ERROR_TERMS_VISUAL_ERROR_TERM_BASE_H_
#define CERES_ERROR_TERMS_VISUAL_ERROR_TERM_BASE_H_
#include <ceres/sized_cost_function.h>
#include <glog/logging.h>

namespace ceres_error_terms {
class VisualCostFunction {
 public:
  explicit VisualCostFunction(double pixel_sigma) {
    CHECK_GT(pixel_sigma, 0);
    pixel_sigma_inverse_ = 1.0 / pixel_sigma;
  }

  virtual ~VisualCostFunction() {}

  double sigma() const {
    return 1.0 / pixel_sigma_inverse_;
  }

 protected:
  double pixel_sigma_inverse_;
};
}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_VISUAL_ERROR_TERM_BASE_H_
