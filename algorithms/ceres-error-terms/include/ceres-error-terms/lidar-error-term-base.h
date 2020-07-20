#ifndef CERES_ERROR_TERMS_LIDAR_ERROR_TERM_BASE_H_
#define CERES_ERROR_TERMS_LIDAR_ERROR_TERM_BASE_H_
#include <ceres/sized_cost_function.h>
#include <glog/logging.h>

namespace ceres_error_terms {
class LidarCostFunction {
 public:
  explicit LidarCostFunction(double pixel_sigma) {
    CHECK_GT(pixel_sigma, 0);
    pixel_sigma_inverse_ = 1.0 / pixel_sigma;
  }

  virtual ~LidarCostFunction() {}

  double sigma() const {
    return 1.0 / pixel_sigma_inverse_;
  }

 protected:
  double pixel_sigma_inverse_;
};
}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_LIDAR_ERROR_TERM_BASE_H_
