#ifndef CERES_ERROR_TERMS_LIDAR_ERROR_TERM_BASE_H_
#define CERES_ERROR_TERMS_LIDAR_ERROR_TERM_BASE_H_

#include <ceres/sized_cost_function.h>

namespace ceres_error_terms {

class LidarCostFunction {
 public:
  explicit LidarCostFunction(const double pixel_sigma)
      : pixel_sigma_inverse_(1.0 / pixel_sigma) {
    CHECK_GT(pixel_sigma, 0.0);
  }

  virtual ~LidarCostFunction() = default;

  double sigma() const {
    return 1.0 / pixel_sigma_inverse_;
  }

 protected:
  const double pixel_sigma_inverse_;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_LIDAR_ERROR_TERM_BASE_H_
