#ifndef CERES_ERROR_TERMS_PARAMETERIZATION_UNIT_PARAM_H_
#define CERES_ERROR_TERMS_PARAMETERIZATION_UNIT_PARAM_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace ceres_error_terms {

/*class UnitVectorParameterization : public ceres::LocalParameterization {
 public:
  UnitVectorParameterization(int n) : n_(n){};

  bool Plus(const double* x_data, const double* p_data, double* y_data) const {
    Eigen::Map<const Eigen::VectorXd> x(x_data, n_);
    Eigen::Map<const Eigen::VectorXd> p(p_data, n_);
    Eigen::Map<Eigen::VectorXd> y(y_data, n_);

    Eigen::VectorXd z = x + p;
    y = 1. / z.norm() * z * x.norm();

    return true;
  }

  bool ComputeJacobian(const double* x_data, double* dy_dx_data) const {
    // Ceres uses row-major for Jacobians.
    typedef Eigen::Matrix<
        double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        Jacobian;
    Eigen::Map<const Eigen::VectorXd> x(x_data, n_);
    // Jacobian is n x n.
    Eigen::Map<Jacobian> dy_dx(dy_dx_data, n_, n_);
    dy_dx.setZero();

    double norm_x = x.norm();
    double norm_x3 = norm_x * norm_x * norm_x;

    for (int i = 0; i < n_; i += 1) {
      for (int j = 0; j < n_; j += 1) {
        if (i == j) {
          dy_dx(i, j) = 1; //1. / norm_x - x(i) * x(i) / norm_x3;
        } else {
          dy_dx(i, j) = 0; //-x(i) * x(j) / norm_x3;
        }
      }
    }

    return true;
  }

  int GlobalSize() const {
    return n_;
  }

  int LocalSize() const {
    return n_;
  }

 private:
  int n_;
};*/

struct UnitPlus {
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_delta) const {
    const T x_norm =
        ceres::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3]);

    x_delta[0] = x[0] + delta[0];
    x_delta[1] = x[1] + delta[1];
    x_delta[2] = x[2] + delta[2];
    x_delta[3] = x[3] + delta[3];

    const T x_delta_norm = ceres::sqrt(
        x_delta[0] * x_delta[0] + x_delta[1] * x_delta[1] +
        x_delta[2] * x_delta[2] + x_delta[3] * x_delta[3]);

    x_delta[0] = x_delta[0] / x_delta_norm * x_norm;
    x_delta[1] = x_delta[1] / x_delta_norm * x_norm;
    x_delta[2] = x_delta[2] / x_delta_norm * x_norm;
    x_delta[3] = x_delta[3] / x_delta_norm * x_norm;

    return true;
  }
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_PARAMETERIZATION_UNIT_PARAM_H_
