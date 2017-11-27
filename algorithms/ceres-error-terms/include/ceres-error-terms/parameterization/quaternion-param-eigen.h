#ifndef CERES_ERROR_TERMS_PARAMETERIZATION_QUATERNION_PARAM_EIGEN_H_
#define CERES_ERROR_TERMS_PARAMETERIZATION_QUATERNION_PARAM_EIGEN_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace ceres_error_terms {

class EigenQuaternionParameterization : public ceres::LocalParameterization {
 public:
  static constexpr int kLocalSize = 3;
  static constexpr int kGlobalSize = 4;

  // The representation for Jacobian computed by this object.
  typedef Eigen::Matrix<double, kGlobalSize, kLocalSize, Eigen::RowMajor>
      Jacobian;
  typedef Eigen::Matrix<double, kLocalSize, kGlobalSize, Eigen::RowMajor>
      LiftJacobian;
  virtual ~EigenQuaternionParameterization() {}
  virtual bool Plus(
      const double* x, const double* delta, double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const {
    return kGlobalSize;
  }
  virtual int LocalSize() const {
    return kLocalSize;
  }

  // Calculates the difference on the manifold:
  // error = q1 boxminus q2
  bool Minus(const double* x, const double* y, double* x_minus_y) const;

  bool ComputeLiftJacobian(const double* x, double* jacobian) const;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_PARAMETERIZATION_QUATERNION_PARAM_EIGEN_H_
