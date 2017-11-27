#ifndef PARAMETERIZATION_QUATERNION_PARAM_HAMILTON_H_
#define PARAMETERIZATION_QUATERNION_PARAM_HAMILTON_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

class HamiltonQuaternionParameterization : public ceres::LocalParameterization {
 public:
  virtual ~HamiltonQuaternionParameterization() {}
  virtual bool Plus(
      const double* x, const double* delta, double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const {
    return 4;
  }
  virtual int LocalSize() const {
    return 3;
  }
};

class HamiltonYawOnlyQuaternionParameterization
    : public ceres::LocalParameterization {
 public:
  virtual ~HamiltonYawOnlyQuaternionParameterization() {}
  virtual bool Plus(
      const double* x, const double* delta, double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const {
    return 4;
  }
  virtual int LocalSize() const {
    return 1;
  }
};

}  // namespace ceres_error_terms

#endif  // PARAMETERIZATION_QUATERNION_PARAM_HAMILTON_H_
