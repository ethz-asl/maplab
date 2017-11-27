#ifndef CERES_ERROR_TERMS_PARAMETERIZATION_QUATERNION_PARAM_JPL_H_
#define CERES_ERROR_TERMS_PARAMETERIZATION_QUATERNION_PARAM_JPL_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace ceres_error_terms {

class JplQuaternionParameterization : public ceres::LocalParameterization {
 public:
  virtual ~JplQuaternionParameterization() {}
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

// Yaw is defined as rotations around the z-axis of a left multiplied frame.
// E.g. When using the pose q_AB a yaw increment corresponds to a rotation
// around the A_[0,0,1] axis: q_AB + delta_yaw  = q_AhatA(delta_yaw) * qAB
class JplYawQuaternionParameterization : public ceres::LocalParameterization {
 public:
  virtual ~JplYawQuaternionParameterization() {}
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

// Rotation parameterization that keeps global yaw fixed; yaw = rotation around
// G_[0;0;1]. The poses are composed by: q_GI = q_GM * q_MI and thus the
// parameterization requires access to both q_GM and q_MI, which is not
// supported by ceres. Luckily the baseframe q_GM is constant for VI problems,
// for which we need this parameterization. Therefore the (constant) baseframe
// transformation is set using the constructor.
class JplRollPitchQuaternionParameterization
    : public ceres::LocalParameterization {
 public:
  JplRollPitchQuaternionParameterization(
      const Eigen::Matrix<double, 4, 1>& q_G_M_JPL);
  virtual ~JplRollPitchQuaternionParameterization() {}
  virtual bool Plus(
      const double* q_I_M, const double* delta, double* q_I_M_plus_delta) const;

  virtual bool ComputeJacobian(const double* q_I_M, double* jacobian) const;
  virtual int GlobalSize() const {
    return 4;
  }
  virtual int LocalSize() const {
    return 2;
  }

 private:
  Eigen::Matrix3d R_M_G_;
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_PARAMETERIZATION_QUATERNION_PARAM_JPL_H_
