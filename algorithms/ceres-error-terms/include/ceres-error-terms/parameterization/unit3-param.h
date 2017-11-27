#ifndef CERES_ERROR_TERMS_PARAMETERIZATION_UNIT3_PARAM_H_
#define CERES_ERROR_TERMS_PARAMETERIZATION_UNIT3_PARAM_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

#include <maplab-common/quaternion-math.h>
#include "ceres-error-terms/parameterization/quaternion-param-eigen.h"

namespace ceres_error_terms {

// Implementation according to
// Bloesch et al - IEKF-based Visual-Inertial Odometry using Direct Photometric
// Feedback
// We use a quaternion q to represent a unit vector

namespace Unit3 {
const int kLocalSize = 2;
const int kGlobalSize = 4;

inline void GetFromVector(
    const Eigen::Vector3d& vector, Eigen::Quaterniond* unit_vector) {
  *unit_vector = vector.norm() < 1e-12 ? Eigen::Quaterniond::Identity()
                                       : Eigen::Quaterniond::FromTwoVectors(
                                             Eigen::Vector3d(0, 0, 1), vector);
}

// The unit vector is given by the R(q) * e_z
// The tangent space(basis) is given by R(q) * e_x and R * e_y
inline void GetNormalVectorAndBasis(
    const Eigen::Quaterniond& q, Eigen::Vector3d* normal_vector,
    Eigen::Matrix<double, 3, 2>* basis) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  if (normal_vector != nullptr) {
    *normal_vector = R.block<3, 1>(0, 2);
  }
  if (basis != nullptr) {
    *basis = R.block<3, 2>(0, 0);
  }
}

inline Eigen::Vector3d GetNormalVector(const Eigen::Quaterniond& q) {
  return q.toRotationMatrix().block<3, 1>(0, 2);
}

inline Eigen::Vector3d GetBasis1(const Eigen::Quaterniond& q) {
  return q.toRotationMatrix().block<3, 1>(0, 0);
}

inline Eigen::Vector3d GetBasis2(const Eigen::Quaterniond& q) {
  return q.toRotationMatrix().block<3, 1>(0, 1);
}

inline Eigen::Matrix<double, 3, 2> GetBasis(const Eigen::Quaterniond& q) {
  return q.toRotationMatrix().block<3, 2>(0, 0);
}

// TODO(burrimi): Modify to also allow Eigen::Map types.
// Rotates the unit1 vector with the error respecting the manifold structure:
// unit2 = unit1 boxplus error
inline bool Plus(
    const Eigen::Quaterniond& unit1, const Eigen::Vector2d& error,
    Eigen::Quaterniond* unit2) {
  CHECK_NOTNULL(unit2);
  *unit2 = common::eigen_quaternion_helpers::ExpMap(
               error(0) * GetBasis1(unit1) + error(1) * GetBasis2(unit1)) *
           unit1;
  return true;
}

// TODO(burrimi): Modify to also allow Eigen::Map types.
// Calculates the shortest connection respecting the manifold structure:
// error = unit1 boxminus unit2
inline bool Minus(
    const Eigen::Quaterniond& unit1, const Eigen::Quaterniond& unit2,
    Eigen::Vector2d* error) {
  CHECK_NOTNULL(error);
  const Eigen::Vector3d error_angle = common::eigen_quaternion_helpers::LogMap(
      Eigen::Quaterniond::FromTwoVectors(
          GetNormalVector(unit2), GetNormalVector(unit1)));
  *error = GetBasis(unit2).transpose() * error_angle;
  return true;
}
}  // namespace Unit3

class Unit3Parameterization : public ceres::LocalParameterization {
 public:
  // The representation for Jacobian computed by this object.
  typedef Eigen::Matrix<
      double, Unit3::kGlobalSize, Unit3::kLocalSize, Eigen::RowMajor>
      Jacobian;
  typedef Eigen::Matrix<
      double, Unit3::kLocalSize, Unit3::kGlobalSize, Eigen::RowMajor>
      LiftJacobian;

  virtual ~Unit3Parameterization() {}
  virtual bool Plus(
      const double* x, const double* delta, double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const {
    return Unit3::kGlobalSize;
  }
  virtual int LocalSize() const {
    return Unit3::kLocalSize;
  }

  // Calculates the difference on the manifold:
  // error = unit1 boxminus unit2
  bool Minus(const double* x, const double* y, double* x_minus_y) const;

  bool ComputeLiftJacobian(const double* x, double* jacobian) const;

 private:
};

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_PARAMETERIZATION_UNIT3_PARAM_H_
