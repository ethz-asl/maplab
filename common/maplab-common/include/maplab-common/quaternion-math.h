#ifndef MAPLAB_COMMON_QUATERNION_MATH_H_
#define MAPLAB_COMMON_QUATERNION_MATH_H_

#include <Eigen/Core>

#include <maplab-common/conversions.h>
#include <maplab-common/geometry.h>
#include <maplab-common/pose_types.h>

namespace common {

template <typename Derived, typename ScalarType>
void toRotationMatrixJPL(
    const Eigen::MatrixBase<Derived>& q,
    Eigen::Matrix<ScalarType, 3, 3>* rot_matrix);

template <typename Derived, typename ScalarType>
void fromRotationMatrixJPL(
    const Eigen::MatrixBase<Derived>& rot,
    Eigen::Matrix<ScalarType, 4, 1>* q_JPL);

template <typename ScalarType>
void rotationVectorToQuaternionJPL(
    const Eigen::Matrix<ScalarType, 3, 1>& A_rotation_vector,
    const Eigen::Matrix<ScalarType, 4, 1>& q_A_Ahat_const);

template <typename Derived1, typename Derived2, typename Derived3>
void signedQuaternionProductJPL(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2,
    const Eigen::MatrixBase<Derived3>& product_const);

template <typename Derived1, typename Derived2, typename Derived3>
void positiveQuaternionProductJPL(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2,
    const Eigen::MatrixBase<Derived3>& product_const);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> quaternionInverseJPL(
    const Eigen::MatrixBase<Derived>& q);

template <typename Scalar>
kindr::minimal::RotationQuaternionTemplate<Scalar>
signedQuaternionProductHamilton(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& lhs,
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& rhs);

template <typename Scalar>
kindr::minimal::RotationQuaternionTemplate<Scalar>
positiveQuaternionProductHamilton(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& lhs,
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& rhs);

template <typename Derived, typename ScalarType>
Eigen::Matrix<ScalarType, 3, 1> getRollPitchYawFromQuaternion(
    const Eigen::Quaternion<ScalarType>& q);

inline double getYawAngleDifferenceRadians(
    const kindr::minimal::QuatTransformation& T_A_B1,
    const kindr::minimal::QuatTransformation& T_A_B2);

namespace eigen_quaternion_helpers {

template <typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> Gamma(
    const Eigen::Matrix<Scalar, 3, 1>& phi);
template <typename Scalar>
inline Eigen::Quaternion<Scalar> ExpMap(
    const Eigen::Matrix<Scalar, 3, 1>& theta);

inline Eigen::Quaterniond ExpMap(const Eigen::Vector3d& theta);
inline Eigen::Vector3d LogMap(const Eigen::Quaterniond& q);

// Rotates the quaternion p with the error theta on the tangent space:
// p_plus_theta = p boxplus theta
// TODO(schneith): Check if there is a better way to pass mapped quaternions.
inline void Plus(
    const Eigen::Ref<const Eigen::Vector4d>& p,
    const Eigen::Ref<const Eigen::Vector3d>& theta,
    Eigen::Quaterniond* p_plus_theta);

// Calculates the shortest connection respecting the manifold structure:
// theta = p boxminus q
// TODO(schneith): Modify to also allow Eigen::Map types.
// TODO(schneith): Extend this function to also return Jacobians to reuse
// computation.
inline void Minus(
    const Eigen::Quaterniond& p, const Eigen::Quaterniond& q,
    Eigen::Vector3d* p_minus_q);

// Calculates the Jacobian of the boxminus operator w.r.t. the two orientations
// p and q to properly account for the manifold structure.
// Reminder: theta = p boxminus q
// TODO(schneith): Modify to also allow Eigen::Map types.
// TODO(schneith): Deprecate this function and extend Minus() to also
// return Jacobians to reuse computation.
inline void GetBoxminusJacobians(
    const Eigen::Quaterniond& p, const Eigen::Quaterniond& q,
    Eigen::Matrix3d* J_boxminus_wrt_p, Eigen::Matrix3d* J_boxminus_wrt_q);

}  // namespace eigen_quaternion_helpers

namespace internal {

template <typename Scalar>
Eigen::Quaternion<Scalar> internalSignedQuaternionProductHamilton(
    const Eigen::Quaternion<Scalar>& lhs, const Eigen::Quaternion<Scalar>& rhs);

}  // namespace internal

}  // namespace common

#include "./quaternion-math-inl.h"

#endif  // MAPLAB_COMMON_QUATERNION_MATH_H_
