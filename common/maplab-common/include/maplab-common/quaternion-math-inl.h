#ifndef MAPLAB_COMMON_QUATERNION_MATH_INL_H_
#define MAPLAB_COMMON_QUATERNION_MATH_INL_H_

namespace common {

template <typename Derived, typename ScalarType>
void toRotationMatrixJPL(
    const Eigen::MatrixBase<Derived>& q,
    Eigen::Matrix<ScalarType, 3, 3>* rot_matrix) {
  CHECK_NOTNULL(rot_matrix);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 4);

  ScalarType one = static_cast<ScalarType>(1.0);
  ScalarType two = static_cast<ScalarType>(2.0);

  (*rot_matrix)(0, 0) = one - two * (q(1) * q(1) + q(2) * q(2));
  (*rot_matrix)(0, 1) = two * (q(0) * q(1) + q(2) * q(3));
  (*rot_matrix)(0, 2) = two * (q(0) * q(2) - q(1) * q(3));

  (*rot_matrix)(1, 0) = two * (q(0) * q(1) - q(2) * q(3));
  (*rot_matrix)(1, 1) = one - two * (q(0) * q(0) + q(2) * q(2));
  (*rot_matrix)(1, 2) = two * (q(1) * q(2) + q(0) * q(3));

  (*rot_matrix)(2, 0) = two * (q(0) * q(2) + q(1) * q(3));
  (*rot_matrix)(2, 1) = two * (q(1) * q(2) - q(0) * q(3));
  (*rot_matrix)(2, 2) = one - two * (q(0) * q(0) + q(1) * q(1));
}

// Conversion from quaternion to euler angles.
template <typename ScalarType>
Eigen::Matrix<ScalarType, 3, 1> getRollPitchYawFromQuaternion(
    const Eigen::Quaternion<ScalarType>& q) {
  Eigen::Matrix<ScalarType, 3, 1> rpy_rad;
  rpy_rad(0) = std::atan2(
      2.0 * (q.w() * q.x() + q.y() * q.z()),
      1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  rpy_rad(1) = std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  rpy_rad(2) = std::atan2(
      2.0 * (q.w() * q.z() + q.x() * q.y()),
      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return rpy_rad;
}

// Conversion from quaternion to euler angles.
template <typename ScalarType>
Eigen::Matrix<ScalarType, 3, 1> getRollPitchYawFromQuaternionJpl(
    const Eigen::Quaternion<ScalarType>& q_AB) {
  Eigen::Matrix<ScalarType, 3, 3> R_AB;
  common::toRotationMatrixJPL(q_AB.coeffs(), &R_AB);
  return common::RotationMatrixToRollPitchYaw(R_AB);
}

// Conversion from rotation matrix to quaternion JPL (xyzw).
template <typename Derived, typename ScalarType>
void fromRotationMatrixJPL(
    const Eigen::MatrixBase<Derived>& rot,
    Eigen::Matrix<ScalarType, 4, 1>* q_JPL) {
  CHECK_NOTNULL(q_JPL);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);

  ScalarType one = static_cast<ScalarType>(1.0);
  ScalarType four = static_cast<ScalarType>(4.0);

  double T = rot.trace();
  if ((rot(0, 0) > T) && (rot(0, 0) > rot(1, 1)) && (rot(0, 0) > rot(2, 2))) {
    (*q_JPL)(0) = sqrt((one + (2 * rot(0, 0)) - T) / four);
    (*q_JPL)(1) = (one / (four * (*q_JPL)(0))) * (rot(0, 1) + rot(1, 0));
    (*q_JPL)(2) = (one / (four * (*q_JPL)(0))) * (rot(0, 2) + rot(2, 0));
    (*q_JPL)(3) = (one / (four * (*q_JPL)(0))) * (rot(1, 2) - rot(2, 1));
  } else if (
      (rot(1, 1) > T) && (rot(1, 1) > rot(0, 0)) && (rot(1, 1) > rot(2, 2))) {
    (*q_JPL)(1) = sqrt((one + (2 * rot(1, 1)) - T) / four);
    (*q_JPL)(0) = (one / (four * (*q_JPL)(1))) * (rot(0, 1) + rot(1, 0));
    (*q_JPL)(2) = (one / (four * (*q_JPL)(1))) * (rot(1, 2) + rot(2, 1));
    (*q_JPL)(3) = (one / (four * (*q_JPL)(1))) * (rot(2, 0) - rot(0, 2));
  } else if (
      (rot(2, 2) > T) && (rot(2, 2) > rot(0, 0)) && (rot(2, 2) > rot(1, 1))) {
    (*q_JPL)(2) = sqrt((one + (2 * rot(2, 2)) - T) / four);
    (*q_JPL)(0) = (one / (four * (*q_JPL)(2))) * (rot(0, 2) + rot(2, 0));
    (*q_JPL)(1) = (one / (four * (*q_JPL)(2))) * (rot(1, 2) + rot(2, 1));
    (*q_JPL)(3) = (one / (four * (*q_JPL)(2))) * (rot(0, 1) - rot(1, 0));
  } else {
    (*q_JPL)(3) = sqrt((one + T) / four);
    (*q_JPL)(0) = (one / (four * (*q_JPL)(3))) * (rot(1, 2) - rot(2, 1));
    (*q_JPL)(1) = (one / (four * (*q_JPL)(3))) * (rot(2, 0) - rot(0, 2));
    (*q_JPL)(2) = (one / (four * (*q_JPL)(3))) * (rot(0, 1) - rot(1, 0));
  }
  if ((*q_JPL)(3) < 0.0) {
    (*q_JPL) = -(*q_JPL);
  }
  q_JPL->normalize();
}

template <typename ScalarType>
inline Eigen::Matrix<ScalarType, 4, 1> rotationVectorToQuaternionJPL(
    const Eigen::Matrix<ScalarType, 3, 1>& A_rotation_vector_BA) {
  constexpr ScalarType _0 = static_cast<ScalarType>(0);
  constexpr ScalarType _1 = static_cast<ScalarType>(1);
  constexpr ScalarType _2 = static_cast<ScalarType>(2);

  const ScalarType theta = A_rotation_vector_BA.norm();
  Eigen::Matrix<ScalarType, 4, 1> q_BA;
  if (theta < static_cast<ScalarType>(1e-10)) {
    q_BA << _0, _0, _0, _1;
  } else {
    q_BA << A_rotation_vector_BA / theta * std::sin(theta / _2),
        std::cos(theta / _2);
  }
  return q_BA;
}

template <typename Derived1, typename Derived2, typename Derived3>
void signedQuaternionProductJPL(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2,
    const Eigen::MatrixBase<Derived3>& product_const) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived3, 4);

  typedef typename Derived1::Scalar Scalar;

  // Construct left multiplication matrix.
  Eigen::Matrix<Scalar, 4, 4> matrix_L;
  Eigen::Matrix<Scalar, 3, 3> skew_q1;
  common::skew(q1.head(3), skew_q1);
  matrix_L.block(0, 0, 3, 3) =
      q1.derived()(3) * Eigen::Matrix3d::Identity().cast<Scalar>() - skew_q1;
  matrix_L.block(3, 0, 1, 3) = -q1.derived().head(3).transpose();
  matrix_L.block(0, 3, 4, 1) = q1.derived();

  Eigen::MatrixBase<Derived3>& product =
      const_cast<Eigen::MatrixBase<Derived3>&>(product_const);

  product.derived().resize(4, 1);
  product.derived() = matrix_L * q2;
  product.derived().normalize();
}

template <typename Derived1, typename Derived2, typename Derived3>
void positiveQuaternionProductJPL(
    const Eigen::MatrixBase<Derived1>& q1,
    const Eigen::MatrixBase<Derived2>& q2,
    const Eigen::MatrixBase<Derived3>& product_const) {
  signedQuaternionProductJPL(q1, q2, product_const);

  // Make sure the scalar part is positive.
  typedef typename Derived3::Scalar Scalar;
  Eigen::MatrixBase<Derived3>& product =
      const_cast<Eigen::MatrixBase<Derived3>&>(product_const);
  if (product(3) < static_cast<Scalar>(0.)) {
    product = -product;
  }
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> quaternionInverseJPL(
    const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 1> inverse_q = q;
  inverse_q.template block<3, 1>(0, 0) = -q.template block<3, 1>(0, 0);
  return inverse_q;
}

template <typename Scalar>
kindr::minimal::RotationQuaternionTemplate<Scalar>
signedQuaternionProductHamilton(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& lhs,
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& rhs) {
  return kindr::minimal::RotationQuaternionTemplate<Scalar>(
      internal::internalSignedQuaternionProductHamilton(
          rhs.toImplementation(), lhs.toImplementation()));
}

template <typename Scalar>
kindr::minimal::RotationQuaternionTemplate<Scalar>
positiveQuaternionProductHamilton(
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& lhs,
    const kindr::minimal::RotationQuaternionTemplate<Scalar>& rhs) {
  kindr::minimal::RotationQuaternionTemplate<Scalar> product =
      signedQuaternionProductHamilton(lhs, rhs);

  if (product.w() < static_cast<Scalar>(0.)) {
    product.toImplementation().coeffs() = -product.toImplementation().coeffs();
  }

  return product;
}

inline double getYawAngleDifferenceRadians(
    const kindr::minimal::QuatTransformation& T_A_B1,
    const kindr::minimal::QuatTransformation& T_A_B2) {
  const kindr::minimal::QuatTransformation T_B1_B2 = T_A_B1.inverse() * T_A_B2;
  const kindr::minimal::AngleAxis angle_axis_B1_B2(T_B1_B2.getRotation());
  double angle_rad = angle_axis_B1_B2.angle();
  CHECK_GE(angle_rad, 0.0);
  CHECK_LT(angle_rad, 2.0 * M_PI);
  if (angle_rad > M_PI) {
    angle_rad = 2.0 * M_PI - angle_rad;
  }
  CHECK_GE(angle_rad, 0.0);
  CHECK_LT(angle_rad, M_PI);
  const Eigen::Vector3d angle_axis_B1_B2_scaled_axis =
      angle_axis_B1_B2.axis() * angle_rad;

  return std::fabs(angle_axis_B1_B2_scaled_axis(2));
}

namespace internal {

template <typename Scalar>
Eigen::Quaternion<Scalar> internalSignedQuaternionProductHamilton(
    const Eigen::Quaternion<Scalar>& lhs,
    const Eigen::Quaternion<Scalar>& rhs) {
  return Eigen::Quaternion<Scalar>(
      lhs.w() * rhs.w() - lhs.x() * rhs.x() - lhs.y() * rhs.y() -
          lhs.z() * rhs.z(),
      lhs.w() * rhs.x() + lhs.x() * rhs.w() + lhs.y() * rhs.z() -
          lhs.z() * rhs.y(),
      lhs.w() * rhs.y() - lhs.x() * rhs.z() + lhs.y() * rhs.w() +
          lhs.z() * rhs.x(),
      lhs.w() * rhs.z() + lhs.x() * rhs.y() - lhs.y() * rhs.x() +
          lhs.z() * rhs.w());
}

}  // namespace internal

namespace eigen_quaternion_helpers {
const int kLocalSize = 3;
const int kGlobalSize = 4;

template <typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> Gamma(
    const Eigen::Matrix<Scalar, 3, 1>& phi) {
  const Scalar phi_squared_norm = phi.squaredNorm();

  if (phi_squared_norm < 1e-6) {
    Eigen::Matrix<Scalar, 3, 3> gamma;
    gamma.setIdentity();
    gamma += 0.5 * common::skew(phi);
    return gamma;
  }
  const Scalar phi_norm = sqrt(phi_squared_norm);
  const Eigen::Matrix<Scalar, 3, 3> phi_skew(common::skew(phi));

  Eigen::Matrix<Scalar, 3, 3> gamma;
  gamma.setIdentity();
  gamma += ((1.0 - cos(phi_norm)) / phi_squared_norm) * phi_skew;
  const Scalar phi_cubed = (phi_norm * phi_squared_norm);
  gamma += ((phi_norm - sin(phi_norm)) / phi_cubed) * phi_skew * phi_skew;
  return gamma;
}

template <typename Scalar>
inline Eigen::Quaternion<Scalar> ExpMap(
    const Eigen::Matrix<Scalar, 3, 1>& theta) {
  const Scalar theta_squared_norm = theta.squaredNorm();

  if (theta_squared_norm < 1e-6) {
    Eigen::Quaternion<Scalar> q(
        1, theta(0) * 0.5, theta(1) * 0.5, theta(2) * 0.5);
    q.normalize();
    return q;
  }

  const Scalar theta_norm = sqrt(theta_squared_norm);
  const Eigen::Matrix<Scalar, 3, 1> q_imag =
      sin(theta_norm * 0.5) * theta / theta_norm;
  Eigen::Quaternion<Scalar> q(
      cos(theta_norm * 0.5), q_imag(0), q_imag(1), q_imag(2));
  return q;
}

inline Eigen::Quaterniond ExpMap(const Eigen::Vector3d& theta) {
  return ExpMap<double>(theta);
}

inline Eigen::Vector3d LogMap(const Eigen::Quaterniond& q) {
  const Eigen::Block<const Eigen::Vector4d, 3, 1> q_imag = q.vec();
  const double q_imag_squared_norm = q_imag.squaredNorm();

  if (q_imag_squared_norm < 1e-6) {
    return 2 * copysign(1, q.w()) * q_imag;
  }

  const double q_imag_norm = sqrt(q_imag_squared_norm);
  Eigen::Vector3d q_log = 2 * atan2(q_imag_norm, q.w()) * q_imag / q_imag_norm;
  return q_log;
}

// Rotates the quaternion p with the error theta on the tangent space:
// p_plus_theta = p boxplus theta
// TODO(burrimi): Check if there is a better way to pass mapped quaternions.
inline void Plus(
    const Eigen::Ref<const Eigen::Vector4d>& p,
    const Eigen::Ref<const Eigen::Vector3d>& theta,
    Eigen::Quaterniond* p_plus_theta) {
  CHECK_NOTNULL(p_plus_theta);
  const Eigen::Map<const Eigen::Quaterniond> p_mapped(p.data());
  *p_plus_theta = ExpMap(theta) * p_mapped;
}

// Calculates the shortest connection respecting the manifold structure:
// theta = p boxminus q
// TODO(burrimi): Modify to also allow Eigen::Map types.
// TODO(burrimi): Extend this function to also return Jacobians to reuse
// computation.
inline void Minus(
    const Eigen::Quaterniond& p, const Eigen::Quaterniond& q,
    Eigen::Vector3d* p_minus_q) {
  CHECK_NOTNULL(p_minus_q);
  *p_minus_q = LogMap(p * q.inverse());
}

// Calculates the Jacobian of the boxminus operator w.r.t. the two orientations
// p and q to properly account for the manifold structure.
// Reminder: theta = p boxminus q
// TODO(burrimi): Modify to also allow Eigen::Map types.
// TODO(burrimi): Deprecate this function and extend Minus() to also
// return Jacobians to reuse computation.
inline void GetBoxminusJacobians(
    const Eigen::Quaterniond& p, const Eigen::Quaterniond& q,
    Eigen::Matrix3d* J_boxminus_wrt_p, Eigen::Matrix3d* J_boxminus_wrt_q) {
  if (J_boxminus_wrt_p == NULL && J_boxminus_wrt_q == NULL) {
    return;  // Nothing to do.
  }

  Eigen::Vector3d theta;
  Minus(p, q, &theta);

  const Eigen::Matrix3d Gamma_inverse = Gamma(theta).inverse();

  if (J_boxminus_wrt_p != NULL) {
    *J_boxminus_wrt_p = Gamma_inverse;
  }

  if (J_boxminus_wrt_q != NULL) {
    const Eigen::Quaterniond delta_orientation = p * q.inverse();
    *J_boxminus_wrt_q = -Gamma_inverse * delta_orientation.toRotationMatrix();
  }
}

}  // namespace eigen_quaternion_helpers

}  // namespace common

#endif  // MAPLAB_COMMON_QUATERNION_MATH_INL_H_
