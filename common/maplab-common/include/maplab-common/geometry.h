#ifndef MAPLAB_COMMON_GEOMETRY_H_
#define MAPLAB_COMMON_GEOMETRY_H_
#include <random>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>

namespace common {
typedef Aligned<std::vector, Eigen::Matrix<double, 4, 1>> VectorOfJPLQuaternia;
typedef Aligned<std::vector, Eigen::Matrix<double, 3, 1>> VectorOfPositions;

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(
    const Eigen::MatrixBase<Derived>& vector);

template <typename Derived, typename OtherDerived>
void skew(
    const Eigen::MatrixBase<Derived>& vector,
    Eigen::MatrixBase<OtherDerived> const& matrix_const);

// "xyz" must be a unit vector.
inline Eigen::Vector2d xyzToPhiTheta(const Eigen::Vector3d& xyz) {
  return Eigen::Vector2d(atan2(xyz(1), xyz(0)), asin(xyz(2)));
}

inline Eigen::Vector3d phiThetaToXyz(const Eigen::Vector2d& pt) {
  const double cos_theta = cos(pt(1));
  return Eigen::Vector3d(
      cos(pt(0)) * cos_theta, sin(pt(0)) * cos_theta, sin(pt(1)));
}

// Conversion from rotation matrix to roll, pitch, yaw angles.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
RotationMatrixToRollPitchYaw(const Eigen::MatrixBase<Derived>& rot);

// Conversion from roll, pitch, yaw to rotation matrix.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
RollPitchYawToRotationMatrix(const Eigen::MatrixBase<Derived>& roll_pitch_yaw);

// Skew-symmetric (cross-product) matrix.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetricMatrix(
    const Eigen::MatrixBase<Derived>& x);
// Compute left multiplication matrix.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4>
LeftQuaternionJPLMultiplicationMatrix(const Eigen::MatrixBase<Derived>& q);

// Utility method for computing the least-squares average quaternion from a
// vector of quaternia. Derivation is in Appendix A of "Mirror-Based Extrinsic
// Camera Calibration," Hesch et al. WAFR 2008.
Eigen::Matrix<double, 4, 1> ComputeLSAverageQuaternionJPL(
    const VectorOfJPLQuaternia& Gl_q_Gc_vector);

template <template <typename, typename> class Container>
void transformationRansac(
    const Container<pose::Transformation,
                    Eigen::aligned_allocator<pose::Transformation>>&
        T_A_B_samples,
    int num_iterations, double threshold_orientation_radians,
    double threshold_position_meters, int ransac_seed,
    pose::Transformation* T_A_B, int* num_inliers);

namespace geometry {

// Implementation adopted from the descriptor_projection package.
template <typename Type, int Dimensions>
void computeCovariance(
    const Eigen::Matrix<Type, Dimensions, Eigen::Dynamic>& cloud,
    Eigen::Matrix<Type, Dimensions, Dimensions>* result);

// Leverages self-adjoint property of covariance for speed. Takes the point
// cloud and returns values and vecors sorted in ascending order by value.
template <typename Type, int Dimensions>
void computeCovarianceEigenValuesAndVectors(
    const Eigen::Matrix<Type, Dimensions, Eigen::Dynamic>& cloud,
    Eigen::Matrix<Type, Dimensions, 1>* values,
    Eigen::Matrix<Type, Dimensions, Dimensions>* vectors);

pose::Transformation yawOnly(const pose::Transformation& original);

}  // namespace geometry

}  // namespace common

#include "./geometry-inl.h"

#endif  // MAPLAB_COMMON_GEOMETRY_H_
