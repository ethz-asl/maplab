#ifndef MAPLAB_COMMON_GEOMETRY_INL_H_
#define MAPLAB_COMMON_GEOMETRY_INL_H_

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace common {
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(
    const Eigen::MatrixBase<Derived>& vector) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

  Eigen::Matrix<typename Derived::Scalar, 3, 3> output;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  output << zero, -vector[2], vector[1], vector[2], zero, -vector[0],
      -vector[1], vector[0], zero;
  return output;
}

template <typename Derived, typename OtherDerived>
void skew(
    const Eigen::MatrixBase<Derived>& vector,
    Eigen::MatrixBase<OtherDerived> const& matrix_const) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(OtherDerived, 3, 3);

  typedef typename OtherDerived::Scalar Scalar;

  Eigen::MatrixBase<OtherDerived>& matrix =
      const_cast<Eigen::MatrixBase<OtherDerived>&>(matrix_const);

  Scalar zero = static_cast<Scalar>(0.0);

  matrix.derived() << zero, -vector[2], vector[1], vector[2], zero, -vector[0],
      -vector[1], vector[0], zero;
}

// Conversion from rotation matrix to roll, pitch, yaw angles.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
RotationMatrixToRollPitchYaw(const Eigen::MatrixBase<Derived>& rot) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  Eigen::Matrix<typename Derived::Scalar, 3, 1> rpy;
  rpy(1, 0) =
      atan2(-rot(2, 0), sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));
  if (std::abs(cos(rpy(1, 0))) >
      static_cast<typename Derived::Scalar>(1.0e-12)) {
    rpy(2, 0) = atan2(rot(1, 0) / cos(rpy(1, 0)), rot(0, 0) / cos(rpy(1, 0)));
    rpy(0, 0) = atan2(rot(2, 1) / cos(rpy(1, 0)), rot(2, 2) / cos(rpy(1, 0)));
  } else if (sin(rpy(1, 0)) > static_cast<typename Derived::Scalar>(0)) {
    rpy(2, 0) = static_cast<typename Derived::Scalar>(0);
    rpy(0, 0) = atan2(rot(0, 1), rot(1, 1));
  } else {
    rpy(2, 0) = static_cast<typename Derived::Scalar>(0);
    rpy(0, 0) = -atan2(rot(0, 1), rot(1, 1));
  }
  return rpy;
}

// Conversion from roll, pitch, yaw to rotation matrix.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
RollPitchYawToRotationMatrix(const Eigen::MatrixBase<Derived>& roll_pitch_yaw) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_x;
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0.);
  typename Derived::Scalar one = static_cast<typename Derived::Scalar>(1.);
  rotation_matrix_x << one, zero, zero, zero, cos(roll_pitch_yaw(0)),
      -sin(roll_pitch_yaw(0)), zero, sin(roll_pitch_yaw(0)),
      cos(roll_pitch_yaw(0));

  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_y;
  rotation_matrix_y << cos(roll_pitch_yaw(1)), zero, sin(roll_pitch_yaw(1)),
      zero, one, zero, -sin(roll_pitch_yaw(1)), 0, cos(roll_pitch_yaw(1));

  Eigen::Matrix<typename Derived::Scalar, 3, 3> rotation_matrix_z;
  rotation_matrix_z << cos(roll_pitch_yaw(2)), -sin(roll_pitch_yaw(2)), zero,
      sin(roll_pitch_yaw(2)), cos(roll_pitch_yaw(2)), zero, zero, zero, one;

  return rotation_matrix_z * rotation_matrix_y * rotation_matrix_x;
}

// Skew-symmetric (cross-product) matrix.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetricMatrix(
    const Eigen::MatrixBase<Derived>& x) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 1);
  typename Derived::Scalar zero = static_cast<typename Derived::Scalar>(0);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> s;
  s << zero, -x(2), x(1), x(2), zero, -x(0), -x(1), x(0), zero;
  return s;
}
// Compute left multiplication matrix.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4>
LeftQuaternionJPLMultiplicationMatrix(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4, 1);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> L;

  L.template block<3, 3>(0, 0) =
      q(3) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
      SkewSymmetricMatrix(q.template block<3, 1>(0, 0));
  L.template block<3, 1>(0, 3) = q.template block<3, 1>(0, 0);
  L.template block<1, 3>(3, 0) = -q.template block<3, 1>(0, 0).transpose();
  L(3, 3) = q(3);
  return L;
}

template <template <typename, typename> class Container>
void transformationRansac(
    const Container<pose::Transformation,
                    Eigen::aligned_allocator<pose::Transformation>>&
        T_A_B_samples,
    int num_iterations, double threshold_orientation_radians,
    double threshold_position_meters, int ransac_seed,
    pose::Transformation* T_A_B, int* num_inliers) {
  CHECK_NOTNULL(T_A_B);
  CHECK_NOTNULL(num_inliers);
  CHECK(!T_A_B_samples.empty());
  if (T_A_B_samples.size() == 1u) {
    *T_A_B = T_A_B_samples[0];
    *num_inliers = 1;
    return;
  }
  std::mt19937 generator(ransac_seed);
  std::uniform_int_distribution<> distribution(0, T_A_B_samples.size() - 1);

  std::vector<int> best_inlier_indices;
  best_inlier_indices.push_back(0);

  // Run RANSAC on the transformations. Every transformation is a hypothesis,
  // validation is performed by comparing the position and orientation offset.
  for (int i = 0; i < num_iterations; ++i) {
    const int sample_index = distribution(generator);
    const pose::Transformation& T_A_B_sample = T_A_B_samples[sample_index];
    std::vector<int> inlier_indices;
    for (size_t j = 0; j < T_A_B_samples.size(); ++j) {
      const pose::Transformation& T_A_B_other = T_A_B_samples[j];
      const double p_norm_A_A_hat =
          (T_A_B_sample.getPosition() - T_A_B_other.getPosition()).norm();
      const double q_angle_A_A_hat =
          2. *
          acos(
              (T_A_B_sample.getRotation() * T_A_B_other.getRotation().inverse())
                  .w());
      if (p_norm_A_A_hat < threshold_position_meters &&
          q_angle_A_A_hat < threshold_orientation_radians) {
        inlier_indices.emplace_back(j);
      }
    }
    if (inlier_indices.size() > best_inlier_indices.size()) {
      best_inlier_indices.swap(inlier_indices);
    }
  }

  CHECK(!best_inlier_indices.empty());

  // Run a least-squares refinement on all inliers.
  VectorOfJPLQuaternia q_A_B_inliers;
  q_A_B_inliers.reserve(best_inlier_indices.size());
  Eigen::Vector3d p_A_B_inliers = Eigen::Vector3d::Zero();
  for (int inlier_index : best_inlier_indices) {
    const pose::Transformation& T_A_B_sample = T_A_B_samples[inlier_index];
    q_A_B_inliers.emplace_back(
        T_A_B_sample.getRotation().toImplementation().coeffs());
    p_A_B_inliers += T_A_B_sample.getPosition();
  }

  Eigen::Vector4d q_A_B_JPL = ComputeLSAverageQuaternionJPL(q_A_B_inliers);
  T_A_B->getRotation().toImplementation().coeffs() = q_A_B_JPL;
  T_A_B->getPosition() = p_A_B_inliers / best_inlier_indices.size();
  *num_inliers = best_inlier_indices.size();
}

namespace geometry {

template <typename Type, int Dimensions>
void computeCovariance(
    const Eigen::Matrix<Type, Dimensions, Eigen::Dynamic>& cloud,
    Eigen::Matrix<Type, Dimensions, Dimensions>* result) {
  CHECK_NOTNULL(result);
  result->setZero(cloud.rows(), cloud.rows());
  constexpr int kBlockSize = 10000;
  const int num_blocks = cloud.cols() / kBlockSize + 1;
  for (int i = 0; i < num_blocks; ++i) {
    const int block_start = i * kBlockSize;
    const int block_size =
        std::min<int>((i + 1) * kBlockSize, cloud.cols()) - block_start;
    const Eigen::Block<const Eigen::Matrix<Type, Dimensions, Eigen::Dynamic>>&
        data_block = cloud.block(0, block_start, cloud.rows(), block_size);

    const Eigen::Matrix<Type, Dimensions, Eigen::Dynamic> centered =
        data_block.colwise() - data_block.rowwise().mean();
    double normalizer = std::max(static_cast<int>(data_block.cols() - 1), 1);
    result->noalias() += (centered * centered.adjoint()) / normalizer;
  }
  (*result) /= num_blocks;
}

// Leverages self-adjoint property of covariance for speed. Takes the point
// cloud and returns values and vecors sorted in ascending order by value.
template <typename Type, int Dimensions>
void computeCovarianceEigenValuesAndVectors(
    const Eigen::Matrix<Type, Dimensions, Eigen::Dynamic>& cloud,
    Eigen::Matrix<Type, Dimensions, 1>* values,
    Eigen::Matrix<Type, Dimensions, Dimensions>* vectors) {
  CHECK_NOTNULL(values);
  CHECK_NOTNULL(vectors);
  Eigen::Matrix<Type, Dimensions, Dimensions> covariance;
  computeCovariance(cloud, &covariance);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Type, Dimensions, Dimensions>>
      eigen_comp(covariance);
  *values = eigen_comp.eigenvalues();
  *vectors = eigen_comp.eigenvectors();
}

}  // namespace geometry

}  // namespace common

#endif  // MAPLAB_COMMON_GEOMETRY_INL_H_
