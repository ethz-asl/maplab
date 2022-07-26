#ifndef CERES_ERROR_TERMS_LIDAR_ERROR_TERM_H_
#define CERES_ERROR_TERMS_LIDAR_ERROR_TERM_H_

#include <memory>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>
#include <maplab-common/pose_types.h>

#include "ceres-error-terms/common.h"
#include "ceres-error-terms/parameterization/quaternion-param-jpl.h"

namespace ceres_error_terms {

// Note: this error term accepts rotations expressed as quaternions
// in JPL convention [x, y, z, w]. This convention corresponds to the internal
// coefficient storage of Eigen so you can directly pass pointer to your
// Eigen quaternion data, e.g. your_eigen_quaternion.coeffs().data().
class LidarLandmarkError
    : public ceres::SizedCostFunction<
          lidar::kResidualSize, lidar::kPositionBlockSize,
          lidar::kPoseBlockSize, lidar::kPoseBlockSize, lidar::kPoseBlockSize,
          lidar::kPoseBlockSize, lidar::kOrientationBlockSize,
          lidar::kPositionBlockSize> {
 public:
  // Construct a cost function representing the LiDAR landmark error.
  LidarLandmarkError(
      const Eigen::Vector3d& measurement, double sigma,
      LandmarkErrorType error_term_type)
      : measurement_(measurement), error_term_type_(error_term_type) {
    CHECK_GT(sigma, 0);
    CHECK(isValidLandmarkErrorTermType(error_term_type_));
    sigma_inverse_ = 1.0 / sigma;
  }

  virtual ~LidarLandmarkError() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Don't change the ordering of the enum elements, they have to be the
  // same as the order of the parameter blocks.
  enum {
    kIdxLandmarkP,
    kIdxLandmarkBasePose,
    kIdxLandmarkMissionBasePose,
    kIdxImuMissionBasePose,
    kIdxImuPose,
    kIdxLidarToImuQ,
    kIdxLidarToImuP
  };

  // The representation for Jacobians computed by this object.
  typedef Eigen::Matrix<
      double, lidar::kResidualSize, lidar::kOrientationBlockSize,
      Eigen::RowMajor>
      OrientationJacobian;

  typedef Eigen::Matrix<
      double, lidar::kResidualSize, lidar::kPositionBlockSize, Eigen::RowMajor>
      PositionJacobian;

  typedef Eigen::Matrix<
      double, lidar::kResidualSize, lidar::kPoseBlockSize, Eigen::RowMajor>
      PoseJacobian;

  Eigen::Vector3d measurement_;
  double sigma_inverse_;
  const LandmarkErrorType error_term_type_;
};

}  // namespace ceres_error_terms

#include "ceres-error-terms/lidar-error-term-inl.h"

#endif  // CERES_ERROR_TERMS_LIDAR_ERROR_TERM_H_
