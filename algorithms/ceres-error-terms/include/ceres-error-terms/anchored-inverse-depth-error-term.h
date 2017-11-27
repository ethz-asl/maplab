#ifndef CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_ERROR_TERM_H_
#define CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_ERROR_TERM_H_

#include <memory>

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>
#include <maplab-common/pose_types.h>

#include "ceres-error-terms/common.h"
#include "ceres-error-terms/parameterization/quaternion-param-jpl.h"
#include "ceres-error-terms/parameterization/unit3-param.h"
#include "ceres-error-terms/visual-error-term-base.h"

namespace ceres_error_terms {

// Note: this error term accepts rotations expressed as quaternions
// in JPL convention [x, y, z, w]. This convention corresponds to the internal
// coefficient storage of Eigen so you can directly pass pointer to your
// Eigen quaternion data, e.g. your_eigen_quaternion.coeffs().data().
template <
    typename CameraType, typename DistortionType,
    visual::VisualErrorType error_term_type>
class AIDReprojectionError
    : public ceres::SizedCostFunction<
          visual::kResidualSize, visual::kPositionBlockSize,
          visual::kUnit3BlockSize, visual::kInverseDepthBlockSize,
          visual::kOrientationBlockSize, visual::kPositionBlockSize,
          visual::kOrientationBlockSize, visual::kPositionBlockSize,
          CameraType::parameterCount(), DistortionType::parameterCount()>,
      public VisualCostFunction {
 public:
  typedef VisualCostFunction Base;

  // Construct a cost function representing the reprojection error. Sigma is
  // standard deviation (in pixels).
  AIDReprojectionError(
      const Eigen::Vector2d& measurement, double pixel_sigma,
      const CameraType* camera)
      : Base(pixel_sigma), measurement_(measurement), camera_ptr_(camera) {
    CHECK(camera);
  }

  virtual ~AIDReprojectionError() {}

  virtual bool Evaluate(
      double const* const* parameters, double* residuals,
      double** jacobians) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Don't change the ordering of the enum elements, they have to be the
  // same as the order of the parameter blocks.
  enum {
    kIdxAnchorPoint,
    kIdxUnitVector,
    kIdxInverseDepth,
    kIdxImuQ,
    kIdxImuP,
    kIdxCameraToImuQ,
    kIdxCameraToImuP,
    kIdxCameraIntrinsics,
    kIdxCameraDistortion
  };

  // The representation for Jacobians computed by this object.
  typedef Eigen::Matrix<
      double, visual::kResidualSize, visual::kOrientationBlockSize,
      Eigen::RowMajor>
      OrientationJacobian;

  typedef Eigen::Matrix<
      double, visual::kResidualSize, visual::kUnit3BlockSize, Eigen::RowMajor>
      Unit3Jacobian;

  typedef Eigen::Matrix<
      double, visual::kResidualSize, visual::kInverseDepthBlockSize>
      InverseDepthJacobian;

  typedef Eigen::Matrix<
      double, visual::kResidualSize, visual::kPositionBlockSize,
      Eigen::RowMajor>
      PositionJacobian;

  typedef Eigen::Matrix<
      double, visual::kResidualSize, visual::kPoseBlockSize, Eigen::RowMajor>
      PoseJacobian;

  typedef Eigen::Matrix<
      double, visual::kResidualSize, CameraType::parameterCount(),
      Eigen::RowMajor>
      IntrinsicsJacobian;

  typedef Eigen::Matrix<
      double, visual::kResidualSize, Eigen::Dynamic, Eigen::RowMajor>
      DistortionJacobian;

  typedef Eigen::Matrix<double, visual::kResidualSize, Eigen::Dynamic>
      JacobianWrtIntrinsicsType;

  typedef Eigen::Matrix<double, visual::kResidualSize, Eigen::Dynamic>
      JacobianWrtDistortionType;

  typedef Eigen::Matrix<
      double, visual::kResidualSize, visual::kPositionBlockSize>
      VisualJacobianType;

  Eigen::Vector2d measurement_;
  const CameraType* camera_ptr_;
};

}  // namespace ceres_error_terms

#include "./ceres-error-terms/anchored-inverse-depth-error-term-inl.h"

#endif  // CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_ERROR_TERM_H_
