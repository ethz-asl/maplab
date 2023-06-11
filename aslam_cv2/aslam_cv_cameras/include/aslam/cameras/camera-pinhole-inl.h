#ifndef ASLAM_CAMERAS_PINHOLE_CAMERA_INL_H_
#define ASLAM_CAMERAS_PINHOLE_CAMERA_INL_H_

#include <memory>

namespace aslam {

// TODO(dymczykm) actually, I'm not sure if it wouldn't be better if
// specialized versions of these function (for double) would use this
// implementation instead of repeating it twice. But that would mean we need
// to template the whole class on ScalarType (or at least intrinsics and few
// methods).

template <typename ScalarType, typename DistortionType,
          typename MIntrinsics, typename MDistortion>
const ProjectionResult PinholeCamera::project3Functional(
    const Eigen::Matrix<ScalarType, 3, 1>& point_3d,
    const Eigen::MatrixBase<MIntrinsics>& intrinsics_external,
    const Eigen::MatrixBase<MDistortion>& distortion_coefficients_external,
    Eigen::Matrix<ScalarType, 2, 1>* out_keypoint) const {

  CHECK_NOTNULL(out_keypoint);
  CHECK_EQ(intrinsics_external.size(), kNumOfParams) << "intrinsics: invalid size!";

  const ScalarType& fu = intrinsics_external[0];
  const ScalarType& fv = intrinsics_external[1];
  const ScalarType& cu = intrinsics_external[2];
  const ScalarType& cv = intrinsics_external[3];

  ScalarType rz = static_cast<ScalarType>(1.0) / point_3d[2];
  Eigen::Matrix<ScalarType, 2, 1> keypoint;
  keypoint[0] = point_3d[0] * rz;
  keypoint[1] = point_3d[1] * rz;

  // Distort the point (if a distortion model is set)
  const DistortionType& distortion =
      static_cast<const DistortionType&>(*distortion_);
  distortion.distortUsingExternalCoefficients(
      distortion_coefficients_external, keypoint, &keypoint);

  (*out_keypoint)[0] = fu * keypoint[0] + cu;
  (*out_keypoint)[1] = fv * keypoint[1] + cv;

  return evaluateProjectionResult(*out_keypoint, point_3d);
}

template <typename DerivedKeyPoint, typename DerivedPoint3d>
inline const ProjectionResult PinholeCamera::evaluateProjectionResult(
    const Eigen::MatrixBase<DerivedKeyPoint>& keypoint,
    const Eigen::MatrixBase<DerivedPoint3d>& point_3d) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedPoint3d, 3, 1);

  Eigen::Matrix<typename DerivedKeyPoint::Scalar, 2, 1> kp = keypoint;
  const bool visibility = isKeypointVisible(kp);

  if (visibility && (point_3d[2] > kMinimumDepth))
    return ProjectionResult(ProjectionResult::Status::KEYPOINT_VISIBLE);
  else if (!visibility && (point_3d[2] > kMinimumDepth))
    return ProjectionResult(ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX);
  else if (point_3d[2] < 0.0)
    return ProjectionResult(ProjectionResult::Status::POINT_BEHIND_CAMERA);
  else
    return ProjectionResult(ProjectionResult::Status::PROJECTION_INVALID);
}

}  // namespace aslam
#endif  // ASLAM_CAMERAS_PINHOLE_CAMERA_INL_H_
