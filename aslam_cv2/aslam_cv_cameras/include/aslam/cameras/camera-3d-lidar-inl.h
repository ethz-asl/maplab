#ifndef ASLAM_CAMERAS_CAMERA_3D_LIDAR_INL_H_
#define ASLAM_CAMERAS_CAMERA_3D_LIDAR_INL_H_

#include <memory>

namespace aslam {

template <
    typename ScalarType, typename DistortionType, typename MIntrinsics,
    typename MDistortion>
const ProjectionResult Camera3DLidar::project3Functional(
    const Eigen::Matrix<ScalarType, 3, 1>& point_3d,
    const Eigen::MatrixBase<MIntrinsics>& intrinsics_external,
    const Eigen::MatrixBase<MDistortion>& /*distortion_coefficients_external*/,
    Eigen::Matrix<ScalarType, 2, 1>* out_keypoint) const {
  CHECK_NOTNULL(out_keypoint);
  CHECK_EQ(intrinsics_external.size(), kNumOfParams)
      << "intrinsics: invalid size!";

  if (point_3d.norm() < 1e-6) {
    (*out_keypoint)[0] = 0;
    (*out_keypoint)[1] = 0;
    return ProjectionResult(ProjectionResult::Status::PROJECTION_INVALID);
  }

  const Eigen::Matrix<ScalarType, 3, 1> bearing_vector = point_3d.normalized();

  (*out_keypoint)[0] = (std::atan2(bearing_vector.x(), bearing_vector.z()) +
                        horizontalCenter()) /
                       horizontalResolution();

  if ((*out_keypoint)[0] < 0.0) {
    (*out_keypoint)[0] = imageWidth() + (*out_keypoint)[0];
  }

  (*out_keypoint)[1] =
      ((std::asin(bearing_vector.y()) + verticalCenter()) /
       verticalResolution());

  return evaluateProjectionResult(*out_keypoint, point_3d);
}

template <typename DerivedKeyPoint, typename DerivedPoint3d>
inline const ProjectionResult Camera3DLidar::evaluateProjectionResult(
    const Eigen::MatrixBase<DerivedKeyPoint>& keypoint,
    const Eigen::MatrixBase<DerivedPoint3d>& point_3d) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedPoint3d, 3, 1);

  Eigen::Matrix<typename DerivedKeyPoint::Scalar, 2, 1> kp = keypoint;
  const bool is_visible = isKeypointVisible(kp);
  const double squaredNorm = point_3d.squaredNorm();
  const bool is_outside_min_depth = squaredNorm > kSquaredMinimumDepth;

  if (is_visible && is_outside_min_depth) {
    return ProjectionResult(ProjectionResult::Status::KEYPOINT_VISIBLE);
  } else if (!is_visible && is_outside_min_depth) {
    return ProjectionResult(
        ProjectionResult::Status::KEYPOINT_OUTSIDE_IMAGE_BOX);
  } else {
    return ProjectionResult(ProjectionResult::Status::PROJECTION_INVALID);
  }
}

}  // namespace aslam
#endif  // ASLAM_CAMERAS_CAMERA_3D_LIDAR_INL_H_
