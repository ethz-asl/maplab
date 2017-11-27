#ifndef CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_ERROR_TERM_INL_H_
#define CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_ERROR_TERM_INL_H_

#include <aslam/cameras/camera.h>
#include <maplab-common/geometry.h>
#include <maplab-common/quaternion-math.h>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

template <visual::VisualErrorType error_type>
struct is_valid_error_type {
  static const bool value =
      (error_type == visual::VisualErrorType::kLocalKeyframe ||
       error_type == visual::VisualErrorType::kLocalMission ||
       error_type == visual::VisualErrorType::kGlobal);
};

template <
    typename CameraType, typename DistortionType,
    visual::VisualErrorType error_term_type>
bool AIDReprojectionError<CameraType, DistortionType, error_term_type>::
    Evaluate(
        double const* const* parameters, double* residuals,
        double** jacobians) const {
  // Coordinate frames:
  //  M = mission of the keyframe vertex, expressed in G
  //  A = Anchor point of landmark (position of first observation), expressed in
  //  M
  //  I = IMU position of the keyframe vertex, expressed in M
  //  C = Camera position, expressed in I

  // TODO(burrimi): is this still needed?
  static_assert(
      is_valid_error_type<error_term_type>::value,
      "Visual error type must be a valid enum value.");

  // Unpack parameter blocks.
  Eigen::Map<const Eigen::Vector3d> M_p_M_A(parameters[kIdxAnchorPoint]);
  Eigen::Map<const Eigen::Quaterniond> A_u_A_L(parameters[kIdxUnitVector]);
  const double inverse_depth_A = *parameters[kIdxInverseDepth];

  Eigen::Map<const Eigen::Quaterniond> q_M_I(parameters[kIdxImuQ]);
  Eigen::Map<const Eigen::Vector3d> M_p_M_I(parameters[kIdxImuP]);
  Eigen::Map<const Eigen::Quaterniond> q_C_I(parameters[kIdxCameraToImuQ]);
  Eigen::Map<const Eigen::Vector3d> C_p_C_I(parameters[kIdxCameraToImuP]);
  Eigen::Map<const Eigen::Matrix<double, CameraType::parameterCount(), 1>>
      intrinsics_map(parameters[kIdxCameraIntrinsics]);
  Eigen::Matrix<double, Eigen::Dynamic, 1> distortion_map;

  if (DistortionType::parameterCount() > 0) {
    distortion_map = Eigen::Map<
        const Eigen::Matrix<double, DistortionType::parameterCount(), 1>>(
        parameters[kIdxCameraDistortion]);
  }

  // Jacobian of landmark pose in camera system w.r.t. keyframe pose
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_q_M_I;
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_M_I;

  // Jacobian of landmark pose in camera system w.r.t. cam-to-IMU transformation
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_q_C_I;
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_C_I;

  // Jacobians w.r.t. camera intrinsics and distortion coefficients
  JacobianWrtIntrinsicsType J_keypoint_wrt_intrinsics(
      visual::kResidualSize, CameraType::parameterCount());
  JacobianWrtDistortionType J_keypoint_wrt_distortion(
      visual::kResidualSize, DistortionType::parameterCount());

  // To avoid division by zero (landmarks at infinity) we calculate a scaled
  // version of p_C_fi.
  // This is identical to using the non scaled value due to the projection to
  // the image plane.
  // C_p_CL_scaled = C_p_CL * inverse_depth_A =
  //   R_CI * R_MI' * (inverse_depth_A * (M_p_MA - M_p_MI) + A_u_AL) +
  //   inverse_depth_A * C_p_CI

  // Do the calculations and cache some intermediate steps for the Jacobians
  const Eigen::Matrix3d R_C_M = (q_C_I * q_M_I.inverse()).toRotationMatrix();
  const Eigen::Vector3d M_p_I_A = M_p_M_A - M_p_M_I;
  const Eigen::Vector3d A_u_A_L_normal_vector = Unit3::GetNormalVector(A_u_A_L);

  // Note: R_MA is identity and left out.
  const Eigen::Vector3d M_p_I_L_scaled =
      inverse_depth_A * M_p_I_A + A_u_A_L_normal_vector;
  const Eigen::Vector3d C_p_I_L_scaled = R_C_M * M_p_I_L_scaled;
  // Finally we get the landmark in the camera frame
  const Eigen::Vector3d C_p_C_L_scaled =
      C_p_I_L_scaled + inverse_depth_A * C_p_C_I;

  // TODO(burrimi): Is this needed in the future?
  //  if (error_term_type == visual::VisualErrorType::kLocalKeyframe) {
  //    // The landmark baseframe is in fact our keyframe.
  //  } else {
  //  }

  VisualJacobianType J_keypoint_wrt_p_C_fi;
  // TODO(schneith): is this copy needed?
  Eigen::VectorXd intrinsics = intrinsics_map;
  Eigen::VectorXd distortion = distortion_map;

  // Only evaluate the jacobian if requested.
  VisualJacobianType* J_keypoint_wrt_p_C_fi_ptr = nullptr;
  if (jacobians) {
    J_keypoint_wrt_p_C_fi_ptr = &J_keypoint_wrt_p_C_fi;
  }
  JacobianWrtIntrinsicsType* J_keypoint_wrt_intrinsics_ptr = nullptr;
  if (jacobians && jacobians[kIdxCameraIntrinsics]) {
    J_keypoint_wrt_intrinsics_ptr = &J_keypoint_wrt_intrinsics;
  }
  JacobianWrtDistortionType* J_keypoint_wrt_distortion_ptr = nullptr;
  if (jacobians && DistortionType::parameterCount() > 0 &&
      jacobians[kIdxCameraDistortion]) {
    J_keypoint_wrt_distortion_ptr = &J_keypoint_wrt_distortion;
  }

  Eigen::Vector2d reprojected_landmark;
  aslam::ProjectionResult projection_result;
  projection_result = camera_ptr_->project3Functional(
      C_p_C_L_scaled, &intrinsics, &distortion, &reprojected_landmark,
      J_keypoint_wrt_p_C_fi_ptr, J_keypoint_wrt_intrinsics_ptr,
      J_keypoint_wrt_distortion_ptr);

  // TODO(schneith): This is not really required for the optimization!
  if (!projection_result.isKeypointVisible()) {
    VLOG(200) << "Projection of point failed " << projection_result;
  }

  if (jacobians) {
    // Eigen quaternion parameterization is used
    EigenQuaternionParameterization quat_parameterization;
    Unit3Parameterization unit3_parameterization;

    // Jacobian of landmark pose in camera system w.r.t. inverse depth feature
    // parameterization
    const Eigen::Matrix3d J_p_C_fi_wrt_p_M_A = inverse_depth_A * R_C_M;
    Eigen::Matrix<double, visual::kPositionBlockSize, Unit3::kLocalSize>
        J_p_C_fi_wrt_u_A_L = -R_C_M * common::skew(A_u_A_L_normal_vector) *
                             Unit3::GetBasis(A_u_A_L);
    const Eigen::Vector3d J_p_C_fi_wrt_inv_depth = R_C_M * M_p_I_A + C_p_C_I;

    // Jacobian of landmark pose in camera system w.r.t. inverse depth feature
    // parameterization
    const Eigen::Matrix3d J_p_C_fi_wrt_q_M_I =
        R_C_M * common::skew(M_p_I_L_scaled);
    const Eigen::Matrix3d J_p_C_fi_wrt_p_M_I = -J_p_C_fi_wrt_p_M_A;

    // Jacobian of landmark pose in camera system w.r.t. inverse depth feature
    // parameterization
    const Eigen::Matrix3d J_p_C_fi_wrt_q_C_I = -common::skew(C_p_I_L_scaled);
    const Eigen::Matrix3d J_p_C_fi_wrt_p_C_I =
        inverse_depth_A * Eigen::Matrix3d::Identity();

    // Jacobian w.r.t. landmark position expressed in landmark base frame.
    if (jacobians[kIdxAnchorPoint]) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxAnchorPoint]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_M_A *
          this->pixel_sigma_inverse_;
    }

    // Jacobian w.r.t. landmark position expressed in landmark base frame.
    if (jacobians[kIdxUnitVector]) {
      // we need to have 3x4 matrix that will be a jacobian of 2-element
      // unit3 w.r.t. quaternion; note that ComputeJacobians needs row major
      Unit3Parameterization::LiftJacobian lift_jacobian;
      unit3_parameterization.ComputeLiftJacobian(
          parameters[kIdxUnitVector], lift_jacobian.data());
      Eigen::Map<Unit3Jacobian> J(jacobians[kIdxUnitVector]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_u_A_L * lift_jacobian *
          this->pixel_sigma_inverse_;
    }

    // Jacobian w.r.t. landmark position expressed in landmark base frame.
    if (jacobians[kIdxInverseDepth]) {
      Eigen::Map<InverseDepthJacobian> J(jacobians[kIdxInverseDepth]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_inv_depth *
          this->pixel_sigma_inverse_;
    }

    // Jacobian w.r.t. IMU pose expressed in keyframe mission base frame.
    if (jacobians[kIdxImuQ]) {
      // we need to have 3x4 matrix that will be a jacobian of 3-element
      // rotation w.r.t. quaternion; note that ComputeJacobians needs row major
      EigenQuaternionParameterization::LiftJacobian lift_jacobian;
      quat_parameterization.ComputeLiftJacobian(
          parameters[kIdxImuQ], lift_jacobian.data());
      Eigen::Map<OrientationJacobian> J(jacobians[kIdxImuQ]);

      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_q_M_I * lift_jacobian *
          this->pixel_sigma_inverse_;
    }

    if (jacobians[kIdxImuP]) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxImuP]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_M_I *
          this->pixel_sigma_inverse_;
    }

    // Jacobian w.r.t. camera-to-IMU orientation.
    if (jacobians[kIdxCameraToImuQ]) {
      EigenQuaternionParameterization::LiftJacobian lift_jacobian;
      quat_parameterization.ComputeLiftJacobian(
          parameters[kIdxCameraToImuQ], lift_jacobian.data());
      Eigen::Map<OrientationJacobian> J(jacobians[kIdxCameraToImuQ]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_q_C_I * lift_jacobian *
          this->pixel_sigma_inverse_;
    }

    // Jacobian w.r.t. camera-to-IMU position.
    if (jacobians[kIdxCameraToImuP]) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxCameraToImuP]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_C_I *
          this->pixel_sigma_inverse_;
    }

    // Jacobian w.r.t. intrinsics.
    if (jacobians[kIdxCameraIntrinsics]) {
      Eigen::Map<IntrinsicsJacobian> J(jacobians[kIdxCameraIntrinsics]);
      J = J_keypoint_wrt_intrinsics * this->pixel_sigma_inverse_;
    }

    // Jacobian w.r.t. distortion.
    if (DistortionType::parameterCount() > 0 &&
        jacobians[kIdxCameraDistortion]) {
      Eigen::Map<DistortionJacobian> J(
          jacobians[kIdxCameraDistortion], visual::kResidualSize,
          DistortionType::parameterCount());
      J = J_keypoint_wrt_distortion * this->pixel_sigma_inverse_;
    }
  }

  // Compute residuals.
  Eigen::Map<Eigen::Vector2d> residual(residuals);
  residual = (reprojected_landmark - measurement_) * this->pixel_sigma_inverse_;

  return true;
}

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_ANCHORED_INVERSE_DEPTH_ERROR_TERM_INL_H_
