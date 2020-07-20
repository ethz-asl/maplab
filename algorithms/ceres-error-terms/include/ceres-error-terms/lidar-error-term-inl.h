#ifndef CERES_ERROR_TERMS_LIDAR_ERROR_TERM_INL_H_
#define CERES_ERROR_TERMS_LIDAR_ERROR_TERM_INL_H_

#include <aslam/cameras/camera.h>
#include <maplab-common/geometry.h>
#include <maplab-common/quaternion-math.h>

#include "ceres-error-terms/common.h"

namespace ceres_error_terms {

template <typename CameraType, typename DistortionType>
bool LidarPositionError<CameraType, DistortionType>::Evaluate(
    double const* const* parameters, double* residuals,
    double** jacobians) const {
  // Coordinate frames:
  //  G = global
  //  M = mission of the keyframe vertex, expressed in G
  //  LM = mission of the landmark-base vertex, expressed in G
  //  B = base vertex of the landmark, expressed in LM
  //  I = IMU position of the keyframe vertex, expressed in M
  //  C = Camera position, expressed in I

  // Unpack parameter blocks.
  Eigen::Map<const Eigen::Vector3d> p_B_fi(parameters[kIdxLandmarkP]);
  Eigen::Map<const Eigen::Quaterniond> q_B_LM(parameters[kIdxLandmarkBasePose]);
  Eigen::Map<const Eigen::Vector3d> p_LM_B(
      parameters[kIdxLandmarkBasePose] + visual::kOrientationBlockSize);
  Eigen::Map<const Eigen::Quaterniond> q_G_LM(
      parameters[kIdxLandmarkMissionBasePose]);
  Eigen::Map<const Eigen::Vector3d> p_G_LM(
      parameters[kIdxLandmarkMissionBasePose] + visual::kOrientationBlockSize);
  Eigen::Map<const Eigen::Quaterniond> q_G_M(
      parameters[kIdxImuMissionBasePose]);
  Eigen::Map<const Eigen::Vector3d> p_G_M(
      parameters[kIdxImuMissionBasePose] + visual::kOrientationBlockSize);
  Eigen::Map<const Eigen::Quaterniond> q_I_M(parameters[kIdxImuPose]);
  Eigen::Map<const Eigen::Vector3d> p_M_I(
      parameters[kIdxImuPose] + visual::kOrientationBlockSize);
  Eigen::Map<const Eigen::Quaterniond> q_C_I(parameters[kIdxCameraToImuQ]);
  Eigen::Map<const Eigen::Vector3d> p_C_I(parameters[kIdxCameraToImuP]);
  Eigen::Map<const Eigen::Matrix<double, CameraType::parameterCount(), 1> >
      intrinsics_map(parameters[kIdxCameraIntrinsics]);

  // Jacobian of landmark pose in camera system w.r.t. keyframe pose
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_q_I_M;
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_M_I;

  // Jacobian of landmark pose in camera system w.r.t. keyframe mission pose
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_q_G_M;
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_G_M;

  // Jacobian of landmark pose in camera system w.r.t. landmark mission pose
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_q_G_q_LM;
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_G_p_LM;

  // Jacobian of landmark pose in camera system w.r.t. cam-to-IMU transformation
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_q_C_I;
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_C_I;

  // Jacobian of landmark pose in camera system w.r.t. landmark base pose
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_q_B_LM;
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_LM_B;

  // Jacobian of landmark pose in camera system w.r.t. landmark position
  Eigen::Matrix<double, visual::kPositionBlockSize, 3> J_p_C_fi_wrt_p_B_fi;

  // Jacobians w.r.t. camera intrinsics coefficients
  typedef Eigen::Matrix<double, visual::kLidarResidualSize, Eigen::Dynamic>
      JacobianWrtIntrinsicsType;
  JacobianWrtIntrinsicsType J_keypoint_wrt_intrinsics(
      visual::kLidarResidualSize, CameraType::parameterCount());

  Eigen::Matrix3d R_B_LM, R_LM_B;
  Eigen::Matrix3d R_G_LM;
  Eigen::Matrix3d R_G_M, R_M_G;
  Eigen::Matrix3d R_I_M;
  Eigen::Matrix3d R_C_I;
  common::toRotationMatrixJPL(q_C_I.coeffs(), &R_C_I);

  Eigen::Vector3d p_M_fi = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_G_fi = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_I_fi = Eigen::Vector3d::Zero();
  if (error_term_type_ == visual::VisualErrorType::kLocalKeyframe) {
    // The landmark baseframe is in fact our keyframe.
    p_I_fi = p_B_fi;
  } else {
    common::toRotationMatrixJPL(q_B_LM.coeffs(), &R_B_LM);
    R_LM_B = R_B_LM.transpose();
    common::toRotationMatrixJPL(q_I_M.coeffs(), &R_I_M);
    if (error_term_type_ == visual::VisualErrorType::kLocalMission) {
      // In this case M == LM.
      p_M_fi = R_LM_B * p_B_fi + p_LM_B;
      p_I_fi = R_I_M * (p_M_fi - p_M_I);
    } else if (error_term_type_ == visual::VisualErrorType::kGlobal) {
      common::toRotationMatrixJPL(q_G_LM.coeffs(), &R_G_LM);
      common::toRotationMatrixJPL(q_G_M.coeffs(), &R_G_M);
      R_M_G = R_G_M.transpose();

      const Eigen::Vector3d p_LM_fi = R_LM_B * p_B_fi + p_LM_B;
      p_G_fi = R_G_LM * p_LM_fi + p_G_LM;
      p_M_fi = R_M_G * (p_G_fi - p_G_M);
      p_I_fi = R_I_M * (p_M_fi - p_M_I);
    } else {
      LOG(FATAL) << "Unknown visual error term type.";
    }
  }
  // Note that: p_C_fi = R_C_I * (p_I_fi - p_I_C)
  // And: p_I_C = - R_C_I.transpose() * p_C_I
  const Eigen::Vector3d p_C_fi = R_C_I * p_I_fi + p_C_I;

  // Jacobian of 3d keypoint (including intrinsics)
  // w.r.t. to landmark position in camera coordinates
  Eigen::Vector3d reprojected_landmark = p_C_fi;
  typedef Eigen::Matrix<
      double, visual::kLidarResidualSize, visual::kPositionBlockSize>
      LidarJacobianType;
  LidarJacobianType J_keypoint_wrt_p_C_fi = Eigen::Matrix3d::Identity();
  Eigen::VectorXd intrinsics = intrinsics_map;
  if (jacobians) {
    // JPL quaternion parameterization is used because our memory layout
    // of quaternions is JPL.
    JplQuaternionParameterization quat_parameterization;

    // These Jacobians will be used in all the cases.
    J_p_C_fi_wrt_p_C_I = Eigen::Matrix3d::Identity();
    J_p_C_fi_wrt_q_C_I = common::skew(p_C_fi);

    if (error_term_type_ == visual::VisualErrorType::kGlobal) {
      // The following commented expressions are evaluated in an optimized way
      // below and provided here in comments for better readability.
      // J_p_C_fi_wrt_p_M_I = -R_C_I * R_I_M;
      // J_p_C_fi_wrt_p_G_M = -R_C_I * R_I_M * R_M_G;
      // J_p_C_fi_wrt_p_G_p_LM = R_C_I * R_I_M * R_M_G;
      // J_p_C_fi_wrt_p_LM_B = R_C_I * R_I_M * R_M_G * R_G_LM;
      // J_p_C_fi_wrt_p_B_fi = R_C_I * R_I_M * R_M_G * R_G_LM * R_LM_B;
      J_p_C_fi_wrt_p_M_I = -R_C_I * R_I_M;
      J_p_C_fi_wrt_p_G_M = J_p_C_fi_wrt_p_M_I * R_M_G;
      J_p_C_fi_wrt_p_G_p_LM = -J_p_C_fi_wrt_p_G_M;
      J_p_C_fi_wrt_p_LM_B = J_p_C_fi_wrt_p_G_p_LM * R_G_LM;
      J_p_C_fi_wrt_p_B_fi = J_p_C_fi_wrt_p_LM_B * R_LM_B;

      // J_p_C_fi_wrt_q_B_LM =
      //     -R_C_I * R_I_M * R_M_G * R_G_LM * R_LM_B * common::skew(p_B_fi);
      // J_p_C_fi_wrt_q_G_q_LM = R_C_I * R_I_M * R_M_G * common::skew(p_G_fi);
      // J_p_C_fi_wrt_q_G_M = -R_C_I * R_I_M * R_M_G * common::skew(p_G_fi);
      // J_p_C_fi_wrt_q_I_M = R_C_I * common::skew(p_I_fi);
      J_p_C_fi_wrt_q_B_LM = -J_p_C_fi_wrt_p_B_fi * common::skew(p_B_fi);
      J_p_C_fi_wrt_q_G_q_LM = J_p_C_fi_wrt_p_G_p_LM * common::skew(p_G_fi);
      J_p_C_fi_wrt_q_G_M = J_p_C_fi_wrt_p_G_M * common::skew(p_G_fi);
      J_p_C_fi_wrt_q_I_M = R_C_I * common::skew(p_I_fi);
    } else if (error_term_type_ == visual::VisualErrorType::kLocalMission) {
      // These 4 Jacobians won't be used in the kLocalKeyframe case.
      // The following commented expressions are evaluated in an optimized way
      // below and provided here in comments for better readability.
      // J_p_C_fi_wrt_p_M_I = -R_C_I * R_I_M;
      // J_p_C_fi_wrt_p_LM_B = R_C_I * R_I_M;
      // J_p_C_fi_wrt_q_I_M = R_C_I * common::skew(p_I_fi);
      // J_p_C_fi_wrt_p_B_fi = R_C_I * R_I_M * R_LM_B;
      // J_p_C_fi_wrt_q_B_LM = -R_C_I * R_I_M * R_LM_B * common::skew(p_B_fi);
      J_p_C_fi_wrt_p_M_I = -R_C_I * R_I_M;
      J_p_C_fi_wrt_p_LM_B = -J_p_C_fi_wrt_p_M_I;
      J_p_C_fi_wrt_q_I_M = R_C_I * common::skew(p_I_fi);
      J_p_C_fi_wrt_p_B_fi = J_p_C_fi_wrt_p_LM_B * R_LM_B;
      J_p_C_fi_wrt_q_B_LM = -J_p_C_fi_wrt_p_B_fi * common::skew(p_B_fi);
    } else if (error_term_type_ == visual::VisualErrorType::kLocalKeyframe) {
      J_p_C_fi_wrt_p_B_fi = R_C_I;
    }
    // Jacobian w.r.t. landmark position expressed in landmark base frame.
    if (jacobians[kIdxLandmarkP]) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxLandmarkP]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_B_fi;
    }

    // Jacobian w.r.t. landmark base pose expressed in landmark mission frame.
    if (jacobians[kIdxLandmarkBasePose]) {
      Eigen::Map<PoseJacobian> J(jacobians[kIdxLandmarkBasePose]);
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J_quat_local_param;
      quat_parameterization.ComputeJacobian(
          q_B_LM.coeffs().data(), J_quat_local_param.data());
      J.leftCols(visual::kOrientationBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_q_B_LM * 4.0 *
          J_quat_local_param.transpose();
      J.rightCols(visual::kPositionBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_LM_B;
    }

    // Jacobian w.r.t. global landmark mission base pose.
    if (jacobians[kIdxLandmarkMissionBasePose]) {
      Eigen::Map<PoseJacobian> J(jacobians[kIdxLandmarkMissionBasePose]);
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J_quat_local_param;
      quat_parameterization.ComputeJacobian(
          q_G_LM.coeffs().data(), J_quat_local_param.data());

      J.leftCols(visual::kOrientationBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_q_G_q_LM * 4.0 *
          J_quat_local_param.transpose();
      J.rightCols(visual::kPositionBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_G_p_LM;
    }

    // Jacobian w.r.t. global keyframe mission base pose.
    if (jacobians[kIdxImuMissionBasePose]) {
      Eigen::Map<PoseJacobian> J(jacobians[kIdxImuMissionBasePose]);
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J_quat_local_param;
      quat_parameterization.ComputeJacobian(
          q_G_M.coeffs().data(), J_quat_local_param.data());

      J.leftCols(visual::kOrientationBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_q_G_M * 4.0 *
          J_quat_local_param.transpose();
      J.rightCols(visual::kPositionBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_G_M;
    }

    // Jacobian w.r.t. IMU pose expressed in keyframe mission base frame.
    if (jacobians[kIdxImuPose]) {
      Eigen::Map<PoseJacobian> J(jacobians[kIdxImuPose]);
      // We need to get the Jacobian of the local parameterization that
      // relates small changes in the tangent space (rotation vec, 3d) to
      // changes in the ambient space (quaternion, 4d); note that
      // ComputeJacobians needs row major.
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J_quat_local_param;
      quat_parameterization.ComputeJacobian(
          q_I_M.coeffs().data(), J_quat_local_param.data());

      J.leftCols(visual::kOrientationBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_q_I_M * 4.0 *
          J_quat_local_param.transpose();
      J.rightCols(visual::kPositionBlockSize) =
          J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_M_I;
    }

    // Jacobian w.r.t. camera-to-IMU orientation.
    if (jacobians[kIdxCameraToImuQ]) {
      Eigen::Map<OrientationJacobian> J(jacobians[kIdxCameraToImuQ]);
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> J_quat_local_param;
      quat_parameterization.ComputeJacobian(
          q_C_I.coeffs().data(), J_quat_local_param.data());

      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_q_C_I * 4.0 *
          J_quat_local_param.transpose();
    }

    // Jacobian w.r.t. camera-to-IMU position.
    if (jacobians[kIdxCameraToImuP]) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxCameraToImuP]);
      J = J_keypoint_wrt_p_C_fi * J_p_C_fi_wrt_p_C_I;
    }
    // Jacobian w.r.t. intrinsics.
    if (jacobians[kIdxCameraIntrinsics]) {
      Eigen::Map<IntrinsicsJacobian> J(jacobians[kIdxCameraIntrinsics]);
      J.setZero();
    }
  }
  // Compute residuals.
  Eigen::Map<Eigen::Vector3d> residual(residuals);
  residual = reprojected_landmark - measurement_;

  return true;
}

}  // namespace ceres_error_terms

#endif  // CERES_ERROR_TERMS_VISUAL_ERROR_TERM_INL_H_
