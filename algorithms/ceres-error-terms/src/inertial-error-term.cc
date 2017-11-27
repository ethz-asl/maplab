#include "ceres-error-terms/inertial-error-term.h"

#include <ceres-error-terms/parameterization/quaternion-param-jpl.h>
#include <imu-integrator/imu-integrator.h>
#include <maplab-common/quaternion-math.h>

namespace ceres_error_terms {

template <typename Derived>
void DrawSparsityPattern(
    const Eigen::MatrixBase<Derived>& matrix, const std::string& name) {
  std::cout << "-------- " << name << " --------" << std::endl;
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      if (matrix(i, j) != 0.0) {
        std::cout << " * ";
      } else {
        std::cout << "   ";
      }
    }
    std::cout << std::endl << std::endl;
  }
  std::cout << "----------------------" << std::endl;
}

void InertialErrorTerm::IntegrateStateAndCovariance(
    const InertialState& current_state,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
    InertialState* next_state, InertialStateCovariance* phi_accum,
    InertialStateCovariance* Q_accum) const {
  CHECK_NOTNULL(next_state);
  CHECK_NOTNULL(phi_accum);
  CHECK_NOTNULL(Q_accum);

  Eigen::Matrix<double, 2 * imu_integrator::kImuReadingSize, 1>
      debiased_imu_readings;
  InertialStateCovariance phi;
  InertialStateCovariance new_phi_accum;
  InertialStateCovariance Q;
  InertialStateCovariance new_Q_accum;

  Q_accum->setZero();
  phi_accum->setIdentity();

  typedef Eigen::Matrix<double, imu_integrator::kStateSize, 1>
      InertialStateVector;
  InertialStateVector current_state_vec, next_state_vec;
  current_state_vec = current_state.toVector();

  for (int i = 0; i < imu_data.cols() - 1; ++i) {
    CHECK_GE(imu_timestamps(0, i + 1), imu_timestamps(0, i))
        << "IMU measurements not properly ordered";

    const Eigen::Block<InertialStateVector, imu_integrator::kGyroBiasBlockSize,
                       1>
        current_gyro_bias =
            current_state_vec.segment<imu_integrator::kGyroBiasBlockSize>(
                imu_integrator::kStateGyroBiasOffset);
    const Eigen::Block<InertialStateVector, imu_integrator::kAccelBiasBlockSize,
                       1>
        current_accel_bias =
            current_state_vec.segment<imu_integrator::kAccelBiasBlockSize>(
                imu_integrator::kStateAccelBiasOffset);

    debiased_imu_readings << imu_data.col(i).segment<3>(
                                 imu_integrator::kAccelReadingOffset) -
                                 current_accel_bias,
        imu_data.col(i).segment<3>(imu_integrator::kGyroReadingOffset) -
            current_gyro_bias,
        imu_data.col(i + 1).segment<3>(imu_integrator::kAccelReadingOffset) -
            current_accel_bias,
        imu_data.col(i + 1).segment<3>(imu_integrator::kGyroReadingOffset) -
            current_gyro_bias;

    const double delta_time_seconds =
        (imu_timestamps(0, i + 1) - imu_timestamps(0, i)) *
        imu_integrator::kNanoSecondsToSeconds;
    integrator_.integrate(
        current_state_vec, debiased_imu_readings, delta_time_seconds,
        &next_state_vec, &phi, &Q);

    current_state_vec = next_state_vec;
    new_Q_accum = phi * (*Q_accum) * phi.transpose() + Q;

    Q_accum->swap(new_Q_accum);
    new_phi_accum = phi * (*phi_accum);
    phi_accum->swap(new_phi_accum);
  }

  *next_state = InertialState::fromVector(next_state_vec);
}

bool InertialErrorTerm::Evaluate(
    double const* const* parameters, double* residuals_ptr,
    double** jacobians) const {
  enum {
    kIdxPoseFrom,
    kIdxGyroBiasFrom,
    kIdxVelocityFrom,
    kIdxAccBiasFrom,
    kIdxPoseTo,
    kIdxGyroBiasTo,
    kIdxVelocityTo,
    kIdxAccBiasTo
  };

  // Keep Jacobians in row-major for Ceres, Eigen default is column-major.
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kGyroBiasBlockSize, Eigen::RowMajor>
      GyroBiasJacobian;
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kVelocityBlockSize, Eigen::RowMajor>
      VelocityJacobian;
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kAccelBiasBlockSize, Eigen::RowMajor>
      AccelBiasJacobian;
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kStatePoseBlockSize, Eigen::RowMajor>
      PoseJacobian;

  const double* q_from_ptr = parameters[kIdxPoseFrom];
  const double* bw_from_ptr = parameters[kIdxGyroBiasFrom];
  const double* v_from_ptr = parameters[kIdxVelocityFrom];
  const double* ba_from_ptr = parameters[kIdxAccBiasFrom];
  const double* p_from_ptr =
      parameters[kIdxPoseFrom] + imu_integrator::kStateOrientationBlockSize;

  const double* q_to_ptr = parameters[kIdxPoseTo];
  const double* bw_to_ptr = parameters[kIdxGyroBiasTo];
  const double* v_to_ptr = parameters[kIdxVelocityTo];
  const double* ba_to_ptr = parameters[kIdxAccBiasTo];
  const double* p_to_ptr =
      parameters[kIdxPoseTo] + imu_integrator::kStateOrientationBlockSize;

  Eigen::Map<const Eigen::Vector4d> q_I_M_from(q_from_ptr);
  Eigen::Map<const Eigen::Vector3d> b_g_from(bw_from_ptr);
  Eigen::Map<const Eigen::Vector3d> v_M_from(v_from_ptr);
  Eigen::Map<const Eigen::Vector3d> b_a_from(ba_from_ptr);
  Eigen::Map<const Eigen::Vector3d> p_M_I_from(p_from_ptr);

  Eigen::Map<const Eigen::Vector4d> q_I_M_to(q_to_ptr);
  Eigen::Map<const Eigen::Vector3d> b_g_to(bw_to_ptr);
  Eigen::Map<const Eigen::Vector3d> v_M_I_to(v_to_ptr);
  Eigen::Map<const Eigen::Vector3d> b_a_to(ba_to_ptr);
  Eigen::Map<const Eigen::Vector3d> p_M_I_to(p_to_ptr);

  Eigen::Map<Eigen::Matrix<double, imu_integrator::kErrorStateSize, 1> >
      residuals(residuals_ptr);

  // Integrate the IMU measurements.
  InertialState begin_state;
  begin_state.q_I_M = q_I_M_from;
  begin_state.b_g = b_g_from;
  begin_state.v_M = v_M_from;
  begin_state.b_a = b_a_from;
  begin_state.p_M_I = p_M_I_from;

  // Reuse a previous integration if the linearization point hasn't changed.
  const bool cache_is_valid = integration_cache_.valid &&
                              (integration_cache_.begin_state == begin_state);
  if (!cache_is_valid) {
    integration_cache_.begin_state = begin_state;
    IntegrateStateAndCovariance(
        integration_cache_.begin_state, imu_timestamps_, imu_data_,
        &integration_cache_.end_state, &integration_cache_.phi_accum,
        &integration_cache_.Q_accum);

    integration_cache_.L_cholesky_Q_accum.compute(integration_cache_.Q_accum);
    integration_cache_.valid = true;
  }
  CHECK(integration_cache_.valid);

  if (imu_covariance_cached_p_q_) {
    // Position.
    imu_covariance_cached_p_q_->block<3, 3>(0, 0) =
        integration_cache_.Q_accum.block<3, 3>(12, 12);

    // Rotation.
    imu_covariance_cached_p_q_->block<3, 3>(3, 3) =
        integration_cache_.Q_accum.block<3, 3>(0, 0);

    // Position-orientation cross-terms.
    imu_covariance_cached_p_q_->block<3, 3>(0, 3) =
        integration_cache_.Q_accum.block<3, 3>(12, 0);
    imu_covariance_cached_p_q_->block<3, 3>(3, 0) =
        integration_cache_.Q_accum.block<3, 3>(0, 12);
  }

  if (residuals_ptr) {
    Eigen::Quaterniond quaternion_to;
    quaternion_to.coeffs() = q_I_M_to;

    Eigen::Quaterniond quaternion_integrated;
    quaternion_integrated.coeffs() = integration_cache_.end_state.q_I_M;

    Eigen::Vector4d delta_q;
    common::positiveQuaternionProductJPL(
        q_I_M_to, quaternion_integrated.inverse().coeffs(), delta_q);
    CHECK_GE(delta_q(3), 0.);

    residuals <<
        // While our quaternion representation is Hamilton, underlying memory
        // layout is JPL because of Eigen.
        2. * delta_q.head<3>(),
        b_g_to - integration_cache_.end_state.b_g,
        v_M_I_to - integration_cache_.end_state.v_M,
        b_a_to - integration_cache_.end_state.b_a,
        p_M_I_to - integration_cache_.end_state.p_M_I;

    integration_cache_.L_cholesky_Q_accum.matrixL().solveInPlace(residuals);
  } else {
    LOG(WARNING)
        << "Skipped residual calculation, since residual pointer was NULL";
  }

  if (jacobians != NULL) {
    if (!cache_is_valid) {
      InertialJacobianType& J_end = integration_cache_.J_end;
      InertialJacobianType& J_begin = integration_cache_.J_begin;

      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> theta_local_begin;
      Eigen::Matrix<double, 4, 3, Eigen::RowMajor> theta_local_end;
      // This is the jacobian lifting the error state to the state. JPL
      // quaternion
      // parameterization is used because our memory layout of quaternions is
      // JPL.
      JplQuaternionParameterization parameterization;
      parameterization.ComputeJacobian(q_I_M_to.data(), theta_local_end.data());
      parameterization.ComputeJacobian(
          q_I_M_from.data(), theta_local_begin.data());

      // Calculate the Jacobian for the end of the edge:
      J_end.setZero();
      J_end.block<3, 4>(0, 0) = 4.0 * theta_local_end.transpose();
      J_end.block<12, 12>(3, 4) = Eigen::Matrix<double, 12, 12>::Identity();

      // Since Ceres separates the actual Jacobian from the Jacobian of the
      // local
      // parameterization, we apply the inverse of the local parameterization.
      // Ceres can then apply the local parameterization Jacobian on top of this
      // and we get the correct Jacobian in the end. This is necessary since we
      // propagate the state as error state.
      J_begin.setZero();
      J_begin.block<3, 4>(0, 0) =
          -4.0 * integration_cache_.phi_accum.block<3, 3>(0, 0) *
          theta_local_begin.transpose();
      J_begin.block<3, 12>(0, 4) =
          -integration_cache_.phi_accum.block<3, 12>(0, 3);
      J_begin.block<12, 4>(3, 0) =
          -4.0 * integration_cache_.phi_accum.block<12, 3>(3, 0) *
          theta_local_begin.transpose();
      J_begin.block<12, 12>(3, 4) =
          -integration_cache_.phi_accum.block<12, 12>(3, 3);

      // Invert and apply by using backsolve.
      integration_cache_.L_cholesky_Q_accum.matrixL().solveInPlace(J_end);
      integration_cache_.L_cholesky_Q_accum.matrixL().solveInPlace(J_begin);
    }

    const InertialJacobianType& J_end = integration_cache_.J_end;
    const InertialJacobianType& J_begin = integration_cache_.J_begin;

    if (jacobians[kIdxPoseFrom] != NULL) {
      Eigen::Map<PoseJacobian> J(jacobians[kIdxPoseFrom]);
      J.leftCols<imu_integrator::kStateOrientationBlockSize>() =
          J_begin.middleCols<imu_integrator::kStateOrientationBlockSize>(
              imu_integrator::kStateOrientationOffset);
      J.rightCols<imu_integrator::kPositionBlockSize>() =
          J_begin.middleCols<imu_integrator::kPositionBlockSize>(
              imu_integrator::kStatePositionOffset);
    }

    if (jacobians[kIdxGyroBiasFrom] != NULL) {
      Eigen::Map<GyroBiasJacobian> J(jacobians[kIdxGyroBiasFrom]);
      J = J_begin.middleCols<imu_integrator::kGyroBiasBlockSize>(
          imu_integrator::kStateGyroBiasOffset);
    }
    if (jacobians[kIdxVelocityFrom] != NULL) {
      Eigen::Map<VelocityJacobian> J(jacobians[kIdxVelocityFrom]);
      J = J_begin.middleCols<imu_integrator::kVelocityBlockSize>(
          imu_integrator::kStateVelocityOffset);
    }
    if (jacobians[kIdxAccBiasFrom] != NULL) {
      Eigen::Map<AccelBiasJacobian> J(jacobians[kIdxAccBiasFrom]);
      J = J_begin.middleCols<imu_integrator::kAccelBiasBlockSize>(
          imu_integrator::kStateAccelBiasOffset);
    }

    if (jacobians[kIdxPoseTo] != NULL) {
      Eigen::Map<PoseJacobian> J(jacobians[kIdxPoseTo]);
      J.leftCols<imu_integrator::kStateOrientationBlockSize>() =
          J_end.middleCols<imu_integrator::kStateOrientationBlockSize>(
              imu_integrator::kStateOrientationOffset);
      J.rightCols<imu_integrator::kPositionBlockSize>() =
          J_end.middleCols<imu_integrator::kPositionBlockSize>(
              imu_integrator::kStatePositionOffset);
    }
    if (jacobians[kIdxGyroBiasTo] != NULL) {
      Eigen::Map<GyroBiasJacobian> J(jacobians[kIdxGyroBiasTo]);
      J = J_end.middleCols<imu_integrator::kGyroBiasBlockSize>(
          imu_integrator::kStateGyroBiasOffset);
    }
    if (jacobians[kIdxVelocityTo] != NULL) {
      Eigen::Map<VelocityJacobian> J(jacobians[kIdxVelocityTo]);
      J = J_end.middleCols<imu_integrator::kVelocityBlockSize>(
          imu_integrator::kStateVelocityOffset);
    }
    if (jacobians[kIdxAccBiasTo] != NULL) {
      Eigen::Map<AccelBiasJacobian> J(jacobians[kIdxAccBiasTo]);
      J = J_end.middleCols<imu_integrator::kAccelBiasBlockSize>(
          imu_integrator::kStateAccelBiasOffset);
    }
  }
  return true;
}

} /* namespace ceres_error_terms */
