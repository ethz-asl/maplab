#include "ceres-error-terms/inertial-error-term-eigen.h"

#include <iostream>

#include <imu-integrator/imu-integrator-eigen.h>
#include <maplab-common/quaternion-math.h>

#include "ceres-error-terms/parameterization/quaternion-param-eigen.h"

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

void InertialErrorTermEigen::IntegrateStateAndCovariance(
    const InertialStateEigen& current_state,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps_ns,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data,
    InertialStateEigen* next_state, InertialStateCovariance* phi_accum,
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
    CHECK_GE(imu_timestamps_ns(0, i + 1), imu_timestamps_ns(0, i))
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
        (imu_timestamps_ns(0, i + 1) - imu_timestamps_ns(0, i)) *
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

  *next_state = InertialStateEigen::fromVector(next_state_vec);
}

bool InertialErrorTermEigen::Evaluate(
    double const* const* parameters, double* residuals_ptr,
    double** jacobians) const {
  // Keep Jacobians in row-major for Ceres, Eigen default is column-major.
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kVelocityBlockSize, Eigen::RowMajor>
      VelocityJacobian;
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kImuBiasBlockSize, Eigen::RowMajor>
      ImuBiasJacobian;
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kStateOrientationBlockSize,
                        Eigen::RowMajor>
      OrientationJacobian;
  typedef Eigen::Matrix<double, imu_integrator::kErrorStateSize,
                        imu_integrator::kPositionBlockSize, Eigen::RowMajor>
      PositionJacobian;

  const double* q_from_ptr = parameters[kIdxOrientationFrom];
  const double* v_from_ptr = parameters[kIdxVelocityFrom];
  const double* bw_from_ptr = parameters[kIdxImuBiasFrom];
  const double* ba_from_ptr = parameters[kIdxImuBiasFrom] + 3;
  const double* p_from_ptr = parameters[kIdxPositionFrom];

  const double* q_to_ptr = parameters[kIdxOrientationTo];
  const double* v_to_ptr = parameters[kIdxVelocityTo];
  const double* bw_to_ptr = parameters[kIdxImuBiasTo];
  const double* ba_to_ptr = parameters[kIdxImuBiasTo] + 3;
  const double* p_to_ptr = parameters[kIdxPositionTo];

  Eigen::Map<const Eigen::Vector4d> q_M_I_from(q_from_ptr);
  Eigen::Map<const Eigen::Vector3d> b_g_from(bw_from_ptr);
  Eigen::Map<const Eigen::Vector3d> I_v_I_from(v_from_ptr);
  Eigen::Map<const Eigen::Vector3d> b_a_from(ba_from_ptr);
  Eigen::Map<const Eigen::Vector3d> M_p_MI_from(p_from_ptr);

  Eigen::Map<const Eigen::Vector4d> q_M_I_to(q_to_ptr);
  Eigen::Map<const Eigen::Vector3d> b_g_to(bw_to_ptr);
  Eigen::Map<const Eigen::Vector3d> I_v_I_to(v_to_ptr);
  Eigen::Map<const Eigen::Vector3d> b_a_to(ba_to_ptr);
  Eigen::Map<const Eigen::Vector3d> M_p_MI_to(p_to_ptr);

  Eigen::Map<Eigen::Matrix<double, imu_integrator::kErrorStateSize, 1> >
      residuals(residuals_ptr);

  if (VLOG_IS_ON(5) && I_v_I_from.squaredNorm() == 0 &&
      I_v_I_to.squaredNorm() == 0) {
    VLOG(5) << "The velocity at both keyframes is zero.";
  }

  // Integrate the IMU measurements.
  InertialStateEigen begin_state;
  begin_state.q_M_I = q_M_I_from;
  begin_state.b_g = b_g_from;
  begin_state.I_v_I = I_v_I_from;
  begin_state.b_a = b_a_from;
  begin_state.M_p_MI = M_p_MI_from;

  // Reuse a previous integration if the linearization point hasn't changed.
  const bool cache_is_valid = integration_cache_.valid &&
                              (integration_cache_.begin_state == begin_state);
  if (!cache_is_valid) {
    integration_cache_.begin_state = begin_state;
    IntegrateStateAndCovariance(
        integration_cache_.begin_state, imu_timestamps_ns_, imu_data_,
        &integration_cache_.end_state, &integration_cache_.phi_accum,
        &integration_cache_.Q_accum);

    integration_cache_.L_cholesky_Q_accum.compute(integration_cache_.Q_accum);
    integration_cache_.valid = true;
  }

  CHECK(integration_cache_.valid);

  if (imu_covariance_cached_) {
    // TODO(slynen): double check frame of reference.
    imu_covariance_cached_->block<3, 3>(0, 0) =
        integration_cache_.Q_accum.block<3, 3>(0, 0);
    imu_covariance_cached_->block<3, 3>(0, 3) =
        integration_cache_.Q_accum.block<3, 3>(0, 12);
    imu_covariance_cached_->block<3, 3>(3, 3) =
        integration_cache_.Q_accum.block<3, 3>(12, 12);
    imu_covariance_cached_->block<3, 3>(3, 0) =
        integration_cache_.Q_accum.block<3, 3>(12, 0);
  }

  if (residuals_ptr) {
    Eigen::Quaterniond quaternion_to;
    quaternion_to.coeffs() = q_M_I_to;

    const Eigen::Quaterniond quaternion_integrated(
        integration_cache_.end_state.q_M_I);

    Eigen::Vector3d orientation_error;
    common::eigen_quaternion_helpers::Minus(
        quaternion_to, quaternion_integrated, &orientation_error);

    // Note: The residual must have the same order as the Jacobians!!!
    residuals << orientation_error, b_g_to - integration_cache_.end_state.b_g,
        I_v_I_to - integration_cache_.end_state.I_v_I,
        b_a_to - integration_cache_.end_state.b_a,
        M_p_MI_to - integration_cache_.end_state.M_p_MI;

    integration_cache_.L_cholesky_Q_accum.matrixL().solveInPlace(residuals);

  } else {
    LOG(WARNING)
        << "Skipped residual calculation, since residual pointer was NULL";
  }

  if (jacobians != NULL) {
    if (!cache_is_valid) {
      InertialJacobianType& J_end = integration_cache_.J_end;
      InertialJacobianType& J_begin = integration_cache_.J_begin;

      Eigen::Quaterniond quaternion_to;
      quaternion_to.coeffs() = q_M_I_to;
      const Eigen::Quaterniond quaternion_integrated(
          integration_cache_.end_state.q_M_I);

      // These are the Jacobians lifting the orientation in the tangent space to
      // the quaternions in the state.
      EigenQuaternionParameterization::LiftJacobian lift_jacobian_from;
      EigenQuaternionParameterization::LiftJacobian lift_jacobian_to;

      // Jacobians required to properly account for the manifold structure of
      // SO3. Reminder: q_residual = q_to boxminus q_integrated
      // TODO(burrimi): Check if this is actually improving the convergence.
      // Otherwise just set to identity as done in almost every BA.
      Eigen::Matrix3d J_boxminus_wrt_q_to;
      Eigen::Matrix3d J_boxminus_wrt_q_integrated;

      common::eigen_quaternion_helpers::GetBoxminusJacobians(
          quaternion_to, quaternion_integrated, &J_boxminus_wrt_q_to,
          &J_boxminus_wrt_q_integrated);

      EigenQuaternionParameterization parameterization;
      parameterization.ComputeLiftJacobian(
          q_M_I_from.data(), lift_jacobian_from.data());
      parameterization.ComputeLiftJacobian(
          q_M_I_to.data(), lift_jacobian_to.data());

      // Calculate the Jacobian for the end of the edge:
      J_end.setZero();
      J_end.block<3, 4>(0, 0) = J_boxminus_wrt_q_to * lift_jacobian_to;
      J_end.block<12, 12>(3, 4) = Eigen::Matrix<double, 12, 12>::Identity();

      // Since Ceres separates the actual Jacobian from the Jacobian of the
      // local
      // parameterization, we apply the inverse of the local parameterization.
      // Ceres can then apply the local parameterization Jacobian on top of this
      // and we get the correct Jacobian in the end. This is necessary since we
      // propagate the state in the tangent space of the manifold.
      J_begin.setZero();
      J_begin.block<3, 4>(0, 0) =
          J_boxminus_wrt_q_integrated *
          integration_cache_.phi_accum.block<3, 3>(0, 0) * lift_jacobian_from;
      J_begin.block<3, 12>(0, 4) =
          -integration_cache_.phi_accum.block<3, 12>(0, 3);
      J_begin.block<12, 4>(3, 0) =
          -integration_cache_.phi_accum.block<12, 3>(3, 0) * lift_jacobian_from;
      J_begin.block<12, 12>(3, 4) =
          -integration_cache_.phi_accum.block<12, 12>(3, 3);

      // Invert and apply by using backsolve.
      integration_cache_.L_cholesky_Q_accum.matrixL().solveInPlace(J_end);
      integration_cache_.L_cholesky_Q_accum.matrixL().solveInPlace(J_begin);
    }

    const InertialJacobianType& J_end = integration_cache_.J_end;
    const InertialJacobianType& J_begin = integration_cache_.J_begin;

    if (jacobians[kIdxOrientationFrom] != NULL) {
      Eigen::Map<OrientationJacobian> J(jacobians[kIdxOrientationFrom]);
      J = J_begin.middleCols<imu_integrator::kStateOrientationBlockSize>(
          imu_integrator::kStateOrientationOffset);
    }

    if (jacobians[kIdxPositionFrom] != NULL) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxPositionFrom]);
      J = J_begin.middleCols<imu_integrator::kPositionBlockSize>(
          imu_integrator::kStatePositionOffset);
    }

    if (jacobians[kIdxVelocityFrom] != NULL) {
      Eigen::Map<VelocityJacobian> J(jacobians[kIdxVelocityFrom]);
      J = J_begin.middleCols<imu_integrator::kVelocityBlockSize>(
          imu_integrator::kStateVelocityOffset);
    }
    if (jacobians[kIdxImuBiasFrom] != NULL) {
      Eigen::Map<ImuBiasJacobian> J(jacobians[kIdxImuBiasFrom]);
      J.leftCols<imu_integrator::kGyroBiasBlockSize>() =
          J_begin.middleCols<imu_integrator::kGyroBiasBlockSize>(
              imu_integrator::kStateGyroBiasOffset);
      J.rightCols<imu_integrator::kAccelBiasBlockSize>() =
          J_begin.middleCols<imu_integrator::kAccelBiasBlockSize>(
              imu_integrator::kStateAccelBiasOffset);
    }

    if (jacobians[kIdxOrientationTo] != NULL) {
      Eigen::Map<OrientationJacobian> J(jacobians[kIdxOrientationTo]);
      J = J_end.middleCols<imu_integrator::kStateOrientationBlockSize>(
          imu_integrator::kStateOrientationOffset);
    }

    if (jacobians[kIdxPositionTo] != NULL) {
      Eigen::Map<PositionJacobian> J(jacobians[kIdxPositionTo]);
      J = J_end.middleCols<imu_integrator::kPositionBlockSize>(
          imu_integrator::kStatePositionOffset);
    }

    if (jacobians[kIdxVelocityTo] != NULL) {
      Eigen::Map<VelocityJacobian> J(jacobians[kIdxVelocityTo]);
      J = J_end.middleCols<imu_integrator::kVelocityBlockSize>(
          imu_integrator::kStateVelocityOffset);
    }
    if (jacobians[kIdxImuBiasTo] != NULL) {
      Eigen::Map<ImuBiasJacobian> J(jacobians[kIdxImuBiasTo]);
      J.leftCols<imu_integrator::kGyroBiasBlockSize>() =
          J_end.middleCols<imu_integrator::kGyroBiasBlockSize>(
              imu_integrator::kStateGyroBiasOffset);
      J.rightCols<imu_integrator::kAccelBiasBlockSize>() =
          J_end.middleCols<imu_integrator::kAccelBiasBlockSize>(
              imu_integrator::kStateAccelBiasOffset);
    }
  }
  return true;
}

} /* namespace ceres_error_terms */
