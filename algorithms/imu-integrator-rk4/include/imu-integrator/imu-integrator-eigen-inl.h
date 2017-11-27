#ifndef IMU_INTEGRATOR_IMU_INTEGRATOR_EIGEN_INL_H_
#define IMU_INTEGRATOR_IMU_INTEGRATOR_EIGEN_INL_H_

#include <cmath>
#include <iomanip>
#include <limits>

#include <glog/logging.h>
#include <maplab-common/geometry.h>
#include <maplab-common/quaternion-math.h>

#include "imu-integrator/common.h"

namespace imu_integrator {

template <typename ScalarType>
void ImuIntegratorEigen::integrate(
    const Eigen::Matrix<ScalarType, kStateSize, 1>& current_state,
    const Eigen::Matrix<ScalarType, 2 * kImuReadingSize, 1>&
        debiased_imu_readings,
    const ScalarType delta_time_seconds,
    Eigen::Matrix<ScalarType, kStateSize, 1>* next_state,
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>* next_phi,
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>* next_cov)
    const {
  // The (next_phi, next_cov) pair is optional; both pointers have to be null to
  // skip calculations.
  LOG_IF(FATAL, static_cast<bool>(next_phi) != static_cast<bool>(next_cov))
      << "next_phi and next_cov have to be either both valid or be null";
  bool calculate_phi_cov = (next_phi != nullptr) && (next_cov != nullptr);

  const ScalarType o5 = static_cast<ScalarType>(0.5);

  next_state->setZero();

  Eigen::Matrix<ScalarType, kImuReadingSize, 1> imu_reading;
  interpolateImuReadings(
      debiased_imu_readings, delta_time_seconds, o5 * delta_time_seconds,
      &imu_reading);

  // Extract state variables for current state.
  const Eigen::Ref<const Vector3<ScalarType> > M_p_MI(
      current_state.template block<3, 1>(kStatePositionOffset, 0));
  const Eigen::Map<const Eigen::Quaternion<ScalarType> > q_MI(
      &current_state(kStateOrientationOffset));
  const Eigen::Ref<const Vector3<ScalarType> > I_v_I(
      current_state.template block<3, 1>(kStateVelocityOffset, 0));

  // Extract IMU measurement.
  const Eigen::Ref<const Vector3<ScalarType> > I_acceleration(
      imu_reading.template block<3, 1>(kAccelReadingOffset, 0));
  const Eigen::Ref<const Vector3<ScalarType> > I_angular_rate(
      imu_reading.template block<3, 1>(kGyroReadingOffset, 0));

  // Extract state variables for next state.
  Eigen::Ref<Vector3<ScalarType> > M_p_MI_kp1(
      next_state->template block<3, 1>(kStatePositionOffset, 0));

  Eigen::Map<Eigen::Quaternion<ScalarType> > q_MI_kp1(
      &(next_state->coeffRef(kStateOrientationOffset, 0)));
  Eigen::Ref<Vector3<ScalarType> > I_v_I_kp1(
      next_state->template block<3, 1>(kStateVelocityOffset, 0));

  const Eigen::Matrix<ScalarType, 3, 3> R_IM =
      q_MI.toRotationMatrix().transpose();
  const Eigen::Vector3d I_acceleration_with_gravity =
      -gravity_acceleration_ * R_IM.template block<3, 1>(0, 2) + I_acceleration;

  // Do the integration using a first order approximation.
  // TODO(burrimi): Should we add acceleration + 0.5*a*dt^2?
  M_p_MI_kp1 =
      M_p_MI +
      delta_time_seconds * q_MI.toRotationMatrix() *
          (I_v_I + 0.5 * delta_time_seconds * I_acceleration_with_gravity);
  q_MI_kp1 = q_MI * common::eigen_quaternion_helpers::ExpMap<ScalarType>(
                        delta_time_seconds * I_angular_rate);

  I_v_I_kp1 = I_v_I +
              delta_time_seconds * (I_acceleration_with_gravity -
                                    common::skew(I_angular_rate) * I_v_I);

  // Gyro and accelerometer bias are assumed constant.
  next_state->template block<3, 1>(kStateGyroBiasOffset, 0) =
      current_state.template block<3, 1>(kStateGyroBiasOffset, 0);
  next_state->template block<3, 1>(kStateAccelBiasOffset, 0) =
      current_state.template block<3, 1>(kStateAccelBiasOffset, 0);

  if (calculate_phi_cov) {
    // Now calculate state transition matrix and covariance.
    getJacobianAndCovariance(
        current_state, imu_reading, delta_time_seconds, next_phi, next_cov);
  }
}

template <typename ScalarType>
void ImuIntegratorEigen::getJacobianAndCovariance(
    const Eigen::Matrix<ScalarType, kStateSize, 1>& current_state,
    const Eigen::Matrix<ScalarType, kImuReadingSize, 1>&
        debiased_imu_reading,
    const ScalarType dt, StateJacobian<ScalarType>* jacobian,
    CovarianceMatrix<ScalarType>* covariance) const {
  CHECK_NOTNULL(jacobian);
  CHECK_NOTNULL(covariance);

  jacobian->setIdentity();

  // Extract state variables for current state.
  const Eigen::Ref<const Vector3<ScalarType> > M_p_MI(
      current_state.template block<3, 1>(kStatePositionOffset, 0));
  const Eigen::Map<const Eigen::Quaternion<ScalarType> > q_MI(
      &current_state(kStateOrientationOffset));
  const Eigen::Ref<const Vector3<ScalarType> > I_v_I(
      current_state.template block<3, 1>(kStateVelocityOffset, 0));

  // Extract relevant IMU measurement.
  const Eigen::Ref<const Vector3<ScalarType> > I_angular_rate(
      debiased_imu_reading.template block<3, 1>(kGyroReadingOffset, 0));

  const Eigen::Matrix<ScalarType, 3, 3> R_MI = q_MI.toRotationMatrix();
  const Eigen::Matrix<ScalarType, 3, 3> R_IM = R_MI.transpose();

  // Position Jacobians.
  jacobian->template block<3, 3>(
      kErrorStatePositionOffset, kErrorStateOrientationOffset) =
      -dt * common::skew(R_MI * I_v_I);
  jacobian->template block<3, 3>(
      kErrorStatePositionOffset, kErrorStateVelocityOffset) = dt * R_MI;

  // Orientation Jacobians.
  jacobian->template block<3, 3>(
      kErrorStateOrientationOffset, kErrorStateGyroBiasOffset) =
      -dt * R_MI *
      common::eigen_quaternion_helpers::Gamma<ScalarType>(dt * I_angular_rate);

  const Vector3<ScalarType> M_gravity(
      ScalarType(0), ScalarType(0), ScalarType(-gravity_acceleration_));
  // Velocity Jacobians.
  jacobian->template block<3, 3>(
      kErrorStateVelocityOffset, kErrorStateVelocityOffset) +=
      -dt * common::skew(I_angular_rate);
  jacobian->template block<3, 3>(
      kErrorStateVelocityOffset, kErrorStateOrientationOffset) =
      dt * R_IM * common::skew(M_gravity);
  jacobian->template block<3, 3>(
      kErrorStateVelocityOffset, kErrorStateGyroBiasOffset) =
      -dt * common::skew(I_v_I);
  jacobian->template block<3, 3>(
      kErrorStateVelocityOffset, kErrorStateAccelBiasOffset) =
      -dt * Matrix3<ScalarType>::Identity();

  // Relevant parts of G * Qd * G'.
  covariance->setZero();

  // Temporaries.
  const double dt_squared = dt * dt;
  const Matrix3<ScalarType> skew_v = common::skew(I_v_I);
  const Matrix3<ScalarType> R_MI_gamma_w =
      R_MI * common::eigen_quaternion_helpers::Gamma<ScalarType>(
          dt * I_angular_rate);

  Eigen::Matrix<ScalarType, 12, 12> Qd;
  Qd.setZero();
  Qd.diagonal().template segment<3>(0) =
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(dt_squared * gyro_noise_sigma_squared_));
  Qd.diagonal().template segment<3>(3) =
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(dt_squared * gyro_bias_sigma_squared_));
  Qd.diagonal().template segment<3>(6) =
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(dt_squared * acc_noise_sigma_squared_));
  Qd.diagonal().template segment<3>(9) =
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(dt_squared * acc_bias_sigma_squared_));

  Eigen::Matrix<ScalarType, 15, 12> G;
  G.setIdentity();

  G.template block<3, 3>(kErrorStateOrientationOffset, 0) = -R_MI_gamma_w;
  G.template block<3, 3>(kErrorStateVelocityOffset, 0) = -skew_v;

  *covariance = G * Qd * G.transpose();

  // TODO(burrimi): Optimize this code and calculate closed form for cov.
  //  // Diagonal elements.
  //  const Matrix3<ScalarType> cov_q_wrt_n_g =
  //      dt_squared * gyro_noise_sigma_squared_ * R_MI_gamma_w *
  //      R_MI_gamma_w.transpose();
  //
  //  const Eigen::DiagonalMatrix<ScalarType, 3> cov_a(
  //      acc_noise_sigma_squared_, acc_noise_sigma_squared_,
  //      acc_noise_sigma_squared_);
  //  const Matrix3<ScalarType> cov_v_wrt_n_a =
  //      dt_squared * (cov_a.toDenseMatrix() +
  //                    gyro_noise_sigma_squared_ * skew_v *
  //                    skew_v.transpose());
  //
  //  // Off-diagonal elements.
  //  const Matrix3<ScalarType> cov_q_wrt_n_a = dt_squared *
  //                                            gyro_noise_sigma_squared_ *
  //                                            R_MI_gamma_w *
  //                                            skew_v.transpose();
  //  const Matrix3<ScalarType> cov_v_wrt_n_g = dt_squared *
  //                                            gyro_noise_sigma_squared_ *
  //                                            skew_v *
  //                                            R_MI_gamma_w.transpose();
  //
  //  covariance->template block<3, 3>(kErrorStateOrientationOffset, 0) =
  //      cov_q_wrt_n_g;
  //  covariance->template block<3, 3>(kErrorStateOrientationOffset, 3) =
  //      cov_q_wrt_n_a;
  //  covariance->template block<3, 3>(kErrorStateVelocityOffset, 3) =
  //      cov_v_wrt_n_a;
  //  covariance->template block<3, 3>(kErrorStateVelocityOffset, 0) =
  //      cov_v_wrt_n_g;
  //
  //  covariance->diagonal().template segment<3>(kErrorStateGyroBiasOffset) =
  //      Eigen::Matrix<ScalarType, 3, 1>::Constant(
  //          static_cast<ScalarType>(dt_squared * gyro_bias_sigma_squared_));
  //  covariance->diagonal().template segment<3>(kErrorStateAccelBiasOffset) =
  //      Eigen::Matrix<ScalarType, 3, 1>::Constant(
  //          static_cast<ScalarType>(dt_squared * acc_bias_sigma_squared_));
}

template <typename ScalarType>
void ImuIntegratorEigen::interpolateImuReadings(
    const Eigen::Matrix<ScalarType, 2 * kImuReadingSize, 1>& imu_readings,
    const ScalarType delta_time_seconds,
    const ScalarType increment_step_size_seconds,
    Eigen::Matrix<ScalarType, kImuReadingSize, 1>* interpolated_imu_readings)
    const {
  CHECK_NOTNULL(interpolated_imu_readings);
  CHECK_GE(delta_time_seconds, 0.0);

  if (delta_time_seconds < std::numeric_limits<ScalarType>::epsilon()) {
    *interpolated_imu_readings =
        imu_readings.template block<kImuReadingSize, 1>(0, 0);
    return;
  }

  *interpolated_imu_readings =
      imu_readings.template block<kImuReadingSize, 1>(0, 0) +
      (imu_readings.template block<kImuReadingSize, 1>(kImuReadingSize, 0) -
       imu_readings.template block<kImuReadingSize, 1>(0, 0)) *
          (increment_step_size_seconds / delta_time_seconds);
}

}  // namespace imu_integrator

#endif  // IMU_INTEGRATOR_IMU_INTEGRATOR_EIGEN_INL_H_
