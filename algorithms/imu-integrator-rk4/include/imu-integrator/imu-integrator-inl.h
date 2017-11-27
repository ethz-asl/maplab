#ifndef IMU_INTEGRATOR_IMU_INTEGRATOR_INL_H_
#define IMU_INTEGRATOR_IMU_INTEGRATOR_INL_H_

#include <cmath>
#include <iomanip>
#include <limits>

#include <glog/logging.h>

#include <maplab-common/geometry.h>
#include <maplab-common/quaternion-math.h>

#include "imu-integrator/common.h"

namespace imu_integrator {

template <typename ScalarType>
void ImuIntegratorRK4::integrate(
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

  ScalarType o5 = static_cast<ScalarType>(0.5);

  next_state->setZero();
  Eigen::Matrix<ScalarType, kImuReadingSize, 1> imu_readings_k1 =
      debiased_imu_readings.template block<kImuReadingSize, 1>(0, 0);

  Eigen::Matrix<ScalarType, kImuReadingSize, 1> imu_readings_k23;
  interpolateImuReadings(
      debiased_imu_readings, delta_time_seconds, o5 * delta_time_seconds,
      &imu_readings_k23);

  Eigen::Matrix<ScalarType, kImuReadingSize, 1> imu_readings_k4 =
      debiased_imu_readings.template block<kImuReadingSize, 1>(
          kImuReadingSize, 0);

  Eigen::Matrix<ScalarType, kStateSize, 1> state_der1;
  Eigen::Matrix<ScalarType, kStateSize, 1> state_der2;
  Eigen::Matrix<ScalarType, kStateSize, 1> state_der3;
  Eigen::Matrix<ScalarType, kStateSize, 1> state_der4;
  getStateDerivativeRungeKutta(imu_readings_k1, current_state, &state_der1);

  getStateDerivativeRungeKutta(
      imu_readings_k23,
      static_cast<const Eigen::Matrix<ScalarType, kStateSize, 1> >(
          current_state + o5 * delta_time_seconds * state_der1),
      &state_der2);
  getStateDerivativeRungeKutta(
      imu_readings_k23,
      static_cast<const Eigen::Matrix<ScalarType, kStateSize, 1> >(
          current_state + o5 * delta_time_seconds * state_der2),
      &state_der3);
  getStateDerivativeRungeKutta(
      imu_readings_k4,
      static_cast<const Eigen::Matrix<ScalarType, kStateSize, 1> >(
          current_state + delta_time_seconds * state_der3),
      &state_der4);

  // Calculate final state using RK4.
  *next_state = current_state +
                delta_time_seconds * (state_der1 + ScalarType(2) * state_der2 +
                                      ScalarType(2) * state_der3 + state_der4) /
                    ScalarType(6);

  if (calculate_phi_cov) {
    next_phi->setZero();
    next_cov->setZero();

    const ScalarType* state_q_ptr = next_state->head(4).data();
    Eigen::Quaternion<ScalarType> B_q_G(state_q_ptr);
    B_q_G.normalize();

    // Now calculate state transition matrix and covariance.
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> cov_der1;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> cov_der2;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> cov_der3;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> cov_der4;

    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> transition_der1;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> transition_der2;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> transition_der3;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> transition_der4;

    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> current_cov =
        Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>::Zero();
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>
        current_transition = Eigen::Matrix<ScalarType, kErrorStateSize,
                                           kErrorStateSize>::Identity();

    getCovarianceTransitionDerivativesRungeKutta(
        imu_readings_k1, current_state, current_cov, current_transition,
        &cov_der1, &transition_der1);

    Eigen::Matrix<ScalarType, kStateSize, 1> current_state_intermediate;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>
        current_cov_intermediate;
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>
        current_transition_intermediate;

    ScalarType o5 = static_cast<ScalarType>(0.5);
    current_state_intermediate =
        current_state + o5 * delta_time_seconds * state_der1;
    current_cov_intermediate = current_cov + o5 * delta_time_seconds * cov_der1;
    current_transition_intermediate =
        current_transition + o5 * delta_time_seconds * transition_der1;
    getCovarianceTransitionDerivativesRungeKutta(
        imu_readings_k23, current_state_intermediate, current_cov_intermediate,
        current_transition_intermediate, &cov_der2, &transition_der2);

    current_state_intermediate =
        current_state + o5 * delta_time_seconds * state_der2;
    current_cov_intermediate = current_cov + o5 * delta_time_seconds * cov_der2;
    current_transition_intermediate =
        current_transition + o5 * delta_time_seconds * transition_der2;
    getCovarianceTransitionDerivativesRungeKutta(
        imu_readings_k23, current_state_intermediate, current_cov_intermediate,
        current_transition_intermediate, &cov_der3, &transition_der3);

    current_state_intermediate =
        current_state + delta_time_seconds * state_der3;
    current_cov_intermediate = current_cov + delta_time_seconds * cov_der3;
    current_transition_intermediate =
        current_transition + delta_time_seconds * transition_der3;
    getCovarianceTransitionDerivativesRungeKutta(
        imu_readings_k4, current_state_intermediate, current_cov_intermediate,
        current_transition_intermediate, &cov_der4, &transition_der4);

    *next_cov = current_cov +
                delta_time_seconds *
                    (cov_der1 + static_cast<ScalarType>(2) * cov_der2 +
                     static_cast<ScalarType>(2) * cov_der3 + cov_der4) /
                    static_cast<ScalarType>(6);
    *next_phi = current_transition;

    next_phi->template block<3, 15>(0, 0) +=
        delta_time_seconds *
        (transition_der1.template block<3, 15>(0, 0) +
         ScalarType(2) * transition_der2.template block<3, 15>(0, 0) +
         ScalarType(2) * transition_der3.template block<3, 15>(0, 0) +
         transition_der4.template block<3, 15>(0, 0)) /
        ScalarType(6.0);
    next_phi->template block<3, 15>(6, 0) +=
        delta_time_seconds *
        (transition_der1.template block<3, 15>(6, 0) +
         ScalarType(2) * transition_der2.template block<3, 15>(6, 0) +
         ScalarType(2) * transition_der3.template block<3, 15>(6, 0) +
         transition_der4.template block<3, 15>(6, 0)) /
        ScalarType(6.0);
    next_phi->template block<3, 15>(12, 0) +=
        delta_time_seconds *
        (transition_der1.template block<3, 15>(12, 0) +
         ScalarType(2) * transition_der2.template block<3, 15>(12, 0) +
         ScalarType(2) * transition_der3.template block<3, 15>(12, 0) +
         transition_der4.template block<3, 15>(12, 0)) /
        ScalarType(6.0);
  }
}

template <typename ScalarType>
void ImuIntegratorRK4::getStateDerivativeRungeKutta(
    const Eigen::Matrix<ScalarType, kImuReadingSize, 1>& debiased_imu_readings,
    const Eigen::Matrix<ScalarType, kStateSize, 1>& current_state,
    Eigen::Matrix<ScalarType, kStateSize, 1>* state_derivative) const {
  CHECK_NOTNULL(state_derivative);

  Eigen::Quaternion<ScalarType> B_q_G(current_state.head(4).data());
  // As B_q_G is calculated using linearization, it may not be normalized
  // -> we need to do it explicitly before passing to quaternion object.
  ScalarType o5 = static_cast<ScalarType>(0.5);
  B_q_G.normalize();
  Eigen::Matrix<ScalarType, 3, 3> B_R_G;
  common::toRotationMatrixJPL(B_q_G.coeffs(), &B_R_G);
  const Eigen::Matrix<ScalarType, 3, 3> G_R_B = B_R_G.transpose();

  const Eigen::Matrix<ScalarType, 3, 1> acc_meas(
      debiased_imu_readings.template block<3, 1>(kAccelReadingOffset, 0)
          .data());
  const Eigen::Matrix<ScalarType, 3, 1> gyr_meas(
      debiased_imu_readings.template block<3, 1>(kGyroReadingOffset, 0).data());

  Eigen::Matrix<ScalarType, 4, 4> gyro_omega;
  gyroOmegaJPL(gyr_meas, &gyro_omega);

  Eigen::Matrix<ScalarType, 4, 1> q_dot = o5 * gyro_omega * B_q_G.coeffs();
  Eigen::Matrix<ScalarType, 3, 1> v_dot =
      G_R_B * acc_meas -
      Eigen::Matrix<ScalarType, 3, 1>(
          ScalarType(0), ScalarType(0), ScalarType(gravity_acceleration_));
  Eigen::Matrix<ScalarType, 3, 1> p_dot =
      current_state.template block<3, 1>(kStateVelocityOffset, 0);

  state_derivative->setZero();  // Bias derivatives are zero.
  state_derivative->template block<4, 1>(kStateOrientationOffset, 0) = q_dot;
  state_derivative->template block<3, 1>(kStateVelocityOffset, 0) = v_dot;
  state_derivative->template block<3, 1>(kStatePositionOffset, 0) = p_dot;
}

template <typename ScalarType>
void ImuIntegratorRK4::getCovarianceTransitionDerivativesRungeKutta(
    const Eigen::Matrix<ScalarType, kImuReadingSize, 1>& debiased_imu_readings,
    const Eigen::Matrix<ScalarType, kStateSize, 1>& current_state,
    const Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>&
        current_cov,
    const Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>&
        current_transition,
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>* cov_derivative,
    Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>*
        transition_derivative) const {
  CHECK_NOTNULL(cov_derivative);
  CHECK_NOTNULL(transition_derivative);

  Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize> phi_cont =
      Eigen::Matrix<ScalarType, kErrorStateSize, kErrorStateSize>::Zero();

  Eigen::Quaternion<ScalarType> B_q_G(current_state.head(4).data());
  // As B_q_G is calculated using linearization, it may not be normalized
  // -> we need to do it explicitly before passing to quaternion object.
  B_q_G.normalize();
  Eigen::Matrix<ScalarType, 3, 3> B_R_G;
  common::toRotationMatrixJPL(B_q_G.coeffs(), &B_R_G);
  const Eigen::Matrix<ScalarType, 3, 3> G_R_B = B_R_G.transpose();

  Eigen::Matrix<ScalarType, 3, 1> acc_meas(
      debiased_imu_readings.template block<3, 1>(kAccelReadingOffset, 0)
          .data());
  const Eigen::Matrix<ScalarType, 3, 1> gyr_meas(
      debiased_imu_readings.template block<3, 1>(kGyroReadingOffset, 0).data());

  const Eigen::Matrix<ScalarType, 3, 3> gyro_skew;
  common::skew(gyr_meas, gyro_skew);

  const Eigen::Matrix<ScalarType, 3, 3> acc_skew;
  common::skew(acc_meas, acc_skew);

  phi_cont.template block<3, 3>(0, 3) =
      -Eigen::Matrix<ScalarType, 3, 3>::Identity();
  phi_cont.template block<3, 3>(12, 6) =
      Eigen::Matrix<ScalarType, 3, 3>::Identity();
  phi_cont.template block<3, 3>(0, 0) = -gyro_skew;
  phi_cont.template block<3, 3>(6, 9) = -G_R_B;
  phi_cont.template block<3, 3>(6, 0) = -G_R_B * acc_skew;

  // Compute *transition_derivative = phi_cont * current_transition blockwise.
  transition_derivative->setZero();
  transition_derivative->template block<3, 15>(0, 0) =
      phi_cont.template block<3, 3>(0, 0) *
          current_transition.template block<3, 15>(0, 0) -
      current_transition.template block<3, 15>(3, 0);
  transition_derivative->template block<3, 15>(6, 0) =
      phi_cont.template block<3, 3>(6, 0) *
          current_transition.template block<3, 15>(0, 0) +
      phi_cont.template block<3, 3>(6, 9) *
          current_transition.template block<3, 15>(9, 0);
  transition_derivative->template block<3, 15>(12, 0) =
      current_transition.template block<3, 15>(6, 0);

  Eigen::Matrix<ScalarType, 15, 15> phi_cont_cov =
      Eigen::Matrix<ScalarType, 15, 15>::Zero();
  phi_cont_cov.template block<3, 15>(0, 0) =
      phi_cont.template block<3, 3>(0, 0) *
          current_cov.template block<3, 15>(0, 0) -
      current_cov.template block<3, 15>(3, 0);
  phi_cont_cov.template block<3, 15>(6, 0) =
      phi_cont.template block<3, 3>(6, 0) *
          current_cov.template block<3, 15>(0, 0) +
      phi_cont.template block<3, 3>(6, 9) *
          current_cov.template block<3, 15>(9, 0);
  phi_cont_cov.template block<3, 15>(12, 0) =
      current_cov.template block<3, 15>(6, 0);
  *cov_derivative = phi_cont_cov + phi_cont_cov.transpose();

  // Relevant parts of Gc * Qc * Gc'.
  cov_derivative->diagonal().template segment<3>(0) +=
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(gyro_noise_sigma_squared_));
  cov_derivative->diagonal().template segment<3>(3) +=
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(gyro_bias_sigma_squared_));
  cov_derivative->diagonal().template segment<3>(6) +=
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(acc_noise_sigma_squared_));
  cov_derivative->diagonal().template segment<3>(9) +=
      Eigen::Matrix<ScalarType, 3, 1>::Constant(
          static_cast<ScalarType>(acc_bias_sigma_squared_));
}

template <typename ScalarType>
void ImuIntegratorRK4::interpolateImuReadings(
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

template <typename ScalarType>
void ImuIntegratorRK4::gyroOmegaJPL(
    const Eigen::Matrix<ScalarType, 3, 1>& gyro_readings,
    Eigen::Matrix<ScalarType, 4, 4>* omega_matrix) const {
  CHECK_NOTNULL(omega_matrix);

  const ScalarType scalar_type_zero = static_cast<ScalarType>(0.);

  *omega_matrix << scalar_type_zero, gyro_readings[2], -gyro_readings[1],
      gyro_readings[0], -gyro_readings[2], scalar_type_zero, gyro_readings[0],
      gyro_readings[1], gyro_readings[1], -gyro_readings[0], scalar_type_zero,
      gyro_readings[2], -gyro_readings[0], -gyro_readings[1], -gyro_readings[2],
      scalar_type_zero;
}

}  // namespace imu_integrator

#endif  // IMU_INTEGRATOR_IMU_INTEGRATOR_INL_H_
