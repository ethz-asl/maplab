#include "vio-common/imu-forward-propagation.h"

#include <imu-integrator/common.h>
#include <imu-integrator/imu-integrator.h>
#include <maplab-common/conversions.h>

namespace vio {

bool propagatePoseUsingImu(
    const vio::ViNodeState& vi_state_k, const double gravity_acceleration,
    const int64_t timestamp_kp1,
    const vio_common::ImuMeasurementBuffer& imu_buffer,
    vio::ViNodeState* vi_state_kp1) {
  CHECK_NOTNULL(vi_state_kp1);
  CHECK_GT(timestamp_kp1, vi_state_k.getTimestamp());
  CHECK_GE(gravity_acceleration, 0.);

  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements;
  vio_common::ImuMeasurementBuffer::QueryResult result =
      imu_buffer.getImuDataInterpolatedBorders(
          vi_state_k.getTimestamp(), timestamp_kp1, &imu_timestamps,
          &imu_measurements);

  if (result != vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable) {
    // No data in the desired range is available.
    return false;
  }

  if (imu_timestamps.cols() < 2) {
    // At least 2 IMU measurements are necessary to interpolate the pose.
    return false;
  }

  // These values does not matter as we only propagate the state and not
  // the covariance matrix.
  constexpr double kGyroNoiseSigma = 1.;
  constexpr double kGyroBiasSigma = 1.;
  constexpr double kAccNoiseSigma = 1.;
  constexpr double kAccBiasSigma = 1.;
  imu_integrator::ImuIntegratorRK4 integrator(
      kGyroNoiseSigma, kGyroBiasSigma, kAccNoiseSigma, kAccBiasSigma,
      gravity_acceleration);

  Eigen::Matrix<double, imu_integrator::kStateSize, 1> current_state;
  current_state.block<4, 1>(imu_integrator::kStateOrientationOffset, 0) =
      vi_state_k.get_T_M_I().getRotation().toImplementation().coeffs();
  current_state.block<3, 1>(imu_integrator::kStateGyroBiasOffset, 0) =
      vi_state_k.getGyroBias();
  current_state.block<3, 1>(imu_integrator::kStateVelocityOffset, 0) =
      vi_state_k.get_v_M_I();
  current_state.block<3, 1>(imu_integrator::kStateAccelBiasOffset, 0) =
      vi_state_k.getAccBias();
  current_state.block<3, 1>(imu_integrator::kStatePositionOffset, 0) =
      vi_state_k.get_T_M_I().getPosition();

  Eigen::Matrix<double, 6, 1> imu_biases;
  imu_biases.block<3, 1>(imu_integrator::kAccelReadingOffset, 0) =
      vi_state_k.getAccBias();
  imu_biases.block<3, 1>(imu_integrator::kGyroReadingOffset, 0) =
      vi_state_k.getGyroBias();

  // Debias the IMU measurements.
  imu_measurements = imu_measurements.colwise() - imu_biases;

  for (int i = 0; i < imu_timestamps.cols() - 1; ++i) {
    const double delta_time_seconds =
        (imu_timestamps(i + 1) - imu_timestamps(i)) * kNanosecondsToSeconds;
    CHECK_GT(delta_time_seconds, 0.0);

    Eigen::Matrix<double, 12, 1> debiased_imu_measurements;
    debiased_imu_measurements << imu_measurements.col(i),
        imu_measurements.col(i + 1);

    Eigen::Matrix<double, imu_integrator::kStateSize, 1> next_state;
    integrator.integrateStateOnly(
        current_state, debiased_imu_measurements, delta_time_seconds,
        &next_state);
    current_state = next_state;
  }

  pose::Transformation T_M_I_kp1;
  T_M_I_kp1.getPosition() =
      current_state.block<3, 1>(imu_integrator::kStatePositionOffset, 0);
  T_M_I_kp1.getRotation().toImplementation().coeffs() =
      current_state.block<4, 1>(imu_integrator::kStateOrientationOffset, 0);
  vi_state_kp1->set_T_M_I(T_M_I_kp1);

  vi_state_kp1->setAccBias(
      current_state.block<3, 1>(imu_integrator::kStateAccelBiasOffset, 0));
  vi_state_kp1->setGyroBias(
      current_state.block<3, 1>(imu_integrator::kStateGyroBiasOffset, 0));
  vi_state_kp1->set_v_M_I(
      current_state.block<3, 1>(imu_integrator::kStateVelocityOffset, 0));
  return true;
}

}  // namespace vio
