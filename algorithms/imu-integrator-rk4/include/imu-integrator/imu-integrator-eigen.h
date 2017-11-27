#ifndef IMU_INTEGRATOR_IMU_INTEGRATOR_EIGEN_H_
#define IMU_INTEGRATOR_IMU_INTEGRATOR_EIGEN_H_

#include <Eigen/Core>
#include <maplab-common/pose_types.h>

#include "imu-integrator/common.h"

namespace imu_integrator {

// Note: this class accepts rotations expressed as quaternions
// in Hamilton(Eigen) convention.
// However, according to Eigen the coefficients are stored as [x, y, z, w].
// This convention corresponds to the internal
// coefficient storage of Eigen so you can directly construct your state vector
// from the coeffs() vector of your Eigen quaternion.
class ImuIntegratorEigen {
  template <class ScalarType>
  using Vector3 = Eigen::Matrix<ScalarType, 3, 1>;
  template <class ScalarType>
  using Matrix3 = Eigen::Matrix<ScalarType, 3, 3>;
  template <class ScalarType>
  using StateJacobian = Eigen::Matrix<
      ScalarType, kErrorStateSize, kErrorStateSize>;
  template <class ScalarType>
  using CovarianceMatrix = Eigen::Matrix<
      ScalarType, kErrorStateSize, kErrorStateSize>;

 public:
  ImuIntegratorEigen(
      double gyro_noise_sigma, double gyro_bias_sigma, double acc_noise_sigma,
      double acc_bias_sigma, double gravity_acceleration);

  template <typename ScalarType>
  inline void integrateStateOnly(
      const Eigen::Matrix<ScalarType, kStateSize, 1>& current_state,
      const Eigen::Matrix<ScalarType, 2 * kImuReadingSize, 1>&
          debiased_imu_readings,
      const ScalarType delta_time_seconds,
      Eigen::Matrix<ScalarType, kStateSize, 1>* next_state) const {
    Eigen::Matrix<
        ScalarType, kErrorStateSize, kErrorStateSize>*
        null_pointer = NULL;
    integrate(
        current_state, debiased_imu_readings, delta_time_seconds, next_state,
        null_pointer, null_pointer);
  }

  /// The (next_phi, next_cov) calculation is optional and can be disabled by
  /// passing two nullptr.
  template <typename ScalarType>
  inline void integrate(
      const Eigen::Matrix<
          ScalarType, kStateSize, 1, 0, kStateSize, 1>&
          current_state,
      const Eigen::Matrix<ScalarType, 2 * kImuReadingSize, 1>&
          debiased_imu_readings,
      const ScalarType delta_time_seconds,
      Eigen::Matrix<ScalarType, kStateSize, 1>* next_state,
      StateJacobian<ScalarType>* next_phi,
      CovarianceMatrix<ScalarType>* next_cov) const;

 private:
  template <typename ScalarType>
  void getJacobianAndCovariance(
      const Eigen::Matrix<
          ScalarType, kStateSize, 1>&
          current_state,
      const Eigen::Matrix<ScalarType, kImuReadingSize, 1>&
          debiased_imu_reading,
      const ScalarType delta_time_seconds, StateJacobian<ScalarType>* next_phi,
      CovarianceMatrix<ScalarType>* next_cov) const;

  template <typename ScalarType>
  inline void interpolateImuReadings(
      const Eigen::Matrix<ScalarType, 2 * kImuReadingSize, 1>& imu_readings,
      const ScalarType delta_time_seconds,
      const ScalarType increment_step_size_seconds,
      Eigen::Matrix<ScalarType, kImuReadingSize, 1>* interpolated_imu_readings)
      const;

  const double gyro_noise_sigma_squared_;
  const double gyro_bias_sigma_squared_;

  const double acc_noise_sigma_squared_;
  const double acc_bias_sigma_squared_;

  const double gravity_acceleration_;
};

}  // namespace imu_integrator

#include "imu-integrator/imu-integrator-eigen-inl.h"

#endif  // IMU_INTEGRATOR_IMU_INTEGRATOR_EIGEN_H_
