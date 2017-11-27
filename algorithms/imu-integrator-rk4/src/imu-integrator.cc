#include "imu-integrator/imu-integrator.h"

namespace imu_integrator {

ImuIntegratorRK4::ImuIntegratorRK4(
    double gyro_noise_sigma, double gyro_bias_sigma, double acc_noise_sigma,
    double acc_bias_sigma, double gravity_acceleration)
    : gravity_acceleration_(gravity_acceleration) {
  gyro_noise_sigma_squared_ = gyro_noise_sigma * gyro_noise_sigma;
  gyro_bias_sigma_squared_ = gyro_bias_sigma * gyro_bias_sigma;
  acc_noise_sigma_squared_ = acc_noise_sigma * acc_noise_sigma;
  acc_bias_sigma_squared_ = acc_bias_sigma * acc_bias_sigma;
}

}  // namespace imu_integrator
