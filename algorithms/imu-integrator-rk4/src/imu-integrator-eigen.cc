#include "imu-integrator/imu-integrator-eigen.h"

namespace imu_integrator {

ImuIntegratorEigen::ImuIntegratorEigen(
    double gyro_noise_sigma, double gyro_bias_sigma, double acc_noise_sigma,
    double acc_bias_sigma, double gravity_acceleration)
    : gyro_noise_sigma_squared_(gyro_noise_sigma * gyro_noise_sigma),
      gyro_bias_sigma_squared_(gyro_bias_sigma * gyro_bias_sigma),
      acc_noise_sigma_squared_(acc_noise_sigma * acc_noise_sigma),
      acc_bias_sigma_squared_(acc_bias_sigma * acc_bias_sigma),
      gravity_acceleration_(gravity_acceleration) {}

}  // namespace imu_integrator
