#ifndef VI_MAP_VI_OPTIMIZATION_TEST_HELPERS_H_
#define VI_MAP_VI_OPTIMIZATION_TEST_HELPERS_H_

#include <glog/logging.h>
#include <sensors/imu.h>

namespace vi_map {

inline void setImuSigmasConstant(const double value, ImuSigmas* imu_sigmas) {
  CHECK_NOTNULL(imu_sigmas);
  imu_sigmas->gyro_noise_density = value;
  imu_sigmas->gyro_bias_random_walk_noise_density = value;
  imu_sigmas->acc_noise_density = value;
  imu_sigmas->acc_bias_random_walk_noise_density = value;
}

inline void setImuSigmasZero(ImuSigmas* imu_sigmas) {
  setImuSigmasConstant(0.0, imu_sigmas);
}

}  // namespace vi_map

#endif  // VI_MAP_VI_OPTIMIZATION_TEST_HELPERS_H_
