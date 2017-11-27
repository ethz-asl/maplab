#ifndef MAP_OPTIMIZATION_LEGACY_TEST_VI_OPTIMIZATION_TEST_HELPERS_H_
#define MAP_OPTIMIZATION_LEGACY_TEST_VI_OPTIMIZATION_TEST_HELPERS_H_

#include <glog/logging.h>
#include <sensors/imu.h>

namespace map_optimization_legacy {

inline void setImuSigmasConstant(
    const double value, vi_map::ImuSigmas* imu_sigmas) {
  CHECK_NOTNULL(imu_sigmas);
  imu_sigmas->gyro_noise_density = value;
  imu_sigmas->gyro_bias_random_walk_noise_density = value;
  imu_sigmas->acc_noise_density = value;
  imu_sigmas->acc_bias_random_walk_noise_density = value;
}

inline void setImuSigmasZero(vi_map::ImuSigmas* imu_sigmas) {
  setImuSigmasConstant(0.0, imu_sigmas);
}

}  // namespace map_optimization_legacy

#endif  // MAP_OPTIMIZATION_LEGACY_TEST_VI_OPTIMIZATION_TEST_HELPERS_H_
