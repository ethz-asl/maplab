#ifndef VIO_COMMON_IMU_FORWARD_PROPAGATION_H_
#define VIO_COMMON_IMU_FORWARD_PROPAGATION_H_

#include "vio-common/imu-measurements-buffer.h"

namespace vio {

bool propagatePoseUsingImu(
    const vio::ViNodeState& vi_state_k, const double gravity_acceleration,
    const int64_t timestamp_kp1,
    const vio_common::ImuMeasurementBuffer& imu_buffer,
    vio::ViNodeState* vi_state_kp1);

}  // namespace vio

#endif  // VIO_COMMON_IMU_FORWARD_PROPAGATION_H_
