#ifndef VIO_COMMON_POSE_LOOKUP_BUFFER_H_
#define VIO_COMMON_POSE_LOOKUP_BUFFER_H_

#include <mutex>

#include <aslam/common/pose-types.h>
#include <maplab-common/gravity-provider.h>
#include <maplab-common/temporal-buffer.h>
#include <vio-common/imu-measurements-buffer.h>
#include <vio-common/vio-types.h>

namespace vio_common {
// Buffers the most recent pose estimates and IMU data to provide poses at
// arbitary timestamps by either interpolation or IMU forward propagation.
class PoseLookupBuffer {
 public:
  enum class ResultStatus {
    kSuccessInterpolated,
    kSuccessImuForwardPropagation,
    kFailed
  };

  PoseLookupBuffer(
      int64_t pose_buffer_length_nanoseconds,
      int64_t max_allowed_propagation_time_ns);

  ResultStatus getPoseAt(
      int64_t timestamp_ns, aslam::Transformation* T_M_I) const;

  void bufferImuMeasurement(const vio::ImuMeasurement& imu);
  void bufferRovioEstimate(const vio::ViNodeState& estimate);

 private:
  const int64_t max_allowed_propagation_time_ns_;

  mutable std::mutex mutex_;
  common::TemporalBuffer<vio::ViNodeState> buffer_rovio_estimate_;
  vio_common::ImuMeasurementBuffer imu_buffer_;

  common::GravityProvider gravity_provider_;
};
}  //  namespace vio_common
#endif  // VIO_COMMON_POSE_LOOKUP_BUFFER_H_
