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
    kFailedNotYetAvailable,
    kFailedWillNeverSucceed
  };

  PoseLookupBuffer(
      int64_t pose_buffer_length_ns, int64_t max_allowed_propagation_time_ns);

  ResultStatus getPoseAt(
      int64_t timestamp_ns, aslam::Transformation* T_M_I) const;

  // Only the gyro and gyro biases are used to obtain this rotation.
  ResultStatus getRotationBetween(
      const int64_t kp1_timestamp_ns, const int64_t k_timestamp_ns,
      aslam::Quaternion* q_Ikp1_Ik) const;

  ResultStatus interpolateViNodeStateAt(
      const int64_t timestamp_ns, vio::ViNodeState* interpolated_vinode) const;

  void bufferImuMeasurement(const vio::ImuMeasurement& imu);
  void bufferOdometryEstimate(const vio::ViNodeState& estimate);

  bool getTimeRangeOfAvailablePoses(
      int64_t* oldest_timestamp_ns, int64_t* newest_timestamp_ns) const;
  bool getNewestTimestampOfAvailablePose(int64_t* timestamp_ns) const;
  bool getOldestTimestampOfAvailablePose(int64_t* timestamp_ns) const;
  bool getValueAtOrAfterTime(
      int64_t timestamp_ns, vio::ViNodeState* vinode) const;

  const vio_common::ImuMeasurementBuffer& imu_buffer() const;
  vio_common::ImuMeasurementBuffer& imu_buffer_mutable();

  void shutdown();

 private:
  const int64_t max_allowed_propagation_time_ns_;
  const int64_t pose_buffer_length_ns_;

  mutable std::mutex odometry_mutex_;
  common::TemporalBuffer<vio::ViNodeState> buffer_odometry_estimate_;
  common::TemporalBuffer<Eigen::Vector3d> gyro_bias_buffer_;

  // Doesn't need a mutex, since it is already thread-safe.
  vio_common::ImuMeasurementBuffer imu_buffer_;

  common::GravityProvider gravity_provider_;
};
}  //  namespace vio_common
#endif  // VIO_COMMON_POSE_LOOKUP_BUFFER_H_
