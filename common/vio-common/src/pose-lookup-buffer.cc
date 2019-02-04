#include "vio-common/pose-lookup-buffer.h"

#include <mutex>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <aslam/common/time.h>
#include <maplab-common/interpolation-helpers.h>
#include <maplab-common/temporal-buffer.h>
#include <vio-common/imu-forward-propagation.h>
#include <vio-common/imu-measurements-buffer.h>
#include <vio-common/vio-types.h>

namespace vio_common {
PoseLookupBuffer::PoseLookupBuffer(
    int64_t pose_buffer_length_nanoseconds,
    int64_t max_allowed_propagation_time_ns)
    : max_allowed_propagation_time_ns_(max_allowed_propagation_time_ns),
      buffer_rovio_estimate_(pose_buffer_length_nanoseconds),
      imu_buffer_(max_allowed_propagation_time_ns_),
      gravity_provider_(
          common::locations::kAltitudeZurichMeters,
          common::locations::kLatitudeZurichDegrees) {
  CHECK(aslam::time::isValidTime(pose_buffer_length_nanoseconds));
  CHECK(aslam::time::isValidTime(max_allowed_propagation_time_ns));
}

PoseLookupBuffer::ResultStatus PoseLookupBuffer::getPoseAt(
    int64_t timestamp_ns, aslam::Transformation* T_M_I) const {
  CHECK_NOTNULL(T_M_I);
  std::lock_guard<std::mutex> lock(mutex_);

  // Early exit if the timestamp_ns is older than the buffered values.
  vio::ViNodeState oldest_rovio_estimate;
  if (buffer_rovio_estimate_.getOldestValue(&oldest_rovio_estimate) &&
      timestamp_ns < oldest_rovio_estimate.getTimestamp()) {
    return ResultStatus::kFailed;
  }

  // First try to interpolate the pose.
  vio::ViNodeState estimate_interpolated;
  if (buffer_rovio_estimate_.interpolateAt(
          timestamp_ns, &estimate_interpolated)) {
    *T_M_I = estimate_interpolated.get_T_M_I();
    return ResultStatus::kSuccessInterpolated;
  }

  // If that failed, propagate the pose using IMU data.
  vio::ViNodeState most_recent_estimate;
  CHECK(buffer_rovio_estimate_.getNewestValue(&most_recent_estimate));
  CHECK_GT(timestamp_ns, most_recent_estimate.getTimestamp());

  const int64_t propagation_time_nanoseconds =
      most_recent_estimate.getTimestamp() - timestamp_ns;
  if (propagation_time_nanoseconds > max_allowed_propagation_time_ns_) {
    return ResultStatus::kFailed;
  }

  vio::ViNodeState estimate_propagated;
  if (!vio::propagatePoseUsingImu(
          most_recent_estimate, gravity_provider_.getGravityMagnitude(),
          timestamp_ns, imu_buffer_, &estimate_propagated)) {
    return ResultStatus::kFailed;
  }
  *T_M_I = estimate_propagated.get_T_M_I();
  return ResultStatus::kSuccessImuForwardPropagation;
}

void PoseLookupBuffer::bufferImuMeasurement(const vio::ImuMeasurement& imu) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_buffer_.addMeasurement(imu.timestamp, imu.imu_data);
}

void PoseLookupBuffer::bufferRovioEstimate(const vio::ViNodeState& estimate) {
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_rovio_estimate_.addValue(estimate.getTimestamp(), estimate);
}
}  //  namespace vio_common
