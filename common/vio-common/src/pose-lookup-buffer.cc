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
    int64_t pose_buffer_length_ns, int64_t max_allowed_propagation_time_ns)
    : max_allowed_propagation_time_ns_(max_allowed_propagation_time_ns),
      pose_buffer_length_ns_(pose_buffer_length_ns),
      buffer_odometry_estimate_(pose_buffer_length_ns),
      gyro_bias_buffer_(pose_buffer_length_ns),
      imu_buffer_(pose_buffer_length_ns),
      gravity_provider_(
          common::locations::kAltitudeZurichMeters,
          common::locations::kLatitudeZurichDegrees) {
  CHECK(aslam::time::isValidTime(pose_buffer_length_ns));
  CHECK(aslam::time::isValidTime(max_allowed_propagation_time_ns));

  VLOG(1) << " Initialized pose lookup buffer if length "
          << pose_buffer_length_ns << "ns";
}

const vio_common::ImuMeasurementBuffer& PoseLookupBuffer::imu_buffer() const {
  return imu_buffer_;
}

vio_common::ImuMeasurementBuffer& PoseLookupBuffer::imu_buffer_mutable() {
  return imu_buffer_;
}

void PoseLookupBuffer::shutdown() {
  imu_buffer_.shutdown();
}

PoseLookupBuffer::ResultStatus PoseLookupBuffer::getRotationBetween(
    const int64_t kp1_timestamp_ns, const int64_t k_timestamp_ns,
    aslam::Quaternion* q_Ikp1_Ik) const {
  CHECK_NOTNULL(q_Ikp1_Ik)->setIdentity();
  Eigen::Vector3d most_recent_gyro_bias;
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements;

  const int64_t delta_ns = kp1_timestamp_ns - k_timestamp_ns;
  CHECK_GT(delta_ns, 0);
  const int64_t kWaitTimeoutNanoseconds = 0;
  vio_common::ImuMeasurementBuffer::QueryResult result;
  result = imu_buffer_.getImuDataInterpolatedBordersBlocking(
      k_timestamp_ns, kp1_timestamp_ns, kWaitTimeoutNanoseconds,
      &imu_timestamps, &imu_measurements);

  switch (result) {
    case vio_common::ImuMeasurementBuffer::QueryResult::kDataNotYetAvailable:
      LOG(ERROR) << "IMU data for rotation estimate not yet available! The "
                 << "synchronizer should never allow this!";
      return ResultStatus::kFailedNotYetAvailable;
    case vio_common::ImuMeasurementBuffer::QueryResult::kDataNeverAvailable:
      LOG(WARNING) << "IMU data for rotation estimate not available anymore!";
      return ResultStatus::kFailedWillNeverSucceed;
    case vio_common::ImuMeasurementBuffer::QueryResult::kQueueShutdown:
      LOG(ERROR) << "IMU data for rotation cannot be retrieved, IMU buffer has "
                 << "shut down!";
      return ResultStatus::kFailedWillNeverSucceed;
    case vio_common::ImuMeasurementBuffer::QueryResult::kDataAvailable:
      // success
      break;
    default:
      LOG(FATAL) << "Unknown ImuMeasurementBuffer::QueryResult "
                 << static_cast<int>(result);
  }

  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    if (!gyro_bias_buffer_.getNearestValueToTime(
            kp1_timestamp_ns, pose_buffer_length_ns_, &most_recent_gyro_bias)) {
      LOG(ERROR) << "No gyro bias available for rotation estimation. Assuming "
                 << "zero bias. The synchronizer should never allow this!";
      most_recent_gyro_bias.setZero();
    }
  }

  Eigen::Vector3d gyro_measurement;
  double delta_s;
  CHECK_GE(imu_measurements.cols(), 2);
  for (int i = 1; i < imu_measurements.cols(); ++i) {
    delta_s = (imu_timestamps(i) - imu_timestamps(i - 1)) * 1e-9;
    CHECK_GT(delta_s, 0);
    gyro_measurement =
        imu_measurements.col(i).tail<3>() - most_recent_gyro_bias;
    *q_Ikp1_Ik =
        *q_Ikp1_Ik * aslam::Quaternion::exp(gyro_measurement * delta_s);
  }

  *q_Ikp1_Ik = q_Ikp1_Ik->inverse();
  return ResultStatus::kSuccessImuForwardPropagation;
}

PoseLookupBuffer::ResultStatus PoseLookupBuffer::interpolateViNodeStateAt(
    const int64_t timestamp_ns, vio::ViNodeState* interpolated_vinode) const {
  CHECK_NOTNULL(interpolated_vinode);
  std::lock_guard<std::mutex> lock(odometry_mutex_);

  // First try to interpolate the pose.
  if (buffer_odometry_estimate_.empty()) {
    return ResultStatus::kFailedNotYetAvailable;
  }

  // Early exit if the timestamp_ns is older than the buffered values.
  vio::ViNodeState oldest_rovio_estimate;
  if (buffer_odometry_estimate_.getOldestValue(&oldest_rovio_estimate) &&
      timestamp_ns < oldest_rovio_estimate.getTimestamp()) {
    return ResultStatus::kFailedWillNeverSucceed;
  }

  // First try to interpolate the pose.
  if (buffer_odometry_estimate_.interpolateAt(
          timestamp_ns, interpolated_vinode)) {
    return ResultStatus::kSuccessInterpolated;
  }

  // If that failed, propagate the pose using IMU data.
  vio::ViNodeState most_recent_estimate;
  CHECK(buffer_odometry_estimate_.getNewestValue(&most_recent_estimate));
  CHECK_GT(timestamp_ns, most_recent_estimate.getTimestamp());

  const int64_t propagation_time_nanoseconds =
      most_recent_estimate.getTimestamp() - timestamp_ns;
  if (propagation_time_nanoseconds > max_allowed_propagation_time_ns_) {
    return ResultStatus::kFailedNotYetAvailable;
  }

  if (!vio::propagatePoseUsingImu(
          most_recent_estimate, gravity_provider_.getGravityMagnitude(),
          timestamp_ns, imu_buffer_, interpolated_vinode)) {
    return ResultStatus::kFailedNotYetAvailable;
  }
  return ResultStatus::kSuccessImuForwardPropagation;
}

PoseLookupBuffer::ResultStatus PoseLookupBuffer::getPoseAt(
    int64_t timestamp_ns, aslam::Transformation* T_M_I) const {
  CHECK_NOTNULL(T_M_I);
  std::lock_guard<std::mutex> lock(odometry_mutex_);

  // First try to interpolate the pose.
  if (buffer_odometry_estimate_.empty()) {
    return ResultStatus::kFailedNotYetAvailable;
  }

  // Early exit if the timestamp_ns is older than the buffered values.
  vio::ViNodeState oldest_rovio_estimate;
  if (buffer_odometry_estimate_.getOldestValue(&oldest_rovio_estimate) &&
      timestamp_ns < oldest_rovio_estimate.getTimestamp()) {
    return ResultStatus::kFailedWillNeverSucceed;
  }

  // TODO(mfehr): check if this is using the IMU or not, and find a solution so
  // this does not have to interpolate the full VINodeState.

  // First try to interpolate the pose.
  vio::ViNodeState estimate_interpolated;
  if (buffer_odometry_estimate_.interpolateAt(
          timestamp_ns, &estimate_interpolated)) {
    *T_M_I = estimate_interpolated.get_T_M_I();
    return ResultStatus::kSuccessInterpolated;
  }

  // If that failed, propagate the pose using IMU data.
  vio::ViNodeState most_recent_estimate;
  CHECK(buffer_odometry_estimate_.getNewestValue(&most_recent_estimate));
  CHECK_GT(timestamp_ns, most_recent_estimate.getTimestamp());

  const int64_t propagation_time_nanoseconds =
      most_recent_estimate.getTimestamp() - timestamp_ns;
  if (propagation_time_nanoseconds > max_allowed_propagation_time_ns_) {
    return ResultStatus::kFailedNotYetAvailable;
  }

  vio::ViNodeState estimate_propagated;
  if (!vio::propagatePoseUsingImu(
          most_recent_estimate, gravity_provider_.getGravityMagnitude(),
          timestamp_ns, imu_buffer_, &estimate_propagated)) {
    return ResultStatus::kFailedNotYetAvailable;
  }
  *T_M_I = estimate_propagated.get_T_M_I();
  return ResultStatus::kSuccessImuForwardPropagation;
}

void PoseLookupBuffer::bufferImuMeasurement(const vio::ImuMeasurement& imu) {
  imu_buffer_.addMeasurement(imu.timestamp, imu.imu_data);
}

void PoseLookupBuffer::bufferOdometryEstimate(
    const vio::ViNodeState& estimate) {
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  CHECK_GE(estimate.getTimestamp(), 0);
  buffer_odometry_estimate_.addValue(estimate.getTimestamp(), estimate);
  gyro_bias_buffer_.addValue(estimate.getTimestamp(), estimate.getGyroBias());
}

bool PoseLookupBuffer::getTimeRangeOfAvailablePoses(
    int64_t* oldest_timestamp_ns, int64_t* newest_timestamp_ns) const {
  CHECK_NOTNULL(oldest_timestamp_ns);
  CHECK_NOTNULL(newest_timestamp_ns);

  bool success = true;
  int64_t min_odometry_timestamp_ns, max_odometry_timestamp_ns;
  success &=
      buffer_odometry_estimate_.getOldestTime(&min_odometry_timestamp_ns);
  success &=
      buffer_odometry_estimate_.getNewestTime(&max_odometry_timestamp_ns);
  int64_t min_imu_timestamp_ns, max_imu_timestamp_ns;
  success &= imu_buffer_.getOldestTime(&min_imu_timestamp_ns);
  success &= imu_buffer_.getNewestTime(&max_imu_timestamp_ns);

  const int64_t min_timestamp_ns =
      std::max(min_odometry_timestamp_ns, min_imu_timestamp_ns);
  const int64_t max_timestamp_ns =
      std::min(max_odometry_timestamp_ns, max_imu_timestamp_ns);

  if (!success || (min_timestamp_ns > max_timestamp_ns)) {
    // If this is the case there is no common time range where IMU and odom is
    // available.
    return false;
  }

  *oldest_timestamp_ns = min_timestamp_ns;
  *newest_timestamp_ns = max_timestamp_ns;

  return true;
}

bool PoseLookupBuffer::getNewestTimestampOfAvailablePose(
    int64_t* timestamp_ns) const {
  CHECK_NOTNULL(timestamp_ns);

  bool success = true;
  int64_t odometry_timestamp_ns;
  success &= buffer_odometry_estimate_.getNewestTime(&odometry_timestamp_ns);
  int64_t imu_timestamp_ns;
  success &= imu_buffer_.getNewestTime(&imu_timestamp_ns);
  *timestamp_ns = std::min(odometry_timestamp_ns, imu_timestamp_ns);
  return success;
}

bool PoseLookupBuffer::getOldestTimestampOfAvailablePose(
    int64_t* timestamp_ns) const {
  CHECK_NOTNULL(timestamp_ns);

  bool success = true;
  int64_t odometry_timestamp_ns;
  success &= buffer_odometry_estimate_.getOldestTime(&odometry_timestamp_ns);
  int64_t imu_timestamp_ns;
  success &= imu_buffer_.getOldestTime(&imu_timestamp_ns);
  *timestamp_ns = std::max(odometry_timestamp_ns, imu_timestamp_ns);
  return success;
}

bool PoseLookupBuffer::getValueAtOrAfterTime(
    int64_t timestamp_ns, vio::ViNodeState* vinode) const {
  int64_t tmp;
  return buffer_odometry_estimate_.getValueAtOrAfterTime(
      timestamp_ns, &tmp, vinode);
}

}  //  namespace vio_common
