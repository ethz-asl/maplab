#ifndef VIO_COMMON_INTERNAL_IMU_MEASUREMENTS_BUFFER_INL_H_
#define VIO_COMMON_INTERNAL_IMU_MEASUREMENTS_BUFFER_INL_H_

namespace vio_common {

inline void ImuMeasurementBuffer::addMeasurement(
    int64_t timestamp_nanoseconds, const vio::ImuData& imu_measurement) {
  // Enforce strict time-wise ordering.
  vio::ImuMeasurement last_value;
  if (buffer_.getNewestValue(&last_value)) {
    CHECK_GT(timestamp_nanoseconds, last_value.timestamp)
        << "Timestamps not strictly increasing.";
  }
  buffer_.addValue(
      timestamp_nanoseconds,
      vio::ImuMeasurement(timestamp_nanoseconds, imu_measurement));

  // Notify possibly waiting consumers.
  cv_new_measurement_.notify_all();
}

inline void ImuMeasurementBuffer::addMeasurements(
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps_nanoseconds,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements) {
  CHECK_EQ(timestamps_nanoseconds.cols(), imu_measurements.cols());
  size_t num_samples = timestamps_nanoseconds.cols();
  CHECK_GT(num_samples, 0u);

  for (size_t idx = 0; idx < num_samples; ++idx) {
    addMeasurement(timestamps_nanoseconds(idx), imu_measurements.col(idx));
  }
}

inline void ImuMeasurementBuffer::clear() {
  buffer_.clear();
}

inline size_t ImuMeasurementBuffer::size() const {
  return buffer_.size();
}

inline void ImuMeasurementBuffer::shutdown() {
  shutdown_ = true;
  cv_new_measurement_.notify_all();
}

}  // namespace vio_common
#endif  // VIO_COMMON_INTERNAL_IMU_MEASUREMENTS_BUFFER_INL_H_
