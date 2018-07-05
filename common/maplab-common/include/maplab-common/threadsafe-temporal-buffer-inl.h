#ifndef MAPLAB_COMMON_THREADSAFE_TEMPORAL_BUFFER_INL_H_
#define MAPLAB_COMMON_THREADSAFE_TEMPORAL_BUFFER_INL_H_

#include <aslam/common/time.h>

namespace common {

template <typename ValueType, typename AllocatorType>
inline bool ThreadsafeTemporalBuffer<ValueType, AllocatorType>::addValue(
    const int64_t timestamp_nanoseconds, const ValueType& value) {
  std::lock_guard<std::mutex> lock(mutex_);
  return addValueImpl(timestamp_nanoseconds, value);
}

template <typename ValueType, typename AllocatorType>
inline bool ThreadsafeTemporalBuffer<ValueType, AllocatorType>::addValueImpl(
    const int64_t timestamp_nanoseconds, const ValueType& value) {
  const bool no_value_overwritten =
      buffer_.addValue(timestamp_nanoseconds, value);
  // Notify possibly waiting consumers.
  cv_new_measurement_.notify_all();

  return no_value_overwritten;
}

template <typename ValueType, typename AllocatorType>
inline bool ThreadsafeTemporalBuffer<ValueType, AllocatorType>::pushBack(
    const int64_t timestamp_nanoseconds, const ValueType& value) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!buffer_.empty()) {
    const int64_t most_recent_timestamp_nanoseconds_ = buffer_.rbegin()->first;
    LOG_IF(FATAL, timestamp_nanoseconds < most_recent_timestamp_nanoseconds_)
        << "Data must be added in chronological order.";
  }

  return addValueImpl(timestamp_nanoseconds, value);
}

template <typename ValueType, typename AllocatorType>
inline void ThreadsafeTemporalBuffer<ValueType, AllocatorType>::shutdown() {
  shutdown_ = true;
  cv_new_measurement_.notify_all();
}

template <typename ValueType, typename AllocatorType>
typename ThreadsafeTemporalBuffer<ValueType, AllocatorType>::QueryResult
ThreadsafeTemporalBuffer<ValueType, AllocatorType>::isDataAvailableUpTo(
    const int64_t timestamp_ns_from, const int64_t timestamp_ns_to) const {
  CHECK_LT(timestamp_ns_from, timestamp_ns_to);

  if (buffer_.empty()) {
    return QueryResult::kDataNotYetAvailable;
  }

  int64_t timestamp_nanoseconds;
  const bool get_latest_time_success =
      buffer_.getNewestTime(&timestamp_nanoseconds);
  if (get_latest_time_success && timestamp_nanoseconds < timestamp_ns_to) {
    return QueryResult::kDataNotYetAvailable;
  }

  if (buffer_.getOldestTime(&timestamp_nanoseconds) &&
      (timestamp_nanoseconds >= timestamp_ns_to ||
       timestamp_ns_from < timestamp_nanoseconds)) {
    return QueryResult::kDataNeverAvailable;
  }
  return QueryResult::kDataAvailable;
}

template <typename ValueType, typename AllocatorType>
template <typename ContainerType>
typename ThreadsafeTemporalBuffer<ValueType, AllocatorType>::QueryResult
ThreadsafeTemporalBuffer<ValueType, AllocatorType>::getInterpolatedBorders(
    const int64_t timestamp_ns_from, const int64_t timestamp_ns_to,
    ContainerType* values) const {
  std::lock_guard<std::mutex> lock(mutex_);

  QueryResult query_result =
      isDataAvailableUpTo(timestamp_ns_from, timestamp_ns_to);
  if (query_result == QueryResult::kDataAvailable) {
    query_result =
        getInterpolatedBordersImpl(timestamp_ns_from, timestamp_ns_to, values);
  }
  return query_result;
}

template <typename ValueType, typename AllocatorType>
template <typename ContainerType>
typename ThreadsafeTemporalBuffer<ValueType, AllocatorType>::QueryResult
ThreadsafeTemporalBuffer<ValueType, AllocatorType>::getInterpolatedBordersImpl(
    const int64_t timestamp_ns_from, const int64_t timestamp_ns_to,
    ContainerType* values) const {
  CHECK_NOTNULL(values)->clear();

  // Copy the data with timestamp_up_to <= timestamps_buffer from the buffer.
  ContainerType between_values;
  buffer_.getValuesBetweenTimes(
      timestamp_ns_from, timestamp_ns_to, &between_values);

  if (between_values.empty()) {
    LOG(WARNING) << "Too few buffer values available between time "
                 << timestamp_ns_from << "[ns] and " << timestamp_ns_to
                 << "[ns].";
    return QueryResult::kTooFewMeasurementsAvailable;
  }

  // The first and last index will be replaced with the interpolated values.
  const size_t num_measurements = between_values.size() + 2u;
  values->reserve(num_measurements);

  // Interpolate lower border.
  int64_t pre_border_timestamp;
  ValueType pre_border_value;
  bool success = buffer_.getValueAtOrBeforeTime(
      timestamp_ns_from, &pre_border_timestamp, &pre_border_value);
  CHECK(success);

  int64_t post_border_timestamp;
  ValueType post_border_value;
  success = buffer_.getValueAtOrAfterTime(
      timestamp_ns_from, &post_border_timestamp, &post_border_value);
  CHECK(success);

  ValueType lower_border_value;
  common::linearInterpolation(
      pre_border_timestamp, pre_border_value, post_border_timestamp,
      post_border_value, timestamp_ns_from, &lower_border_value);

  values->emplace_back(lower_border_value);

  values->insert(values->end(), between_values.begin(), between_values.end());
  CHECK_EQ(values->size(), num_measurements - 1u);

  // Interpolate upper boarder values.
  success = buffer_.getValueAtOrBeforeTime(
      timestamp_ns_to, &pre_border_timestamp, &pre_border_value);
  CHECK(success);
  success = buffer_.getValueAtOrAfterTime(
      timestamp_ns_to, &post_border_timestamp, &post_border_value);
  CHECK(success);

  ValueType upper_border_value;
  common::linearInterpolation(
      pre_border_timestamp, pre_border_value, post_border_timestamp,
      post_border_value, timestamp_ns_to, &upper_border_value);

  values->emplace_back(upper_border_value);

  return QueryResult::kDataAvailable;
}

template <typename ValueType, typename AllocatorType>
typename ThreadsafeTemporalBuffer<ValueType, AllocatorType>::QueryResult
ThreadsafeTemporalBuffer<ValueType, AllocatorType>::
    waitForDataToBecomeAvailable(
        const int64_t timestamp_ns_from, const int64_t timestamp_ns_to,
        const int64_t wait_timeout_nanoseconds,
        std::unique_lock<std::mutex>* lock) const {
  CHECK(CHECK_NOTNULL(lock)->owns_lock());

  // Wait for the buffer to contain the required data within a timeout.
  const int64_t time_start = aslam::time::nanoSecondsSinceEpoch();
  QueryResult query_result = QueryResult::kDataNotYetAvailable;

  while ((query_result =
              isDataAvailableUpTo(timestamp_ns_from, timestamp_ns_to)) ==
         QueryResult::kDataNotYetAvailable) {
    cv_new_measurement_.wait_for(
        *lock, std::chrono::nanoseconds(wait_timeout_nanoseconds));

    if (shutdown_) {
      query_result = QueryResult::kQueueShutdown;
      break;
    }

    // Check if we hit the max. time allowed to wait for the required data.
    int64_t total_elapsed_time_nanoseconds =
        aslam::time::nanoSecondsSinceEpoch() - time_start;
    if (total_elapsed_time_nanoseconds >= wait_timeout_nanoseconds) {
      break;
    }
  }
  switch (query_result) {
    case QueryResult::kDataAvailable:
      break;
    case QueryResult::kDataNeverAvailable:
      LOG(WARNING) << "The requested buffer data will never be available. "
                   << "Either the buffer is too small or a sync issue "
                   << "occurred.";
      break;
    case QueryResult::kDataNotYetAvailable:
      LOG(WARNING) << "The requested buffer data is not yet available after "
                   << "waiting for maximum allowed time.";
      break;
    case QueryResult::kQueueShutdown:
      break;
    default:
      LOG(FATAL) << "Unexpected query result: "
                 << static_cast<int>(query_result);
      break;
  }
  return query_result;
}

template <typename ValueType, typename AllocatorType>
template <typename ContainerType>
typename ThreadsafeTemporalBuffer<ValueType, AllocatorType>::QueryResult
ThreadsafeTemporalBuffer<ValueType, AllocatorType>::
    getInterpolatedBordersBlocking(
        const int64_t timestamp_ns_from, const int64_t timestamp_ns_to,
        const int64_t wait_timeout_nanoseconds, ContainerType* values) const {
  CHECK_NOTNULL(values)->clear();

  std::unique_lock<std::mutex> lock(mutex_);
  QueryResult query_result = waitForDataToBecomeAvailable(
      timestamp_ns_from, timestamp_ns_to, wait_timeout_nanoseconds, &lock);
  CHECK(lock.owns_lock());
  if (query_result == QueryResult::kDataAvailable) {
    query_result =
        getInterpolatedBordersImpl(timestamp_ns_from, timestamp_ns_to, values);
  }
  return query_result;
}

template <typename ValueType, typename AllocatorType>
template <typename ContainerType>
typename ThreadsafeTemporalBuffer<ValueType, AllocatorType>::QueryResult
ThreadsafeTemporalBuffer<ValueType, AllocatorType>::
    getValuesFromExcludingToIncludingBlocking(
        const int64_t timestamp_ns_from, const int64_t timestamp_ns_to,
        const int64_t wait_timeout_nanoseconds, ContainerType* values) const {
  CHECK_NOTNULL(values)->clear();

  std::unique_lock<std::mutex> lock(mutex_);
  QueryResult query_result = waitForDataToBecomeAvailable(
      timestamp_ns_from, timestamp_ns_to, wait_timeout_nanoseconds, &lock);

  if (query_result == QueryResult::kDataAvailable) {
    buffer_.getValuesFromExcludingToIncluding(
        timestamp_ns_from, timestamp_ns_to, values);
  }

  return query_result;
}

}  // namespace common
#endif  // MAPLAB_COMMON_THREADSAFE_TEMPORAL_BUFFER_INL_H_
