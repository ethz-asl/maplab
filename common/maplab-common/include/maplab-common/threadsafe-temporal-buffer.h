#ifndef MAPLAB_COMMON_THREADSAFE_TEMPORAL_BUFFER_H_
#define MAPLAB_COMMON_THREADSAFE_TEMPORAL_BUFFER_H_

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <utility>

#include <glog/logging.h>
#include <maplab-common/macros.h>

#include "maplab-common/temporal-buffer.h"

namespace common {

template <typename ValueType,
          typename AllocatorType =
              std::allocator<std::pair<int64_t, ValueType> > >
class ThreadsafeTemporalBuffer final {
 public:
  MAPLAB_POINTER_TYPEDEFS(ThreadsafeTemporalBuffer);
  typedef
      typename TemporalBuffer<ValueType, AllocatorType>::BufferType BufferType;

  enum class QueryResult {
    /// Query was successful and the data is available.
    kDataAvailable,
    /// The required data is not (yet) available. The query timestamp is above
    /// the last sample's time.
    kDataNotYetAvailable,
    /// The queried timestamp lies before the first sample in the buffer.
    /// This request will never succeed considering chronological ordering
    /// of the buffer input data.
    kDataNeverAvailable,
    kQueueShutdown,
    // Too few measurements available within the requested time range.
    kTooFewMeasurementsAvailable
  };

  // Create buffer of infinite length (buffer_length_nanoseconds = -1)
  ThreadsafeTemporalBuffer() : shutdown_(false) {}
  explicit ThreadsafeTemporalBuffer(const int64_t buffer_length_ns)
      : buffer_(buffer_length_ns), shutdown_(false) {}
  ~ThreadsafeTemporalBuffer() {
    shutdown();
  }

  /// Shutdown the queue and release all blocked waiters.
  inline void shutdown();

  // Returns true if the value is inserted into the buffer without
  // overwriting a previously present value at this time, false otherwise.
  bool addValue(const int64_t timestamp_nanoseconds, const ValueType& value);
  // Inserts a value enforcing strictly increasing timestamps.
  bool pushBack(const int64_t timestamp_nanoseconds, const ValueType& value);

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.empty();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
  }

  /// Return a list of the buffer values between the specified
  /// timestamps. The values get interpolated if the queried timestamp does not
  /// match a buffer value.
  /// The return code signals if the buffer does not contain data up to the
  /// requested timestamp.
  /// In this case the output values with be empty.
  template <typename ContainerType>
  QueryResult getInterpolatedBorders(
      const int64_t timestamp_from, const int64_t timestamp_to,
      ContainerType* values) const;

  /// Try to fetch the requested buffer values for the duration of
  /// wait_timeout_nanoseconds.
  /// If the requested data is still not available when timeout has been
  /// reached, the method will return false and no data will be removed
  /// from the buffer.
  template <typename ContainerType>
  QueryResult getInterpolatedBordersBlocking(
      const int64_t timestamp_ns_from, int64_t timestamp_ns_to,
      const int64_t wait_timeout_nanoseconds, ContainerType* values) const;

  template <typename ContainerType>
  QueryResult getValuesFromExcludingToIncludingBlocking(
      const int64_t timestamp_ns_from, int64_t timestamp_ns_to,
      const int64_t wait_timeout_nanoseconds, ContainerType* values) const;

  bool getNearestValueToTime(
      const int64_t timestamp, const int64_t maximum_delta_ns, ValueType* value,
      int64_t* timestamp_at_value_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.getNearestValueToTime(
        timestamp, maximum_delta_ns, value, timestamp_at_value_ns);
  }

  bool getNearestValueToTime(
      const int64_t timestamp, const int64_t maximum_delta_ns,
      ValueType* value) {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.getNearestValueToTime(timestamp, maximum_delta_ns, value);
  }

  typedef std::function<void(const typename BufferType::value_type&)>
      ForEachFunction;
  void forEach(const ForEachFunction& for_each_function) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::for_each(std::begin(buffer_), std::end(buffer_), for_each_function);
  }

 private:
  typedef TemporalBuffer<ValueType, AllocatorType> Base;

  bool addValueImpl(
      const int64_t timestamp_nanoseconds, const ValueType& value);
  /// Is data available up to this timestamp? Note this function does not lock
  /// the buffers, the caller must hold the lock.
  QueryResult isDataAvailableUpTo(
      const int64_t timestamp_ns_from, const int64_t timestamp_ns_to) const;

  QueryResult waitForDataToBecomeAvailable(
      const int64_t timestamp_ns_from, const int64_t timestamp_ns_to,
      const int64_t wait_timeout_nanoseconds,
      std::unique_lock<std::mutex>* lock) const;

  template <typename ContainerType>
  QueryResult getInterpolatedBordersImpl(
      const int64_t timestamp_ns_from, const int64_t timestamp_ns_to,
      ContainerType* values) const;

  TemporalBuffer<ValueType, AllocatorType> buffer_;

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_new_measurement_;
  std::atomic<bool> shutdown_;
};
}  // namespace common

#include "maplab-common/threadsafe-temporal-buffer-inl.h"

#endif  // MAPLAB_COMMON_THREADSAFE_TEMPORAL_BUFFER_H_
