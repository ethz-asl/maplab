#ifndef MAPLAB_COMMON_TEMPORAL_BUFFER_H_
#define MAPLAB_COMMON_TEMPORAL_BUFFER_H_

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <utility>

#include <glog/logging.h>
#include <maplab-common/macros.h>

namespace common {

template <typename ValueType,
          typename AllocatorType =
              std::allocator<std::pair<int64_t, ValueType> > >
class TemporalBuffer {
 public:
  typedef std::map<int64_t, ValueType, std::less<int64_t>, AllocatorType>
      BufferType;
  MAPLAB_POINTER_TYPEDEFS(TemporalBuffer);

  // Create buffer of infinite length (buffer_length_nanoseconds = -1)
  TemporalBuffer();

  // Buffer length in nanoseconds defines after which time old entries get
  // dropped. (buffer_length_nanoseconds == -1: infinite length.)
  explicit TemporalBuffer(int64_t buffer_length_nanoseconds);

  TemporalBuffer(const TemporalBuffer& other);

  void addValue(int64_t timestamp, const ValueType& value);
  void addValue(
      const int64_t timestamp, const ValueType& value,
      const bool emit_warning_on_value_overwrite);
  void insert(const TemporalBuffer& other);

  inline size_t size() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return values_.size();
  }
  inline bool empty() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return values_.empty();
  }
  void clear() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    values_.clear();
  }

  // Returns false if no value at a given timestamp present.
  bool getValueAtTime(int64_t timestamp_ns, ValueType* value) const;

  bool deleteValueAtTime(int64_t timestamp_ns);

  bool getNearestValueToTime(int64_t timestamp_ns, ValueType* value) const;
  bool getNearestValueToTime(
      int64_t timestamp_ns, int64_t maximum_delta_ns, ValueType* value) const;
  bool getNearestValueToTime(
      int64_t timestamp, int64_t maximum_delta_ns, ValueType* value,
      int64_t* timestamp_at_value_ns) const;

  bool getOldestValue(ValueType* value) const;
  bool getNewestValue(ValueType* value) const;

  bool getValueAtOrBeforeTime(
      int64_t timestamp_ns, int64_t* timestamp_ns_of_value,
      ValueType* value) const;
  bool getValueAtOrAfterTime(
      int64_t timestamp_ns, int64_t* timestamp_ns_of_value,
      ValueType* value) const;

  // Get all values between the two specified timestamps excluding the border
  // values.
  // Example: content: 2 3 4 5
  //          getValuesBetweenTimes(2, 5, ...) returns elements at 3, 4.
  template <typename ValueContainerType>
  bool getValuesBetweenTimes(
      int64_t timestamp_lower_ns, int64_t timestamp_higher_ns,
      ValueContainerType* values) const;

  inline void lockContainer() const {
    mutex_.lock();
  }
  inline void unlockContainer() const {
    mutex_.unlock();
  }

  // The container is exposed so we can iterate over the values in a linear
  // fashion. The container is not locked inside this method so call
  // lockContainer()/unlockContainer() when accessing this.
  const BufferType& buffered_values() const {
    return values_;
  }

  inline bool operator==(const TemporalBuffer& other) const {
    return values_ == other.values_ &&
           buffer_length_nanoseconds_ == other.buffer_length_nanoseconds_;
  }

 protected:
  // Remove items that are older than the buffer length.
  void removeOutdatedItems();

  bool getIteratorAtTimeOrEarlier(
      int64_t timestamp,
      typename BufferType::const_iterator* it_lower_bound) const;

  BufferType values_;
  int64_t buffer_length_nanoseconds_;
  mutable std::recursive_mutex mutex_;
};
}  // namespace common

#include "./temporal-buffer-inl.h"

#endif  // MAPLAB_COMMON_TEMPORAL_BUFFER_H_
