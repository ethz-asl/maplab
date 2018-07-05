#ifndef MAPLAB_COMMON_TEMPORAL_BUFFER_H_
#define MAPLAB_COMMON_TEMPORAL_BUFFER_H_

#include <functional>
#include <map>
#include <memory>
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
  virtual ~TemporalBuffer() = default;

  // Buffer length in nanoseconds defines after which time old entries get
  // dropped. (buffer_length_nanoseconds == -1: infinite length.)
  explicit TemporalBuffer(int64_t buffer_length_nanoseconds);

  TemporalBuffer(const TemporalBuffer& other);

  // Returns false if an already existing value at the given time got
  // overwritten, true otherwise.
  bool addValue(const int64_t timestamp, const ValueType& value);

  void insert(const TemporalBuffer& other) {
    values_.insert(other.begin(), other.end());
  }

  size_t size() const {
    return values_.size();
  }
  bool empty() const {
    return values_.empty();
  }
  void clear() {
    values_.clear();
  }

  // Returns false if no value at a given timestamp present.
  bool getValueAtTime(const int64_t timestamp_ns, ValueType* value) const;

  bool deleteValueAtTime(const int64_t timestamp_ns);

  bool getNearestValueToTime(
      const int64_t timestamp_ns, ValueType* value) const;
  bool getNearestValueToTime(
      const int64_t timestamp_ns, const int64_t maximum_delta_ns,
      ValueType* value) const;
  bool getNearestValueToTime(
      const int64_t timestamp, const int64_t maximum_delta_ns, ValueType* value,
      int64_t* timestamp_at_value_ns) const;

  bool getOldestTime(int64_t* timestamp_nanoseconds) const;
  bool getOldestValue(ValueType* value) const;

  bool getNewestTime(int64_t* timestamp_nanoseconds) const;
  bool getNewestValue(ValueType* value) const;

  bool getValueAtOrBeforeTime(
      const int64_t timestamp_ns, int64_t* timestamp_ns_of_value,
      ValueType* value) const;
  bool getValueAtOrAfterTime(
      const int64_t timestamp_ns, int64_t* timestamp_ns_of_value,
      ValueType* value) const;

  // Returns false if the timestamp is not between two values.
  bool interpolateAt(const int64_t timestamp_ns, ValueType* output) const;

  // Get all values between the two specified timestamps excluding the border
  // values.
  // Example: content: 2 3 4 5
  //          getValuesBetweenTimes(2, 5, ...) returns elements at 3, 4.
  template <typename ValueContainerType>
  void getValuesBetweenTimes(
      const int64_t timestamp_lower_ns, const int64_t timestamp_higher_ns,
      ValueContainerType* values) const;

  template <typename ValueContainerType>
  void getValuesFromExcludingToIncluding(
      const int64_t timestamp_lower_ns, const int64_t timestamp_higher_ns,
      ValueContainerType* values) const;

  bool operator==(const TemporalBuffer& other) const {
    return values_ == other.values_ &&
           buffer_length_nanoseconds_ == other.buffer_length_nanoseconds_;
  }

  bool operator!=(const TemporalBuffer& other) const {
    return !operator==(other);
  }

  typename BufferType::iterator begin() {
    return values_.begin();
  }

  typename BufferType::const_iterator begin() const {
    return values_.cbegin();
  }

  typename BufferType::iterator end() {
    return values_.end();
  }

  typename BufferType::const_iterator end() const {
    return values_.cend();
  }

  typename BufferType::reverse_iterator rbegin() {
    return values_.rbegin();
  }

  typename BufferType::const_reverse_iterator rbegin() const {
    return values_.crbegin();
  }

  typename BufferType::reverse_iterator rend() {
    return values_.rend();
  }

  typename BufferType::const_reverse_iterator rend() const {
    return values_.crend();
  }

 protected:
  BufferType& getValues() {
    return values_;
  }

  const BufferType& getValues() const {
    return values_;
  }

 private:
  // Remove items that are older than the buffer length.
  void removeOutdatedItems();

  bool getIteratorAtTimeOrEarlier(
      int64_t timestamp,
      typename BufferType::const_iterator* it_lower_bound) const;

  BufferType values_;
  int64_t buffer_length_nanoseconds_;
};

}  // namespace common

#include "maplab-common/temporal-buffer-inl.h"

#endif  // MAPLAB_COMMON_TEMPORAL_BUFFER_H_
