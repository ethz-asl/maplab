#ifndef MAPLAB_COMMON_TEMPORAL_BUFFER_INL_H_
#define MAPLAB_COMMON_TEMPORAL_BUFFER_INL_H_

#include <limits>
#include <utility>

#include <glog/logging.h>

#include "maplab-common/interpolation-helpers.h"

namespace common {

template <typename ValueType, typename AllocatorType>
TemporalBuffer<ValueType, AllocatorType>::TemporalBuffer()
    : buffer_length_nanoseconds_(-1) {}

template <typename ValueType, typename AllocatorType>
TemporalBuffer<ValueType, AllocatorType>::TemporalBuffer(
    int64_t buffer_length_nanoseconds)
    : buffer_length_nanoseconds_(buffer_length_nanoseconds) {}

template <typename ValueType, typename AllocatorType>
TemporalBuffer<ValueType, AllocatorType>::TemporalBuffer(
    const TemporalBuffer<ValueType, AllocatorType>& other) {
  values_ = other.values_;
  buffer_length_nanoseconds_ = other.buffer_length_nanoseconds_;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::addValue(
    const int64_t timestamp, const ValueType& value) {
  std::pair<typename BufferType::iterator, bool> it_value_inserted_pair =
      values_.emplace(timestamp, value);

  // Replace value at existing timestamp.
  if (!it_value_inserted_pair.second) {
    CHECK(it_value_inserted_pair.first != values_.end());
    it_value_inserted_pair.first->second = value;
  }

  removeOutdatedItems();
  return it_value_inserted_pair.second;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::deleteValueAtTime(
    int64_t timestamp_ns) {
  return values_.erase(timestamp_ns) > 0u;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getOldestTime(
    int64_t* timestamp_nanoseconds) const {
  CHECK_NOTNULL(timestamp_nanoseconds);
  if (empty()) {
    return false;
  }
  *timestamp_nanoseconds = values_.begin()->first;
  return true;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getOldestValue(
    ValueType* value) const {
  CHECK_NOTNULL(value);
  if (empty()) {
    return false;
  }
  *value = values_.begin()->second;
  return true;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getNewestTime(
    int64_t* timestamp_nanoseconds) const {
  CHECK_NOTNULL(timestamp_nanoseconds);
  if (empty()) {
    return false;
  }
  *timestamp_nanoseconds = values_.rbegin()->first;
  return true;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getNewestValue(
    ValueType* value) const {
  CHECK_NOTNULL(value);
  if (empty()) {
    return false;
  }
  *value = values_.rbegin()->second;
  return true;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getValueAtTime(
    int64_t timestamp, ValueType* value) const {
  CHECK_NOTNULL(value);
  typename BufferType::const_iterator it = values_.find(timestamp);
  if (it != values_.end()) {
    *value = it->second;
    return true;
  }
  return false;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getNearestValueToTime(
    int64_t timestamp, ValueType* value) const {
  CHECK_NOTNULL(value);
  return getNearestValueToTime(
      timestamp, std::numeric_limits<int64_t>::max(), value);
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getValueAtOrBeforeTime(
    const int64_t timestamp, int64_t* timestamp_of_value,
    ValueType* value) const {
  CHECK_NOTNULL(timestamp_of_value);
  CHECK_NOTNULL(value);

  typename BufferType::const_iterator it_lower_bound;
  const bool has_exact_match =
      getIteratorAtTimeOrEarlier(timestamp, &it_lower_bound);

  if (!has_exact_match) {
    if (it_lower_bound == values_.begin()) {
      return false;
    }
    --it_lower_bound;
  }
  *timestamp_of_value = it_lower_bound->first;
  *value = it_lower_bound->second;

  CHECK_LE(*timestamp_of_value, timestamp);
  return true;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getValueAtOrAfterTime(
    const int64_t timestamp, int64_t* timestamp_of_value,
    ValueType* value) const {
  CHECK_NOTNULL(timestamp_of_value);
  CHECK_NOTNULL(value);

  typename BufferType::const_iterator it_lower_bound;
  bool has_exact_match = getIteratorAtTimeOrEarlier(timestamp, &it_lower_bound);

  if (!has_exact_match) {
    if (it_lower_bound == values_.end()) {
      return false;
    }
  }
  *timestamp_of_value = it_lower_bound->first;
  *value = it_lower_bound->second;

  CHECK_GE(*timestamp_of_value, timestamp);
  return true;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getIteratorAtTimeOrEarlier(
    const int64_t timestamp,
    typename BufferType::const_iterator* it_lower_bound) const {
  CHECK_NOTNULL(it_lower_bound);

  // Return false if no values in buffer.
  if (empty()) {
    return false;
  }

  // Returns first element with a time that compares not less to timestamp.
  *it_lower_bound = values_.lower_bound(timestamp);

  if (*it_lower_bound != values_.end() &&
      (*it_lower_bound)->first == timestamp) {
    // It's an exact match.
    return true;
  }
  return false;
}

// Return false if the pose is not between two values.
template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::interpolateAt(
    const int64_t timestamp_ns, ValueType* output) const {
  CHECK_NOTNULL(output);

  int64_t timestamp_before;
  ValueType value_before;
  if (!getValueAtOrBeforeTime(timestamp_ns, &timestamp_before, &value_before)) {
    return false;
  }

  int64_t timestamp_after;
  ValueType value_after;
  if (!getValueAtOrAfterTime(timestamp_ns, &timestamp_after, &value_after)) {
    return false;
  }

  if (timestamp_after == timestamp_before) {
    CHECK_EQ(timestamp_ns, timestamp_after);
    *output = value_before;
    return true;
  }

  CHECK_LT(timestamp_before, timestamp_ns);
  CHECK_GT(timestamp_after, timestamp_ns);

  LinearInterpolationFunctor<int64_t, ValueType>()(
      timestamp_before, value_before, timestamp_after, value_after,
      timestamp_ns, output);
  return true;
}

template <typename ValueType, typename AllocatorType>
template <typename ValueContainerType>
void TemporalBuffer<ValueType, AllocatorType>::getValuesBetweenTimes(
    const int64_t timestamp_lower_ns, const int64_t timestamp_higher_ns,
    ValueContainerType* values) const {
  CHECK_NOTNULL(values)->clear();
  CHECK_GT(timestamp_higher_ns, timestamp_lower_ns);

  typename BufferType::const_iterator it =
      values_.lower_bound(timestamp_lower_ns);
  for (; it != values_.end() && it->first < timestamp_higher_ns; ++it) {
    // lower_bound includes the border so we need to skip them when there are
    // perfect matches.
    if (it->first == timestamp_lower_ns) {
      continue;
    }
    values->emplace_back(it->second);
  }
}

template <typename ValueType, typename AllocatorType>
template <typename ValueContainerType>
void TemporalBuffer<ValueType, AllocatorType>::
    getValuesFromExcludingToIncluding(
        const int64_t timestamp_lower_ns, const int64_t timestamp_higher_ns,
        ValueContainerType* values) const {
  CHECK_NOTNULL(values)->clear();
  CHECK_GT(timestamp_higher_ns, timestamp_lower_ns);
  typename BufferType::const_iterator it =
      values_.lower_bound(timestamp_lower_ns);
  for (; it != values_.end() && it->first <= timestamp_higher_ns; ++it) {
    CHECK(it != values_.end());

    // lower_bound includes the border so we need to skip them when there are
    // perfect matches.
    if (it->first == timestamp_lower_ns) {
      continue;
    }
    values->emplace_back(it->second);
  }
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getNearestValueToTime(
    int64_t timestamp, int64_t maximum_delta_ns, ValueType* value,
    int64_t* timestamp_at_value_ns) const {
  CHECK_NOTNULL(timestamp_at_value_ns);
  CHECK_NOTNULL(value);

  if (empty()) {
    return false;
  }

  const typename BufferType::const_iterator it_lower =
      values_.lower_bound(timestamp);

  // Verify if we got an exact match.
  if (it_lower != values_.end() && it_lower->first == timestamp) {
    *value = it_lower->second;
    *timestamp_at_value_ns = it_lower->first;
    return true;
  }

  // No exact match found so we need to examine lower and upper
  // bound on the timestamp.
  const typename BufferType::const_iterator it_upper =
      values_.upper_bound(timestamp);

  // If the lower bound points out of the array, we have to return the last
  // element.
  if (it_lower == values_.end()) {
    typename BufferType::const_iterator it_last = std::prev(values_.end());
    int64_t delta_ns = std::abs(it_last->first - timestamp);
    if (delta_ns <= maximum_delta_ns) {
      *value = it_last->second;
      *timestamp_at_value_ns = it_lower->first;
      return true;
    } else {
      return false;
    }
  }

  // If the lower bound points to begin() and no exact match was found, we
  // have to return the first element.
  if (it_lower == values_.begin()) {
    typename BufferType::const_iterator it_first = values_.begin();
    int64_t delta_ns = std::abs(it_first->first - timestamp);
    if (delta_ns <= maximum_delta_ns) {
      *value = it_first->second;
      *timestamp_at_value_ns = it_lower->first;
      return true;
    } else {
      return false;
    }
  }

  // Both iterators are within range so need to find out which of them is
  // closer to the timestamp.
  typename BufferType::const_iterator it_before = std::prev(it_lower);
  int64_t delta_before_ns = std::abs(it_before->first - timestamp);
  int64_t delta_after_ns = std::abs(it_upper->first - timestamp);
  if (delta_before_ns < delta_after_ns) {
    if (delta_before_ns <= maximum_delta_ns) {
      *value = it_before->second;
      *timestamp_at_value_ns = it_lower->first;
      return true;
    }
  } else {
    if (delta_after_ns <= maximum_delta_ns) {
      *value = it_upper->second;
      *timestamp_at_value_ns = it_lower->first;
      return true;
    }
  }

  return false;
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getNearestValueToTime(
    int64_t timestamp, int64_t maximum_delta_ns, ValueType* value) const {
  int64_t timestamp_at_value_ns;
  return getNearestValueToTime(
      timestamp, maximum_delta_ns, value, &timestamp_at_value_ns);
}

template <typename ValueType, typename AllocatorType>
void TemporalBuffer<ValueType, AllocatorType>::removeOutdatedItems() {
  if (empty() || buffer_length_nanoseconds_ <= 0) {
    return;
  }

  const int64_t newest_timestamp_ns = values_.rbegin()->first;
  const int64_t buffer_threshold_ns =
      newest_timestamp_ns - buffer_length_nanoseconds_;

  if (values_.begin()->first < buffer_threshold_ns) {
    typename BufferType::const_iterator it =
        values_.lower_bound(buffer_threshold_ns);

    CHECK(it != values_.end());
    CHECK(it != values_.begin());
    values_.erase(values_.begin(), it);
  }
}

}  // namespace common
#endif  // MAPLAB_COMMON_TEMPORAL_BUFFER_INL_H_
