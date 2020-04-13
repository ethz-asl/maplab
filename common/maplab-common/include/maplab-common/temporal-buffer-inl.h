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
template <typename ValueContainerType>
void TemporalBuffer<ValueType, AllocatorType>::
    getValuesFromIncludingToIncluding(
        const int64_t timestamp_lower_ns, const int64_t timestamp_higher_ns,
        ValueContainerType* values) const {
  CHECK_NOTNULL(values)->clear();
  CHECK_GT(timestamp_higher_ns, timestamp_lower_ns);
  typename BufferType::const_iterator it =
      values_.lower_bound(timestamp_lower_ns);
  for (; it != values_.end() && it->first <= timestamp_higher_ns; ++it) {
    CHECK(it != values_.end());
    values->emplace_back(it->second);
  }
}

template <typename ValueType, typename AllocatorType>
bool TemporalBuffer<ValueType, AllocatorType>::getNearestValueToTime(
    int64_t timestamp_ns, int64_t maximum_delta_ns, ValueType* value,
    int64_t* timestamp_at_value_ns) const {
  CHECK_NOTNULL(timestamp_at_value_ns);
  CHECK_NOTNULL(value);

  if (empty()) {
    return false;
  }

  const typename BufferType::const_iterator it_to_newer_or_equal =
      values_.lower_bound(timestamp_ns);

  // Check if we happened to get an exact match.
  if (it_to_newer_or_equal != values_.end() &&
      it_to_newer_or_equal->first == timestamp_ns) {
    *value = it_to_newer_or_equal->second;
    *timestamp_at_value_ns = it_to_newer_or_equal->first;
    return true;
  }

  // If the lower bound points to end, we have to check the last element,
  // since the timestamp is newer than the buffer.
  if (it_to_newer_or_equal == values_.end()) {
    typename BufferType::const_reverse_iterator it_last = values_.crbegin();
    int64_t delta_ns = std::abs(it_last->first - timestamp_ns);
    if (delta_ns <= maximum_delta_ns) {
      *value = it_last->second;
      *timestamp_at_value_ns = it_last->first;
      // The timestamp is newer than the buffer, but within the matching
      // tolerance.
      return true;
    } else {
      // The timestamp is newer than the buffer by more than the tolerance.
      return false;
    }
  }

  // If the upper bound points to begin(), then the timestamp is older than the
  // buffer and we need to check the first value in the buffer.
  if (it_to_newer_or_equal == values_.begin()) {
    typename BufferType::const_iterator it_first = values_.cbegin();
    int64_t delta_ns = std::abs(it_first->first - timestamp_ns);
    if (delta_ns <= maximum_delta_ns) {
      *value = it_first->second;
      *timestamp_at_value_ns = it_first->first;
      // The timestamp is older than the buffer, but within the matching
      // tolerance.
      return true;
    } else {
      // The timestamp is older than the buffer, but by more than the tolerance.
      return false;
    }
  }

  // The timestamp is within the buffer, now we need to figure out which one of
  // the two adjacent values is closer. Since we already checked that the lower
  // bound is not the first element, we can just get the older value by moving
  // the lower_bound iterator backwards.
  const typename BufferType::const_iterator it_to_older =
      std::prev(it_to_newer_or_equal);
  const int64_t delta_to_older_element_ns =
      std::abs(it_to_older->first - timestamp_ns);
  const int64_t delta_to_newer_element_ns =
      std::abs(it_to_newer_or_equal->first - timestamp_ns);
  if (delta_to_newer_element_ns < delta_to_older_element_ns) {
    if (delta_to_newer_element_ns <= maximum_delta_ns) {
      *value = it_to_newer_or_equal->second;
      *timestamp_at_value_ns = it_to_newer_or_equal->first;
      return true;
    }
  } else {
    if (delta_to_older_element_ns <= maximum_delta_ns) {
      *value = it_to_older->second;
      *timestamp_at_value_ns = it_to_older->first;
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
size_t TemporalBuffer<ValueType, AllocatorType>::removeItemsBefore(
    const int64_t timestamp_ns) {
  if (empty()) {
    return 0u;
  }

  if (values_.begin()->first < timestamp_ns) {
    typename BufferType::const_iterator it = values_.lower_bound(timestamp_ns);

    CHECK(it != values_.begin());

    const size_t size_before = values_.size();
    values_.erase(values_.begin(), it);
    const size_t num_elements_erased = size_before - values_.size();
    return num_elements_erased;
  }
  return 0u;
}

template <typename ValueType, typename AllocatorType>
template <typename ValueContainerType>
size_t TemporalBuffer<ValueType, AllocatorType>::extractItemsBeforeIncluding(
    const int64_t timestamp_ns, ValueContainerType* removed_values) {
  CHECK_NOTNULL(removed_values)->clear();

  if (empty()) {
    return 0u;
  }

  // If timestamp is older than oldest timestamp, then there is nothing to
  // return.
  if (timestamp_ns < values_.begin()->first) {
    return 0u;
  }
  // Otherwise there is at least one element that is equal or older.

  // Get first iterator that is greater than the timestamp, i.e. everything
  // between begin() and this iterator is what we want.
  typename BufferType::iterator first_newer_timestamp_it =
		  values_.upper_bound(timestamp_ns);

  // Copy values to be removed.
  std::transform(
      values_.begin(), first_newer_timestamp_it,
      std::back_inserter(*removed_values),
      [](const typename BufferType::value_type& stamped_value) {
	  return stamped_value.second;
  });

  // Erase values.
  values_.erase(values_.begin(), first_newer_timestamp_it);

  return removed_values->size();
}

template <typename ValueType, typename AllocatorType>
template <typename ValueContainerType>
size_t TemporalBuffer<ValueType, AllocatorType>::
    extractItemsBeforeIncludingKeepMostRecent(
        const int64_t timestamp_ns, ValueContainerType* returned_values) {
  CHECK_NOTNULL(returned_values)->clear();

  if (empty()) {
    return 0u;
  }

  // If timestamp is older than oldest timestamp, then there is nothing to
  // return.
  if (timestamp_ns < values_.begin()->first) {
    return 0u;
  }
  // Otherwise there is at least one element that is equal or older.

  // Get first iterator that is greater than the timestamp, i.e. everything
  // between begin() and this iterator is what we want.
  auto first_newer_timestamp_it = values_.upper_bound(timestamp_ns);

  // Copy values earlier or equal to the timestamp.
  std::transform(
      values_.begin(), first_newer_timestamp_it,
      std::back_inserter(*returned_values),
      [](const typename BufferType::value_type& stamped_value) {
	  return stamped_value.second;
  });

  // Remove values that were returned, except for the newest one.
  const size_t size_before = values_.size();
  values_.erase(values_.begin(), std::prev(first_newer_timestamp_it));
  const size_t num_elements_erased = size_before - values_.size();
  CHECK_EQ(num_elements_erased, returned_values->size() - 1);
  return num_elements_erased;
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
