#ifndef MAPLAB_COMMON_VECTOR_WINDOW_OPERATIONS_H_
#define MAPLAB_COMMON_VECTOR_WINDOW_OPERATIONS_H_

#include <algorithm>
#include <functional>
#include <vector>

#include <glog/logging.h>

namespace common {

namespace window_vec_ops {

template <typename Type>
using WindowOperation = std::function<Type(
    typename std::vector<Type>::const_iterator /* begin */,
    typename std::vector<Type>::const_iterator /* end */,
    const Type /* invalid_value */)>;

// Applies a sliding window operation to a vector. Ignores vector entries that
// are equal to the invalid_value. The window size needs to be odd.
template <typename Type>
void computeWindowOperation(
    const std::vector<Type>& input_values, const unsigned int window_size,
    const WindowOperation<Type>& operation, const Type invalid_value,
    std::vector<Type>* result_values) {
  CHECK_NOTNULL(result_values)->clear();
  CHECK_EQ(window_size % 2u, 1u);
  if (input_values.empty()) {
    return;
  }

  const unsigned int num_values = input_values.size();
  const int max_idx = num_values - 1u;
  const int half_window = window_size / 2u;

  result_values->resize(num_values);

  for (int value_idx = 0; value_idx < static_cast<int>(input_values.size());
       ++value_idx) {
    const unsigned int window_begin = std::max(0, value_idx - half_window);
    const unsigned int window_end = std::min(max_idx, value_idx + half_window);
    CHECK_LE(window_begin, window_end);

    typename std::vector<Type>::const_iterator start = input_values.begin();
    typename std::vector<Type>::const_iterator end = input_values.begin();
    std::advance(start, window_begin);
    std::advance(end, window_end + 1);
    (*result_values)[value_idx] = operation(start, end, invalid_value);
  }
}

template <typename Type>
Type averageWindowOperation(
    typename std::vector<Type>::const_iterator values_start,
    typename std::vector<Type>::const_iterator values_end,
    const Type invalid_value) {
  Type sum = static_cast<Type>(0.0);
  unsigned int num_values = 0u;
  for (; values_start != values_end; ++values_start) {
    if (*values_start != invalid_value) {
      sum += *values_start;
      ++num_values;
    }
  }
  return (num_values != 0u) ? sum / num_values : invalid_value;
}

template <typename Type>
void computeRunningAverage(
    const std::vector<Type>& input_values, const unsigned int window_size,
    const Type invalid_value, std::vector<Type>* running_avg_values) {
  CHECK_NOTNULL(running_avg_values);
  computeWindowOperation<Type>(
      input_values, window_size, averageWindowOperation<Type>, invalid_value,
      running_avg_values);
}

inline bool logicalOrOperation(
    std::vector<bool>::const_iterator values_start,
    std::vector<bool>::const_iterator values_end,
    const bool /* invalid_value */) {
  for (; values_start != values_end; ++values_start) {
    if (*values_start) {
      return true;
    }
  }
  return false;
}

inline void dilatateBoolVector(
    const std::vector<bool>& input_values, const unsigned int window_size,
    std::vector<bool>* dilatated_vec) {
  computeWindowOperation<bool>(
      input_values, window_size, logicalOrOperation, false, dilatated_vec);
}

template <typename Type>
Type computeAverage(
    const std::vector<Type>& input_values, const Type invalid_value) {
  return averageWindowOperation(
      input_values.begin(), input_values.end(), invalid_value);
}

}  // namespace window_vec_ops

}  // namespace common

#endif  // MAPLAB_COMMON_VECTOR_WINDOW_OPERATIONS_H_
