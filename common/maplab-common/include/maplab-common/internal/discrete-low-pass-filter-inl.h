#ifndef INTERNAL_DISCRETE_LOW_PASS_FILTER_INL_H_
#define INTERNAL_DISCRETE_LOW_PASS_FILTER_INL_H_

#include <glog/logging.h>

namespace common {

template <typename MeasurementType>
DiscreteLowPassFilter<MeasurementType>::DiscreteLowPassFilter(
    const size_t window_size)
    : window_size_(window_size), running_sum_(0.0) {
  CHECK_GT(window_size_, 0u);
}

template <typename MeasurementType>
MeasurementType DiscreteLowPassFilter<MeasurementType>::addMeasurement(
    const MeasurementType z) {
  if (measurements_.size() >= window_size_) {
    removeLastMeasurement();
  }
  CHECK_LT(measurements_.size(), window_size_);
  running_sum_ += z;
  measurements_.emplace_back(z);
  CHECK_LE(measurements_.size(), window_size_);
  CHECK_GT(measurements_.size(), 0u);
  return running_sum_ / static_cast<double>(measurements_.size());
}

template <typename MeasurementType>
MeasurementType DiscreteLowPassFilter<MeasurementType>::getEstimate() const {
  if (measurements_.empty()) {
    return 0.0;
  }
  return running_sum_ / static_cast<double>(measurements_.size());
}

template <typename MeasurementType>
void DiscreteLowPassFilter<MeasurementType>::removeLastMeasurement() {
  CHECK(!measurements_.empty());
  running_sum_ -= measurements_.front();
  measurements_.pop_front();
}

}  // namespace common

#endif  // INTERNAL_DISCRETE_LOW_PASS_FILTER_INL_H_
