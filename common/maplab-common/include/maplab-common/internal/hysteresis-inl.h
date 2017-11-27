#ifndef INTERNAL_HYSTERESIS_INL_H_
#define INTERNAL_HYSTERESIS_INL_H_

namespace common {

template <typename MeasurementType>
Hysteresis<MeasurementType>::Hysteresis(
    const MeasurementType& threshold, const size_t delay)
    : threshold_(threshold), delay_(delay) {}

template <typename MeasurementType>
bool Hysteresis<MeasurementType>::addMeasurement(
    const MeasurementType& measurement) {
  if (measurements_.size() >= delay_) {
    measurements_.pop_front();
  }
  measurements_.emplace_back(measurement);
  for (const MeasurementType measurement : measurements_) {
    if (measurement < threshold_) {
      return false;
    }
  }
  return true;
}

}  // namespace common

#endif  // INTERNAL_HYSTERESIS_INL_H_
