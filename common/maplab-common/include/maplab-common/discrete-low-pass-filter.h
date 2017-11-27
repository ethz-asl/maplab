#ifndef MAPLAB_COMMON_DISCRETE_LOW_PASS_FILTER_H_
#define MAPLAB_COMMON_DISCRETE_LOW_PASS_FILTER_H_

#include <deque>

#include <Eigen/Core>
#include <maplab-common/macros.h>

namespace common {

template <typename MeasurementType>
class DiscreteLowPassFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(DiscreteLowPassFilter);
  DiscreteLowPassFilter() = delete;
  explicit DiscreteLowPassFilter(const size_t window_size);
  virtual ~DiscreteLowPassFilter() = default;

  MeasurementType addMeasurement(const MeasurementType z);

  MeasurementType getEstimate() const;

 private:
  void removeLastMeasurement();
  size_t window_size_;
  std::deque<MeasurementType> measurements_;
  MeasurementType running_sum_;
};

}  // namespace common

#include "./internal/discrete-low-pass-filter-inl.h"

#endif  // MAPLAB_COMMON_DISCRETE_LOW_PASS_FILTER_H_
