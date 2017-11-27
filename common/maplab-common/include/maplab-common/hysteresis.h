#ifndef MAPLAB_COMMON_HYSTERESIS_H_
#define MAPLAB_COMMON_HYSTERESIS_H_

#include <deque>

#include <Eigen/Core>
#include <maplab-common/macros.h>

namespace common {

template <typename MeasurementType>
class Hysteresis {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Hysteresis);
  Hysteresis() = delete;
  Hysteresis(const MeasurementType& threshold, const size_t delay);
  virtual ~Hysteresis() = default;

  bool addMeasurement(const MeasurementType& measurement);

 private:
  double threshold_;
  size_t delay_;
  std::deque<MeasurementType> measurements_;
};

}  // namespace common

#include "internal/hysteresis-inl.h"

#endif  // MAPLAB_COMMON_HYSTERESIS_H_
