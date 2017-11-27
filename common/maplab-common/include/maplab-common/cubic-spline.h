#ifndef MAPLAB_COMMON_CUBIC_SPLINE_H_
#define MAPLAB_COMMON_CUBIC_SPLINE_H_

#include <Eigen/Dense>

namespace common {

// Current pre-compiled specializations are for double and float and 3
// dimensions. More pre-compiles can be added at the end of the cc file.
template <typename ScalarType, int Dimensions>
class CubicSpline {
 public:
  typedef Eigen::Matrix<ScalarType, Dimensions, 1> Vector;
  typedef Eigen::Matrix<ScalarType, Dimensions, Eigen::Dynamic> Vectors;
  typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> TransposedRowVector;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CubicSpline(const Vectors& nodes);

  // Integer t's correspond to the node positions.
  Vector getValueAt(const double t) const;
  void getValuesAtPeriod(const double period, Vectors* result) const;

  Vector getDerivativeAt(const double t) const;
  void getDerivativesAtPeriod(const double period, Vectors* result) const;

 private:
  const Vectors nodes_;
  const int size_;
  Vectors node_derivatives_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_CUBIC_SPLINE_H_
