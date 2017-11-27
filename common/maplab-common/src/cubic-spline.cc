#include "maplab-common/cubic-spline.h"

#include <glog/logging.h>

#include "maplab-common/tridiagonal-matrix.h"

// See http://mathworld.wolfram.com/CubicSpline.html for a derivation of the
// code below.
namespace common {

template <typename ScalarType, int Dimensions>
CubicSpline<ScalarType, Dimensions>::CubicSpline(const Vectors& nodes)
    : nodes_(nodes), size_(nodes.cols()), node_derivatives_(Dimensions, size_) {
  CHECK_GT(size_, 0);
  TransposedRowVector middle_row(TransposedRowVector::Constant(size_, 1, 4));
  middle_row(0) = 2;
  middle_row(size_ - 1) = 2;
  const TridiagonalMatrix<ScalarType> spline_matrix(
      TransposedRowVector::Constant(size_ - 1, 1, 1), middle_row,
      TransposedRowVector::Constant(size_ - 1, 1, 1));

  Vectors node_2_diffs(Dimensions, size_);
  node_2_diffs.middleCols(1, size_ - 2) =
      nodes_.rightCols(size_ - 2) - nodes_.leftCols(size_ - 2);
  node_2_diffs.col(0) = nodes_.col(1) - nodes_.col(0);
  node_2_diffs.col(size_ - 1) = nodes_.col(size_ - 1) - nodes_.col(size_ - 2);

  for (int d = 0; d < Dimensions; ++d) {
    TransposedRowVector derivative_row;
    spline_matrix.solve(node_2_diffs.row(d) * 3, &derivative_row);
    node_derivatives_.row(d) = derivative_row;
  }
}

template <typename ScalarType, int Dimensions>
typename CubicSpline<ScalarType, Dimensions>::Vector
CubicSpline<ScalarType, Dimensions>::getValueAt(const double t) const {
  double intpart_double;
  const double f1 = modf(t, &intpart_double);
  const int i = static_cast<int>(intpart_double);

  CHECK_GE(i, 0);

  if (f1 == 0.) {
    CHECK_LT(i, size_);
    return nodes_.col(i);
  } else {
    CHECK_LT(i, size_ - 1) << f1;
  }

  const double f2 = f1 * f1;
  const double f3 = f2 * f1;

  return nodes_.col(i) * (1 - 3 * f2 + 2 * f3) +
         nodes_.col(i + 1) * (3 * f2 - 2 * f3) +
         node_derivatives_.col(i) * (f1 - 2 * f2 + f3) +
         node_derivatives_.col(i + 1) * (f3 - f2);
}

template <typename ScalarType, int Dimensions>
void CubicSpline<ScalarType, Dimensions>::getValuesAtPeriod(
    const double period, Vectors* result) const {
  CHECK_GT(period, 0.);
  const int length = static_cast<int>(floor((size_ - 1) / period)) + 1;
  CHECK_NOTNULL(result)->resize(Eigen::NoChange, length);

  for (int i = 0; i < length; ++i) {
    result->col(i) = getValueAt(i * period);
  }
}

template <typename ScalarType, int Dimensions>
typename CubicSpline<ScalarType, Dimensions>::Vector
CubicSpline<ScalarType, Dimensions>::getDerivativeAt(const double t) const {
  double intpart_double;
  const double f1 = modf(t, &intpart_double);
  const int i = static_cast<int>(intpart_double);

  CHECK_GE(i, 0);

  if (f1 == 0.) {
    CHECK_LT(i, size_);
    return nodes_.col(i);
  } else {
    CHECK_LT(i, size_ - 1) << f1;
  }

  const double f2 = f1 * f1;

  return nodes_.col(i) * 6 * (f2 - f1) + nodes_.col(i + 1) * 6 * (f1 - f2) +
         node_derivatives_.col(i) * (1 - 4 * f1 + 3 * f2) +
         node_derivatives_.col(i + 1) * (3 * f2 - 2 * f1);
}

template <typename ScalarType, int Dimensions>
void CubicSpline<ScalarType, Dimensions>::getDerivativesAtPeriod(
    const double period, Vectors* result) const {
  CHECK_GT(period, 0.);
  const int length = static_cast<int>(floor((size_ - 1) / period)) + 1;
  CHECK_NOTNULL(result)->resize(Eigen::NoChange, length);

  for (int i = 0; i < length; ++i) {
    result->col(i) = getDerivativeAt(i * period);
  }
}

template class CubicSpline<double, 3>;
template class CubicSpline<float, 3>;

}  // namespace common
