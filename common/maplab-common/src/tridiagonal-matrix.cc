#include "maplab-common/tridiagonal-matrix.h"

#include <glog/logging.h>

namespace common {

template <typename ScalarType>
TridiagonalMatrix<ScalarType>::TridiagonalMatrix(
    const Vector& lower_diagonal, const Vector& middle_diagonal,
    const Vector& upper_diagonal)
    : lower_diagonal_(lower_diagonal),
      middle_diagonal_(middle_diagonal),
      upper_diagonal_(upper_diagonal),
      size_(middle_diagonal.size()) {
  CHECK_GT(size_, 0);
  CHECK_EQ(lower_diagonal.size(), size_ - 1);
  CHECK_EQ(upper_diagonal.size(), size_ - 1);
}

// From https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
template <typename ScalarType>
void TridiagonalMatrix<ScalarType>::solve(const Vector& b, Vector* x) const {
  CHECK_EQ(b.size(), size_);
  CHECK_NOTNULL(x)->resize(size_, Eigen::NoChange);

  Vector A_forward_sweep(size_ - 1);
  CHECK_NE(middle_diagonal_(0), 0.);
  A_forward_sweep(0) = upper_diagonal_(0) / middle_diagonal_(0);
  for (int i = 1; i < size_ - 1; ++i) {
    A_forward_sweep(i) =
        upper_diagonal_(i) /
        (middle_diagonal_(i) - lower_diagonal_(i - 1) * A_forward_sweep(i - 1));
  }

  Vector b_forward_sweep(size_);
  b_forward_sweep(0) = b(0) / middle_diagonal_(0);
  for (int i = 1; i < size_; ++i) {
    b_forward_sweep(i) =
        (b(i) - lower_diagonal_(i - 1) * b_forward_sweep(i - 1)) /
        (middle_diagonal_(i) - lower_diagonal_(i - 1) * A_forward_sweep(i - 1));
  }

  (*x)(size_ - 1) = b_forward_sweep(size_ - 1);
  for (int i = size_ - 2; i >= 0; --i) {
    (*x)(i) = b_forward_sweep(i) - A_forward_sweep(i) * (*x)(i + 1);
  }
}

template class TridiagonalMatrix<double>;
template class TridiagonalMatrix<float>;

}  // namespace common
