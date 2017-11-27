#ifndef MAPLAB_COMMON_TRIDIAGONAL_MATRIX_H_
#define MAPLAB_COMMON_TRIDIAGONAL_MATRIX_H_

#include <Eigen/Dense>

namespace common {

// This class is motivated by: 1) Memory-efficient tridiagonal matrix
// representation. 2) Because Eigen doesn't support tridigonal matrix solution.
// http://comments.gmane.org/gmane.comp.lib.eigen/1500
// Tridiagonal matrix solution is e.g. used to obtain a cubic spline.

// Current pre-compiled specializations are for double and float.
// More pre-compiles can be added at the end of the cc file.
template <typename ScalarType>
class TridiagonalMatrix {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> Vector;

  TridiagonalMatrix(
      const Vector& lower_diagonal, const Vector& middle_diagonal,
      const Vector& upper_diagonal);

  // Solves "(*this) * x = b" in linear time using the Thomas algorithm.
  void solve(const Vector& b, Vector* x) const;

 private:
  const Vector lower_diagonal_;
  const Vector middle_diagonal_;
  const Vector upper_diagonal_;
  const int size_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_TRIDIAGONAL_MATRIX_H_
