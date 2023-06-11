#ifndef ASLAM_COMMON_EIGEN_HASH_H_
#define ASLAM_COMMON_EIGEN_HASH_H_

#include <functional>

#include <Eigen/Core>

namespace std {

template <typename Scalar, int Rows, int Cols>
struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
  // https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
  size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < static_cast<size_t>(matrix.size()); ++i) {
      Scalar elem = *(matrix.data() + i);
      seed ^=
          std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

}  // namespace std

#endif  // ASLAM_COMMON_EIGEN_HASH_H_
