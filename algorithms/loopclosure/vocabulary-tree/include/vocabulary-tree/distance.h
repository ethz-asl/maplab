// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_DISTANCE_H_
#define VOCABULARY_TREE_DISTANCE_H_

#include <stdint.h>

#include <Eigen/Core>
#include <vocabulary-tree/hamming.h>

namespace loop_closure {
namespace distance {

/**
 * \brief Meta-function returning a type that can be used to accumulate many
 * values of T.
 *
 * By default, the accumulator type is the same as \c T. Specializations for
 * the basic types are:
 * \li \c uint8_t -> \c uint32_t
 * \li \c uint16_t -> \c uint32_t
 * \li \c int8_t -> \c int32_t
 * \li \c int16_t -> \c int32_t
 * \li \c float -> \c double
 */
template <typename T>
struct Accumulator {
  typedef T type;
};

// \cond internal
template <>
struct Accumulator<uint8_t> {
  typedef uint32_t type;
};
template <>
struct Accumulator<uint16_t> {
  typedef uint32_t type;
};
template <>
struct Accumulator<int8_t> {
  typedef int32_t type;
};
template <>
struct Accumulator<int16_t> {
  typedef int32_t type;
};
// \endcond

/**
 * \brief Default implementation of L2 distance metric.
 *
 * Works with std::vector, boost::array, or more generally any container that
 * has a \c value_type typedef, \c size() and array-indexed element access.
 */
template <class Feature>
struct L2 {
  typedef typename Feature::value_type value_type;
  typedef typename Accumulator<value_type>::type result_type;

  result_type operator()(const Feature& a, const Feature& b) const {
    result_type result = result_type();
    for (size_t i = 0; i < a.size(); ++i) {
      result_type diff = a[i] - b[i];
      result += diff * diff;
    }
    return result;
  }
};
// @todo Version for raw data pointers that knows the size of the feature
// @todo Specialization for cv::Vec. Doesn't have size() so default won't work.

// Specialization for Eigen::Matrix types.
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
struct L2<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> > {
  typedef Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>
      feature_type;
  typedef Scalar value_type;
  typedef typename Accumulator<Scalar>::type result_type;

  result_type operator()(const feature_type& a, const feature_type& b) const {
    return (a - b).squaredNorm();
  }
};

template <class Feature>
struct Hamming {
  typedef typename Feature::value_type value_type;
  typedef unsigned int result_type;
  result_type operator()(const Feature& a, const Feature& b) const {
    CHECK_EQ(a.size(), b.size()) << " a: " << a.size() << " b: " << b.size();
    if (a.size() == 64) {
      return HammingDistance512(a.data(), b.data());
    } else if (a.size() == 48) {
      unsigned int sum1 = HammingDistance256(a.data(), b.data());
      unsigned int sum2 = HammingDistance128(a.data() + 32, b.data() + 32);
      return sum1 + sum2;
    } else if (a.size() == 32) {
      return HammingDistance256(a.data(), b.data());
    } else {
      CHECK(a.size() == 64 || a.size() == 48 || a.size() == 32)
          << "Unsupported descriptor length: " << a.size();
      return 0;
    }
  }
};

}  //  namespace distance
}  //  namespace loop_closure

#endif  // VOCABULARY_TREE_DISTANCE_H_
