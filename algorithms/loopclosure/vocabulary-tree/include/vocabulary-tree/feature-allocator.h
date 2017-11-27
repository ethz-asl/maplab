// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_FEATURE_ALLOCATOR_H_
#define VOCABULARY_TREE_FEATURE_ALLOCATOR_H_

#include <memory>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace loop_closure {

/**
 * \brief Meta-function to get the default allocator for a particular feature
 * type.
 *
 * Defaults to \c std::allocator<Feature>.
 */
template <class Feature>
struct DefaultAllocator {
  typedef std::allocator<Feature> type;
};

// Specialization to use aligned allocator for Eigen::Matrix types.
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
struct DefaultAllocator<
    Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> > {
  typedef Eigen::aligned_allocator<
      Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> >
      type;
};
}  //  namespace loop_closure
#endif  // VOCABULARY_TREE_FEATURE_ALLOCATOR_H_
