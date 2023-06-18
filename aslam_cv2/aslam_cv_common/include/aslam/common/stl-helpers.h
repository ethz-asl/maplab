#ifndef ASLAM_STL_HELPERS_H_
#define ASLAM_STL_HELPERS_H_

#include <algorithm>
#include <chrono>
#include <random>
#include <unordered_set>
#include <vector>

#include <aslam/common/memory.h>
#include <Eigen/Core>
#include <glog/logging.h>

namespace aslam {
namespace common {

// Returns the total number of elements in a nested list, that is,
// num_outer_list_elements * num_inner_list_elements.
template<class ElementType, class Allocator, class NestedAllocator>
size_t countNumberOfElementsInNestedList(
    const std::vector<std::vector<ElementType, Allocator>, NestedAllocator>& nested_list) {
  size_t num_elements = 0u;
  for (const std::vector<ElementType, Allocator>& list : nested_list) {
    num_elements += list.size();
  }
  return num_elements;
}

template<typename RandAccessIter>
double median(RandAccessIter begin, RandAccessIter end) {
  CHECK(begin != end) << "No data provided to calculate the median.";
  size_t size = end - begin;
  size_t middle_idx = size / 2;
  RandAccessIter target_high = begin + middle_idx;
  std::nth_element(begin, target_high, end);

  // Odd number of elements.
  if (size % 2 != 0) {
    return *target_high;
  }
  // Even number of elements.
  double target_high_value = *target_high;
  RandAccessIter target_low = target_high - 1;
  std::nth_element(begin, target_low, end);
  return (target_high_value + *target_low) / 2.0;
}


template<typename RandAccessIter>
double mean(RandAccessIter begin, RandAccessIter end) {
  CHECK(begin != end) << "No data provided to calculate the mean.";
  const size_t n = end - begin;
  double mu = 0.0;
  RandAccessIter element_i = begin;
  while (element_i != end) {
    mu += *element_i;
    ++element_i;
  }
  return mu / n;
}


template<typename RandAccessIter>
double stddev(RandAccessIter begin, RandAccessIter end) {
  CHECK(begin != end) << "No data provided to calculate the standard deviation.";
  const size_t n = end - begin;
  RandAccessIter element_i = begin;
  double mu = aslam::common::mean(begin, end);

  double sum = 0.0;
  element_i = begin;
  while (element_i != end) {
    sum += (*element_i - mu) * (*element_i - mu);
    ++element_i;
  }

  CHECK_GE(sum, 0.0);
  return sqrt(sum / n);
}

template<typename ElementType, typename Allocator>
void drawNRandomElements(const size_t n, const std::vector<ElementType, Allocator>& input,
                         std::vector<ElementType, Allocator>* output,
                         const bool use_fixed_seed) {
  CHECK_NE(&input, output);
  CHECK_NOTNULL(output)->clear();
  CHECK_GT(n, 0u);
  const size_t num_input_elements = input.size();
  if (num_input_elements <= n) {
    *output = input;
    return;
  }

  // Draw random indices.
  const unsigned int seed =
      use_fixed_seed ? 0u : std::random_device{}();

  std::default_random_engine generator(seed);
  std::uniform_int_distribution<size_t> distribution(0u, num_input_elements - 1u);

  std::unordered_set<size_t> random_indices;
  while (random_indices.size() < n) {
    random_indices.insert(distribution(generator));
  }

  // Copy to output.
  output->reserve(n);
  for (const size_t idx : random_indices) {
    CHECK_LT(idx, num_input_elements);
    output->emplace_back(input[idx]);
  }
}

template<typename ElementType, typename Allocator>
void drawNRandomElements(const size_t n, const std::vector<ElementType, Allocator>& input,
                         std::vector<ElementType, Allocator>* output) {
  drawNRandomElements(n, input, output, false);
}

// Remove all elements except the N greatest elements. An optional action can be provided
// that is executed on all removed elements.
template<typename ElementType> struct NullAction { void operator()(const ElementType&) const {} };
template<typename ElementType, typename Allocator, typename CompareFunctor,
         typename RemoveActionFunctor = NullAction<ElementType>>
size_t keepOnlyNSortedElements(size_t max_elements_to_keep,
    const CompareFunctor& sort_compare_functor,
    std::vector<ElementType, Allocator>* container,
    const RemoveActionFunctor& action_on_removed_elements = NullAction<ElementType>()) {
  CHECK_NOTNULL(container);

  // Special case for max_elements_to_keep == 0u: only run the action on all elements.
  if (max_elements_to_keep == 0u) {
    for (const ElementType& element : *container) {
      action_on_removed_elements(element);
    }
    container->clear();
    return 0u;
  }

  // Early exit if the container has less elements than the number to keep.
  const size_t num_elements = container->size();
  if (num_elements <= max_elements_to_keep) {
    return num_elements;
  }

  // Sort up to N greatest elements.
  std::partial_sort(container->begin(), container->begin() + max_elements_to_keep,
                    container->end(), sort_compare_functor);
  CHECK_GE(container->size(), max_elements_to_keep);

  // Run the optional action on removed elements.
  typename std::vector<ElementType, Allocator>::const_iterator it =
      container->begin() + max_elements_to_keep;
  for(; it != container->end(); ++it) {
    action_on_removed_elements(*it);
  }

  // Remove the elements.
  container->erase(container->begin() + max_elements_to_keep, container->end());
  CHECK_LE(container->size(), max_elements_to_keep);
  return container->size();
}

template <int VectorDim>
inline void convertEigenToStlVector(
    const Eigen::template Matrix<double, VectorDim, Eigen::Dynamic>& input,
    Aligned<std::vector, Eigen::template Matrix<double, VectorDim, 1>>*
        output) {
  CHECK_NOTNULL(output);
  size_t num_cols = input.cols();
  output->clear();
  output->reserve(num_cols);

  auto inserter = std::inserter(*output, output->end());
  for (size_t idx = 0; idx < num_cols; ++idx) {
    *inserter++ = input.col(idx);
  }
}

// Solution from:
// http://stackoverflow.com/questions/7571937/how-to-delete-items-from-a-stdvector-given-a-list-of-indices
template<typename ElementType, typename Allocator>
inline std::vector<ElementType, Allocator> eraseIndicesFromVector(
    const std::vector<ElementType, Allocator>& data,
    const std::vector<size_t>& indices_to_delete) {
  if (indices_to_delete.empty()) {
    return data;
  }
  std::vector<size_t> mutable_indices_to_delete = indices_to_delete;
  std::sort(mutable_indices_to_delete.begin(), mutable_indices_to_delete.end());
  CHECK_LT(mutable_indices_to_delete.back(), data.size());

  std::vector<ElementType, Allocator> reduced_vector;
  CHECK_GE(data.size(), mutable_indices_to_delete.size());
  reduced_vector.reserve(data.size() - mutable_indices_to_delete.size());

  // Copy blocks from the input vector to the output vector.
  typename std::vector<ElementType, Allocator>::const_iterator it_block_begin = data.begin();

  for (typename std::vector<size_t>::const_iterator it = mutable_indices_to_delete.begin();
      it != mutable_indices_to_delete.end(); ++it) {
    typename std::vector<ElementType, Allocator>::const_iterator it_block_end = data.begin() + *it;
    if (it_block_begin != it_block_end) {
      std::copy(it_block_begin, it_block_end, std::back_inserter(reduced_vector));
    }
    it_block_begin = it_block_end + 1;
  }

  // Copy the last block.
  if (it_block_begin != data.end()) {
    std::copy(it_block_begin, data.end(), std::back_inserter(reduced_vector));
  }
  return reduced_vector;
}

namespace stl_helpers {

constexpr int kColumns = 0;
constexpr int kRows = 1;

// Helps to pass the supplied dynamic matrix to functions taking containers
// with only one dimension, such as the below eraseIndicesFromContainer().
template <typename ScalarType, int StaticDimension>
struct OneDimensionAdapter {
  typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>
  DynamicMatrix;
  DynamicMatrix* matrix;
  const bool allocated_self;

  OneDimensionAdapter(DynamicMatrix* _matrix)
  : matrix(CHECK_NOTNULL(_matrix)), allocated_self(false) {}
  OneDimensionAdapter() : matrix(new DynamicMatrix), allocated_self(true) {}
  ~OneDimensionAdapter() {
    if (allocated_self) {
      delete matrix;
    }
  }

  void swap(OneDimensionAdapter& other) {
    matrix->swap(*other.matrix);
  }
};

// Different implementations are available in the inline header.
template <typename ContainerType>
void eraseIndicesFromContainer(
    const std::vector<size_t>& ordered_indices_to_erase,
    const size_t expected_initial_count, ContainerType* container);

}  // namespace stl_helpers
}  // namespace common
}  // namespace aslam

#include "./stl-helpers-inl.h"

#endif  // ASLAM_STL_HELPERS_H_
