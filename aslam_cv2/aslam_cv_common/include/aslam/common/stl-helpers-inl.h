#ifndef ASLAM_COMMON_STL_HELPERS_INL_H_
#define ASLAM_COMMON_STL_HELPERS_INL_H_

#include <algorithm>

namespace aslam {
namespace common {
namespace stl_helpers {

namespace internal {

template <typename ScalarType, int Rows>
void copyMiddle(
    const int source_start, const int destination_start, const int size,
    Eigen::Matrix<ScalarType, Rows, Eigen::Dynamic>* source,
    Eigen::Matrix<ScalarType, Rows, Eigen::Dynamic>* destination) {
  CHECK_NOTNULL(source);
  CHECK_NOTNULL(destination);
  destination->middleCols(destination_start, size) =
      source->middleCols(source_start, size);
}
template <typename ScalarType, int Cols>
void copyMiddle(
    const int source_start, const int destination_start, const int size,
    Eigen::Matrix<ScalarType, Eigen::Dynamic, Cols>* source,
    Eigen::Matrix<ScalarType, Eigen::Dynamic, Cols>* destination) {
  CHECK_NOTNULL(source);
  CHECK_NOTNULL(destination);
  destination->middleRows(destination_start, size) =
      source->middleRows(source_start, size);
}
template <typename ScalarType>
void copyMiddle(
    const int source_start, const int destination_start, const int size,
    OneDimensionAdapter<ScalarType, kColumns>* source,
    OneDimensionAdapter<ScalarType, kColumns>* destination) {
  CHECK_NOTNULL(source);
  CHECK_NOTNULL(destination);
  destination->matrix->middleCols(destination_start, size) =
      source->matrix->middleCols(source_start, size);
}
template <typename ScalarType>
void copyMiddle(
    const int source_start, const int destination_start, const int size,
    OneDimensionAdapter<ScalarType, kRows>* source,
    OneDimensionAdapter<ScalarType, kRows>*
    destination) {
  CHECK_NOTNULL(source);
  CHECK_NOTNULL(destination);
  destination->matrix->middleRows(destination_start, size) =
      source->matrix->middleRows(source_start, size);
}

// The reference is necessary for proper functioning of OneDimensionAdapter
// resizing.
template <typename ScalarType, int Rows>
void resizeDynamicDimensionLike(
    const int new_size,
    const Eigen::Matrix<ScalarType, Rows, Eigen::Dynamic>& /*reference*/,
    Eigen::Matrix<ScalarType, Rows, Eigen::Dynamic>* matrix) {
  CHECK_NOTNULL(matrix);
  matrix->resize(Eigen::NoChange, new_size);
}
template <typename ScalarType, int Cols>
void resizeDynamicDimensionLike(
    const int new_size,
    const Eigen::Matrix<ScalarType, Eigen::Dynamic, Cols>& /*reference*/,
    Eigen::Matrix<ScalarType, Eigen::Dynamic, Cols>* matrix) {
  CHECK_NOTNULL(matrix);
  matrix->resize(new_size, Eigen::NoChange);
}
template <typename ScalarType>
void resizeDynamicDimensionLike(
    const int new_size,
    const OneDimensionAdapter<ScalarType, kColumns>& reference,
    OneDimensionAdapter<ScalarType, kColumns>* matrix) {
  CHECK_NOTNULL(matrix);
  matrix->matrix->resize(reference.matrix->rows(), new_size);
}
template <typename ScalarType>
void resizeDynamicDimensionLike(
    const int new_size,
    const OneDimensionAdapter<ScalarType, kRows>& reference,
    OneDimensionAdapter<ScalarType, kRows>* matrix) {
  CHECK_NOTNULL(matrix);
  matrix->matrix->resize(new_size, reference.matrix->cols());
}


template <typename ScalarType, int Rows>
size_t dynamicSize(
    const Eigen::Matrix<ScalarType, Rows, Eigen::Dynamic>& matrix) {
  return matrix.cols();
}
template <typename ScalarType, int Cols>
size_t dynamicSize(
    const Eigen::Matrix<ScalarType, Eigen::Dynamic, Cols>& matrix) {
  return matrix.rows();
}
template <typename ScalarType>
size_t dynamicSize(
    const OneDimensionAdapter<ScalarType, kColumns>& matrix) {
  return matrix.matrix->cols();
}
template <typename ScalarType>
size_t dynamicSize(
    const OneDimensionAdapter<ScalarType, kRows>& matrix) {
  return matrix.matrix->rows();
}

}  // namespace internal

template <typename ContainerType>
void eraseIndicesFromContainer(
    const std::vector<size_t>& ordered_indices_to_erase,
    const size_t expected_initial_count,
    ContainerType* container) {
  CHECK_NOTNULL(container);
  CHECK_EQ(internal::dynamicSize(*container), expected_initial_count);
  CHECK_LT(ordered_indices_to_erase.back(), expected_initial_count);

  if (ordered_indices_to_erase.empty()) {
    return;
  }

  const size_t erase_count = ordered_indices_to_erase.size();
  CHECK_LE(erase_count, expected_initial_count);

  LOG_IF(WARNING, erase_count == expected_initial_count)
      << "All items will be removed!";

  const size_t remaining_count = expected_initial_count - erase_count;
  ContainerType result;
  internal::resizeDynamicDimensionLike(remaining_count, *container, &result);

  int result_fill_index = 0;
  int container_block_start = 0;
  for (size_t i = 0u; i < ordered_indices_to_erase.size(); ++i) {
    CHECK_GE(static_cast<int>(ordered_indices_to_erase[i]), container_block_start);
    const int block_size = ordered_indices_to_erase[i] - container_block_start;

    internal::copyMiddle(container_block_start, result_fill_index, block_size,
                         container, &result);

    result_fill_index += block_size;
    container_block_start = ordered_indices_to_erase[i] + 1;
  }
  const int last_block_size = expected_initial_count - container_block_start;
  internal::copyMiddle(container_block_start, result_fill_index,
                       last_block_size, container, &result);

  container->swap(result);
}

template<typename ElementType, typename Allocator>
void eraseIndicesFromContainer(
    const std::vector<size_t>& ordered_indices_to_erase,
    const size_t expected_initial_count,
    std::vector<ElementType, Allocator>* container) {
  CHECK_NOTNULL(container);
  CHECK_EQ(expected_initial_count, container->size());
  std::vector<ElementType, Allocator> result =
      eraseIndicesFromVector(*container, ordered_indices_to_erase);
  container->swap(result);
}

}  // namespace stl_helpers
}  // namespace common
}  // namespace aslam

#endif  // ASLAM_COMMON_STL_HELPERS_INL_H_
