#ifndef MAPLAB_COMMON_EIGEN_HELPERS_H_
#define MAPLAB_COMMON_EIGEN_HELPERS_H_
#include <numeric>
#include <set>
#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace common {

template <
    typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows,
    int _MaxCols>
void RemoveColsFromEigen(
    const std::vector<std::size_t>& indices_to_remove_in,
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>*
        matrix) {
  CHECK_NOTNULL(matrix);

  if (indices_to_remove_in.empty()) {
    return;
  }

  // Sort and remove duplicates.
  std::set<std::size_t> indices_to_remove(
      indices_to_remove_in.begin(), indices_to_remove_in.end());

  // Remove columns.
  std::size_t input_read_upto = 0u;
  std::size_t output_end_index = 0u;

  for (const std::size_t idx_to_remove : indices_to_remove) {
    CHECK_LT(idx_to_remove, matrix->cols());

    const std::size_t num_cols_to_copy = idx_to_remove - input_read_upto;
    matrix->middleCols(output_end_index, num_cols_to_copy) =
        matrix->middleCols(input_read_upto, num_cols_to_copy);

    output_end_index += num_cols_to_copy;
    ++input_read_upto;
  }

  // Copy over the remeaining cols.
  const std::size_t num_cols_to_copy = matrix->cols() - input_read_upto;
  matrix->middleCols(output_end_index, num_cols_to_copy) =
      matrix->middleCols(input_read_upto, num_cols_to_copy);
  output_end_index += num_cols_to_copy;

  matrix->conservativeResize(
      Eigen::NoChange, matrix->cols() - indices_to_remove.size());
}

template <
    typename Scalar, int Rows, int ColsA, int ColsB, int Options, int MaxRows,
    int MaxCols>
void AppendColsRightToEigen(
    Eigen::Matrix<Scalar, Rows, ColsA, Options, MaxRows, MaxCols> cols_to_add,
    Eigen::Matrix<Scalar, Rows, ColsB, Options, MaxRows, MaxCols>*
        inout_merged) {
  CHECK_NOTNULL(inout_merged);

  if (cols_to_add.size() == 0) {
    return;
  }

  if (inout_merged->size() == 0) {
    *inout_merged = cols_to_add;
    return;
  }

  if (Rows == Eigen::Dynamic) {
    CHECK_EQ(cols_to_add.rows(), inout_merged->rows());
  }

  inout_merged->conservativeResize(
      Eigen::NoChange, inout_merged->cols() + cols_to_add.cols());
  inout_merged->rightCols(cols_to_add.cols()) = cols_to_add;
}

template <typename Compare>
std::vector<size_t> SortPermutationEigenVector(
    const Eigen::RowVectorXi& vec, const Compare& compare) {
  std::vector<size_t> permutation(vec.cols());
  std::iota(permutation.begin(), permutation.end(), 0);
  std::sort(
      permutation.begin(), permutation.end(),
      [&](std::size_t i, std::size_t j) {
        return compare(vec(0, i), vec(0, j));
      });
  return permutation;
}

}  // namespace common

#endif  // MAPLAB_COMMON_EIGEN_HELPERS_H_
