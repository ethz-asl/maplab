#include <unordered_set>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <eigen-checks/gtest.h>

#include "maplab-common/eigen-helpers.h"
#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/test/testing-predicates.h"

namespace common {

TEST(MaplabCommon, EigenHelpers_RemoveColsFromEigen) {
  // Create test matrix.
  const int kNumCols = 10u;
  Eigen::MatrixXd matrix_full(2, kNumCols);
  for (int i = 0; i < kNumCols; ++i) {
    matrix_full.col(i).setConstant(i);
  }

  auto createGroundtruthMatrix = [kNumCols](
      const std::vector<size_t>& remove_indices, Eigen::MatrixXd* output) {
    CHECK_NOTNULL(output)->setZero();
    std::unordered_set<size_t> remove_indices_set(
        remove_indices.begin(), remove_indices.end());
    output->resize(2, kNumCols - remove_indices_set.size());
    size_t output_index = 0u;
    for (int i = 0; i < kNumCols; ++i) {
      if (remove_indices_set.count(i) == 0u) {
        output->col(output_index).setConstant(i);
        ++output_index;
      }
    }
  };

  // Remove some middle columns.
  Eigen::MatrixXd matrix = matrix_full;
  std::vector<size_t> indices_to_remove{1, 5};
  common::RemoveColsFromEigen(indices_to_remove, &matrix);

  Eigen::MatrixXd matrix_expected;
  createGroundtruthMatrix(indices_to_remove, &matrix_expected);
  ASSERT_EQ(matrix.cols(), matrix_expected.cols());
  ASSERT_EQ(matrix.rows(), matrix_expected.rows());
  EXPECT_NEAR_EIGEN(matrix, matrix_expected, 1e-7);

  // Test corner cases: remove first column ...
  matrix = matrix_full;
  indices_to_remove = std::vector<size_t>{0};
  common::RemoveColsFromEigen(indices_to_remove, &matrix);

  createGroundtruthMatrix(indices_to_remove, &matrix_expected);
  ASSERT_EQ(matrix.cols(), matrix_expected.cols());
  ASSERT_EQ(matrix.rows(), matrix_expected.rows());
  EXPECT_NEAR_EIGEN(matrix, matrix_expected, 1e-7);

  // ... now remove last column.
  matrix = matrix_full;
  indices_to_remove =
      std::vector<size_t>{static_cast<size_t>(matrix.cols() - 1)};
  common::RemoveColsFromEigen(indices_to_remove, &matrix);

  createGroundtruthMatrix(indices_to_remove, &matrix_expected);
  ASSERT_EQ(matrix.cols(), matrix_expected.cols());
  ASSERT_EQ(matrix.rows(), matrix_expected.rows());
  EXPECT_NEAR_EIGEN(matrix, matrix_expected, 1e-7);

  // Test out of range;
  matrix = matrix_full;
  indices_to_remove = std::vector<size_t>{100000};
  EXPECT_DEATH(common::RemoveColsFromEigen(indices_to_remove, &matrix), "^");

  // Non-ordered input indices.
  matrix = matrix_full;
  indices_to_remove = std::vector<size_t>{9, 2, 5, 3};
  common::RemoveColsFromEigen(indices_to_remove, &matrix);

  createGroundtruthMatrix(indices_to_remove, &matrix_expected);
  ASSERT_EQ(matrix.cols(), matrix_expected.cols());
  ASSERT_EQ(matrix.rows(), matrix_expected.rows());
  EXPECT_NEAR_EIGEN(matrix, matrix_expected, 1e-7);

  // Input with duplicates.
  matrix = matrix_full;
  indices_to_remove = std::vector<size_t>{2, 2, 3, 5};
  common::RemoveColsFromEigen(indices_to_remove, &matrix);

  createGroundtruthMatrix(indices_to_remove, &matrix_expected);
  ASSERT_EQ(matrix.cols(), matrix_expected.cols());
  ASSERT_EQ(matrix.rows(), matrix_expected.rows());
  EXPECT_NEAR_EIGEN(matrix, matrix_expected, 1e-7);
}

TEST(MaplabCommon, EigenHelpers_AppendColsRightToEigen) {
  const int kNumInitCols = 10u;
  const int kNumAppendCols = 5u;

  Eigen::Matrix2Xd matrix_merged_groundtruth(2, kNumInitCols + kNumAppendCols);

  Eigen::Matrix2Xd matrix_init(2, kNumInitCols);
  for (int i = 0; i < kNumInitCols; ++i) {
    matrix_init.col(i).setConstant(i);
    matrix_merged_groundtruth.col(i).setConstant(i);
  }

  Eigen::Matrix2Xd matrix_append(2, kNumAppendCols);
  for (int i = 0; i < kNumAppendCols; ++i) {
    matrix_append.col(i).setConstant(kNumInitCols + i);
    matrix_merged_groundtruth.col(kNumInitCols + i)
        .setConstant(kNumInitCols + i);
  }

  AppendColsRightToEigen(matrix_append, &matrix_init);

  ASSERT_EQ(matrix_init.cols(), matrix_merged_groundtruth.cols());
  ASSERT_EQ(matrix_init.rows(), matrix_merged_groundtruth.rows());
  EXPECT_NEAR_EIGEN(matrix_init, matrix_merged_groundtruth, 1e-7);

  // Append to empty.
  Eigen::Matrix2Xd empty_matrix;
  AppendColsRightToEigen(matrix_init, &empty_matrix);

  ASSERT_EQ(empty_matrix.cols(), matrix_merged_groundtruth.cols());
  ASSERT_EQ(empty_matrix.rows(), matrix_merged_groundtruth.rows());
  EXPECT_NEAR_EIGEN(empty_matrix, matrix_merged_groundtruth, 1e-7);

  // Append empty.
  Eigen::Matrix2Xd empty_matrix2;
  AppendColsRightToEigen(empty_matrix2, &matrix_init);

  ASSERT_EQ(matrix_init.cols(), matrix_merged_groundtruth.cols());
  ASSERT_EQ(matrix_init.rows(), matrix_merged_groundtruth.rows());
  EXPECT_NEAR_EIGEN(matrix_init, matrix_merged_groundtruth, 1e-7);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
