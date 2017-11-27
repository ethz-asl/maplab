#include <gtest/gtest.h>
#include <vector>

#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/vector-window-operations.h"

namespace common {

namespace window_vec_ops {

static const double kAccuracy = 1e-4;

TEST(VectorWindowOps, TestRunningAverage) {
  const std::vector<double> values = {1.0, 3.0, 1.0, 2.0, -1.0, 7.0, 1.0, -1.0};
  const std::vector<double> expected_result = {
      1.666666667, 1.75, 1.75, 3.25, 2.75, 3.333333333, 4, 4};
  constexpr double kInvalidValue = -1.0;

  CHECK_EQ(values.size(), expected_result.size());

  constexpr unsigned int kWindowSize = 5;
  std::vector<double> computed_result;
  common::window_vec_ops::computeRunningAverage(
      values, kWindowSize, kInvalidValue, &computed_result);

  EXPECT_EQ(computed_result.size(), expected_result.size());

  for (unsigned int i = 0u; i < computed_result.size(); ++i) {
    EXPECT_NEAR(computed_result[i], expected_result[i], kAccuracy) << "Index "
                                                                   << i;
  }
}

TEST(VectorWindowOps, TestRunningAverageEmpty) {
  std::vector<double> empty_values;
  constexpr double kInvalidValue = -1.0;

  constexpr unsigned int kWindowSize = 5;
  std::vector<double> computed_result;
  common::window_vec_ops::computeRunningAverage(
      empty_values, kWindowSize, kInvalidValue, &computed_result);

  EXPECT_TRUE(computed_result.empty());
}

TEST(VectorWindowOps, TestRunningAverageAllInvalid) {
  const std::vector<double> invalid_values = {-1.0, -1.0, -1.0, -1.0};
  const std::vector<double> expected_result = {-1.0, -1.0, -1.0, -1.0};
  const double kInvalidValue = -1.0;

  constexpr unsigned int kWindowSize = 5;
  std::vector<double> computed_result;
  common::window_vec_ops::computeRunningAverage(
      invalid_values, kWindowSize, kInvalidValue, &computed_result);

  EXPECT_EQ(computed_result.size(), expected_result.size());

  for (unsigned int i = 0u; i < computed_result.size(); ++i) {
    EXPECT_NEAR(computed_result[i], expected_result[i], kAccuracy) << "Index "
                                                                   << i;
  }
}

TEST(VectorWindowOps, TestBoolVectorDilatation) {
  const std::vector<bool> bool_vec = {false, true,  false, false, false,
                                      false, false, true,  false, false,
                                      true,  false, false, false};
  const std::vector<bool> expected_result = {true, true, true, true, false,
                                             true, true, true, true, true,
                                             true, true, true, false};
  constexpr unsigned int kWindowSize = 5;

  std::vector<bool> computed_result;
  common::window_vec_ops::dilatateBoolVector(
      bool_vec, kWindowSize, &computed_result);

  EXPECT_EQ(computed_result.size(), expected_result.size());

  for (unsigned int i = 0u; i < computed_result.size(); ++i) {
    EXPECT_EQ(computed_result[i], expected_result[i]) << "Index " << i;
  }
}

TEST(VectorWindowOps, TestComputeAverage) {
  const std::vector<double> values = {1.0, 3.0, 1.0, 2.0, -1.0, 7.0, 1.0, -1.0};
  constexpr double kExpectedResult = 2.5;
  constexpr double kInvalidValue = -1.0;

  double computed_result =
      common::window_vec_ops::computeAverage<double>(values, kInvalidValue);

  EXPECT_NEAR(kExpectedResult, computed_result, kAccuracy);
}

}  // namespace window_vec_ops

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
