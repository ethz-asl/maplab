#include <vector>

#include <aslam/common/entrypoint.h>
#include <aslam/common/stl-helpers.h>
#include <gtest/gtest.h>

using namespace aslam;

TEST(StlHelpers, EraseIndices) {
  std::vector<int> test_vector = {0, 1, 2, 3, 4, 5};
  std::vector<size_t> indices_to_remove = {2, 4};

  std::vector<int> result_vector =
      aslam::common::eraseIndicesFromVector(test_vector, indices_to_remove);
  ASSERT_EQ(result_vector.size(), test_vector.size() - indices_to_remove.size());

  std::vector<int> expected_result = {0, 1, 3, 5};
  ASSERT_EQ(result_vector.size(), expected_result.size());
  for (size_t idx = 0u; idx < result_vector.size(); ++idx) {
    EXPECT_EQ(expected_result[idx], result_vector[idx]);
  }
}

TEST(StlHelpers, EraseIndicesAligned) {
  Aligned<std::vector, Eigen::Vector3i> test_vector;
  test_vector.push_back(Eigen::Vector3i::Constant(0));
  test_vector.push_back(Eigen::Vector3i::Constant(1));
  test_vector.push_back(Eigen::Vector3i::Constant(2));
  test_vector.push_back(Eigen::Vector3i::Constant(3));
  std::vector<size_t> indices_to_remove = {2, 1};

  Aligned<std::vector, Eigen::Vector3i> result_vector =
      aslam::common::eraseIndicesFromVector(test_vector, indices_to_remove);
  ASSERT_EQ(result_vector.size(), test_vector.size() - indices_to_remove.size());

  std::vector<int> expected_result = {0, 3};
  ASSERT_EQ(result_vector.size(), expected_result.size());
  for (size_t idx = 0u; idx < result_vector.size(); ++idx) {
    EXPECT_EQ(expected_result[idx], result_vector[idx](0));
  }
}

TEST(StlHelpers, DrawRandom) {
  std::vector<int> test_vector = {0, 1, 2, 3, 4, 5};
  std::vector<int> output;

  constexpr size_t kNumToPick = 3u;
  constexpr bool kUseFixedSeedTrue = true;
  aslam::common::drawNRandomElements(
      kNumToPick, test_vector, &output, kUseFixedSeedTrue);
  EXPECT_EQ(output.size(), kNumToPick);

  output.clear();
  constexpr bool kUseFixedSeedFalse = false;
  aslam::common::drawNRandomElements(
      kNumToPick, test_vector, &output, kUseFixedSeedFalse);
  EXPECT_EQ(output.size(), kNumToPick);

  output.clear();
  aslam::common::drawNRandomElements(kNumToPick, test_vector, &output);
  EXPECT_EQ(output.size(), kNumToPick);

  // Fail when input and output are the same, because this is not allowed.
  EXPECT_DEATH(aslam::common::drawNRandomElements(
      kNumToPick, test_vector, &test_vector), "");

  // Make sure that if we pick a single index 1000 times, every index
  // (i.e., 0 to 5) is at least picked once.
  std::unordered_set<int> indices_encountered;
  const size_t kNumTrials = 1000u;
  for (size_t i = 0u; i < kNumTrials; ++i) {
    aslam::common::drawNRandomElements(
        kNumToPick, test_vector, &output, kUseFixedSeedFalse);
    EXPECT_EQ(output.size(), kNumToPick);
    indices_encountered.insert(output.begin(), output.end());
  }
  EXPECT_EQ(indices_encountered, std::unordered_set<int>({0, 1, 2, 3, 4, 5}));
}

TEST(StdHelpers, CountNestedListElements) {
  const size_t kArbitraryNumElementsOfList = 123u;
  Aligned<std::vector, int> eigen_list(kArbitraryNumElementsOfList);

  const size_t kArbitraryNumElementsOfNestedList = 93u;
  Aligned<std::vector, Aligned<std::vector, int>> eigen_nested_list(
      kArbitraryNumElementsOfNestedList, eigen_list);

  size_t num_elements = aslam::common::countNumberOfElementsInNestedList(
      eigen_nested_list);

  CHECK_EQ(num_elements,
           kArbitraryNumElementsOfList * kArbitraryNumElementsOfNestedList);

  std::vector<int> std_list(kArbitraryNumElementsOfList);
  std::vector<std::vector<int>> std_nested_list(
      kArbitraryNumElementsOfNestedList, std_list);

  num_elements = aslam::common::countNumberOfElementsInNestedList(
      std_nested_list);

  CHECK_EQ(num_elements,
           kArbitraryNumElementsOfList *  kArbitraryNumElementsOfNestedList);
}

ASLAM_UNITTEST_ENTRYPOINT
