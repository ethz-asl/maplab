#include <unordered_map>

#include "maplab-common/combinatorial.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {
TEST(MaplabCommon, CombinatorialEmpty) {
  Aligned<std::vector, Eigen::VectorXi> combinations;
  const size_t kNumElements = 0u;
  getAllBinaryCombinations(kNumElements, &combinations);

  EXPECT_TRUE(combinations.empty());
}

TEST(MaplabCommon, CombinatorialBigNumber) {
  Aligned<std::vector, Eigen::VectorXi> combinations;
  const size_t kNumElements = 20u;
  getAllBinaryCombinations(kNumElements, &combinations);

  EXPECT_EQ(combinations.size(), 1 << kNumElements);
}

TEST(MaplabCommon, CombinatorialWithThreeElements) {
  Aligned<std::vector, Eigen::VectorXi> combinations;
  const size_t kNumElements = 3u;
  getAllBinaryCombinations(kNumElements, &combinations);

  // Create solution vectors.
  Aligned<std::vector, Eigen::Vector3i> solution_vectors(8u);

  solution_vectors[0] << 0, 0, 0;
  solution_vectors[1] << 1, 0, 0;
  solution_vectors[2] << 0, 1, 0;
  solution_vectors[3] << 0, 0, 1;
  solution_vectors[4] << 1, 1, 0;
  solution_vectors[5] << 0, 1, 1;
  solution_vectors[6] << 1, 0, 1;
  solution_vectors[7] << 1, 1, 1;

  const size_t num_solutions = solution_vectors.size();
  ASSERT_EQ(num_solutions, 8u) << "Invalid number of combinations. Should be "
                               << "equal to 2^3.";
  for (size_t solution_idx = 0u; solution_idx < 8u; ++solution_idx) {
    bool found = false;
    for (size_t combination_idx = 0u; combination_idx < num_solutions;
         ++combination_idx) {
      const bool match =
          combinations[combination_idx] == solution_vectors[solution_idx];
      EXPECT_FALSE(found && match)
          << "Found solution vector " << solution_vectors[solution_idx]
          << " twice in the list of returned combinations.";
      found |= match;
    }
    EXPECT_TRUE(found) << "Solution vector " << solution_vectors[solution_idx]
                       << " is missing from the list of returned combinations.";
  }
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
