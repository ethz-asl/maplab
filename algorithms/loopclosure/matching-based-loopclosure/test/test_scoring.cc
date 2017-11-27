#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include <loopclosure-common/types.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "matching-based-loopclosure/scoring.h"

namespace matching_based_loopclosure {
namespace scoring {

class EmptyScoringTest : public ::testing::Test {
 protected:
  typedef int IdType;
  loop_closure::IdToMatches<IdType> id_to_matches_;
  loop_closure::IdToNumDescriptors<IdType> id_to_num_descriptors_;
  size_t num_descriptors_in_database_;
};

class ScoringTest : public ::testing::Test {
 protected:
  typedef int IdType;

  void SetUp() override {
    typedef size_t NumMatches;
    typedef size_t NumDescriptors;

    // What is being tested? We assume that we have a number of IDs. Each one
    // of them has a number of associated descriptors that can receive a vote.
    // In this test scenario, we assume that they have already received a
    // predefined number of votes. Therefore, we can compute the scores for
    // the IDs and test if they conform with the expected scores.
    //
    // Total num descriptors in database: 50 (has to be larger or equal to sum
    // of num descriptors of each ID)
    // Total num votes: 2 + 10 + 6 = 18
    //
    // ID:                            0   1       2
    // Num matches:                   2   10      6
    // Num descriptors:               10  10      10
    //
    // Expected acc. scores:          2   10      6
    // Expected prob. scores:         0   3.12392 1.08807

    std::vector<IdType> id_list = {0, 1, 2};
    std::vector<NumMatches> num_matches_list = {2, 10, 6};
    std::vector<NumDescriptors> num_descriptors_list = {10, 10, 10};
    std::vector<scoring::ScoreType> expected_accumulation_scores(
        num_matches_list.begin(), num_matches_list.end());
    std::vector<scoring::ScoreType> expected_probabilistic_scores = {
        0, 3.12392f, 1.08807f};
    const size_t num_ids = id_list.size();

    CHECK_EQ(num_ids, num_matches_list.size());
    CHECK_EQ(num_ids, num_descriptors_list.size());
    CHECK_EQ(num_ids, expected_accumulation_scores.size());
    CHECK_EQ(num_ids, expected_probabilistic_scores.size());

    for (size_t idx = 0u; idx < num_ids; ++idx) {
      const IdType current_id = id_list[idx];
      for (size_t matches_count = 0u; matches_count < num_matches_list[idx];
           ++matches_count) {
        id_to_matches_[current_id].emplace_back();
      }
      id_to_num_descriptors_[current_id] = num_descriptors_list[idx];
      id_to_expected_accumulation_score_[current_id] =
          expected_accumulation_scores[idx];
      id_to_expected_probabilistic_score_[current_id] =
          expected_probabilistic_scores[idx];
    }
    CHECK_EQ(num_ids, id_to_matches_.size());
    CHECK_EQ(num_ids, id_to_num_descriptors_.size());

    num_descriptors_in_database_ = 50u;
    CHECK_GE(
        num_descriptors_in_database_,
        std::accumulate(
            num_descriptors_list.begin(), num_descriptors_list.end(), 0u));
  }

  loop_closure::IdToMatches<IdType> id_to_matches_;
  loop_closure::IdToNumDescriptors<IdType> id_to_num_descriptors_;
  size_t num_descriptors_in_database_;

  std::unordered_map<IdType, scoring::ScoreType>
      id_to_expected_accumulation_score_;
  std::unordered_map<IdType, scoring::ScoreType>
      id_to_expected_probabilistic_score_;
};

TEST_F(EmptyScoringTest, AccumulationScoring) {
  matching_based_loopclosure::scoring::ScoreList<IdType> scores;
  computeAccumulationScore(
      id_to_matches_, id_to_num_descriptors_, num_descriptors_in_database_,
      &scores);
  EXPECT_EQ(scores.size(), 0u);
}

TEST_F(EmptyScoringTest, ProbabilisticScoring) {
  matching_based_loopclosure::scoring::ScoreList<IdType> scores;
  computeProbabilisticScore(
      id_to_matches_, id_to_num_descriptors_, num_descriptors_in_database_,
      &scores);
  EXPECT_EQ(scores.size(), 0u);
}

TEST_F(ScoringTest, AccumulationScoring) {
  matching_based_loopclosure::scoring::ScoreList<IdType> scores;
  computeAccumulationScore(
      id_to_matches_, id_to_num_descriptors_, num_descriptors_in_database_,
      &scores);
  EXPECT_EQ(scores.size(), id_to_expected_accumulation_score_.size());
  for (const scoring::Score<IdType>& score : scores) {
    const IdType current_id = score.first;
    const scoring::ScoreType current_score = score.second;
    const std::unordered_map<IdType, scoring::ScoreType>::const_iterator iter =
        id_to_expected_accumulation_score_.find(current_id);
    CHECK(iter != id_to_expected_accumulation_score_.cend());
    EXPECT_EQ(current_score, iter->second);
  }
}

TEST_F(ScoringTest, ProbabilisticScoring) {
  matching_based_loopclosure::scoring::ScoreList<IdType> scores;
  computeProbabilisticScore(
      id_to_matches_, id_to_num_descriptors_, num_descriptors_in_database_,
      &scores);
  EXPECT_EQ(scores.size(), id_to_expected_probabilistic_score_.size());
  for (const scoring::Score<IdType>& score : scores) {
    const IdType current_id = score.first;
    const scoring::ScoreType current_score = score.second;
    const std::unordered_map<IdType, scoring::ScoreType>::const_iterator iter =
        id_to_expected_probabilistic_score_.find(current_id);
    CHECK(iter != id_to_expected_probabilistic_score_.cend());
    constexpr auto kScoreTolerance = static_cast<scoring::ScoreType>(1e-4);
    EXPECT_NEAR(current_score, iter->second, kScoreTolerance);
  }
}

}  // namespace scoring
}  // namespace matching_based_loopclosure

MAPLAB_UNITTEST_ENTRYPOINT
