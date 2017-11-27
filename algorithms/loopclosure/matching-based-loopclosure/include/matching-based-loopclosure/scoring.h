#ifndef MATCHING_BASED_LOOPCLOSURE_SCORING_H_
#define MATCHING_BASED_LOOPCLOSURE_SCORING_H_

#include <cmath>
#include <functional>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/math/distributions/binomial.hpp>
#include <descriptor-projection/descriptor-projection.h>
#include <loopclosure-common/types.h>
#include <vi-map/unique-id.h>

namespace matching_based_loopclosure {
namespace scoring {

typedef float ScoreType;

template <typename ScoreIdType>
using Score = std::pair<ScoreIdType, ScoreType>;

template <typename ScoreIdType>
using ScoreList = std::vector<Score<ScoreIdType>>;

template <typename ScoreIdType>
using computeScoresFunction = std::function<void(
    const loop_closure::IdToMatches<ScoreIdType>& id_to_matches_map,
    const loop_closure::IdToNumDescriptors<ScoreIdType>&
        descriptor_count_per_id,
    size_t num_descriptors_in_database, ScoreList<ScoreIdType>* scores)>;

// Simply score the number of votes/matches for an ID with the number of
// matches. This is the fastest and simplest scoring function and works well
// in most scenarios.
template <typename ScoreIdType>
void computeAccumulationScore(
    const loop_closure::IdToMatches<ScoreIdType>& id_to_matches_map,
    const loop_closure::IdToNumDescriptors<ScoreIdType>&
    /*descriptor_count_per_id*/,
    size_t /*num_descriptors_in_database*/, ScoreList<ScoreIdType>* scores) {
  CHECK_NOTNULL(scores)->clear();
  static_assert(
      std::is_arithmetic<ScoreType>::value,
      "The score type must be arithmetic.");

  const size_t num_ids = id_to_matches_map.size();
  if (num_ids == 0u) {
    LOG(WARNING) << "No matches are passed to the scoring function!";
    return;
  }
  scores->reserve(num_ids);
  for (const typename loop_closure::IdToMatches<ScoreIdType>::value_type&
           id_matches_pair : id_to_matches_map) {
    const auto score = static_cast<ScoreType>(id_matches_pair.second.size());
    scores->emplace_back(id_matches_pair.first, score);
  }
};

// Score number of votes associated with an ID in a probabilistic manner.
//
// The score returns the negative logarithm of the probability that the number
// of votes associated with an ID (this can be a keyframe ID or a set of
// keyframes to which an ID is assigned) is non-random. The returned score will
// be equal or larger than 0. Essentially, the score correlates with the
// probability that a place is revisited.
//
// This is useful if you are looking for a score that is independent of the
// size of the database (number of descriptors in the inverted-index/kd-tree),
// number of descriptors in each keyframe (or set thereof) of the
// database or number of returned nearest neighbors for the query
// keyframe. As a consequence, it could be used to discard unlikely
// loopclosures even before they are processed by a RANSAC scheme.
//
// This scoring scheme is especially useful if you have a multi-camera setup
// with different image resolution (thus potentially different number of
// features per image).
//
// This score makes the fewest assumptions compared to the others and in
// general should yield results as least as good or better than the other
// ones that are available. However, it is possible that you will get better
// results with other scoring functions for certain datasets.
//
// WARNING: This is significantly more expensive than scoring by accumulation.
//          Make sure that you only use this for a limited number (< 10^4) of
//          IDs. Can be accelerated significantly by using a smart
//          implementation (using pre-computed factorials) of the poisson
//          distribution to approximate the binomial distribution (not yet
//          implemented).
template <typename ScoreIdType>
void computeProbabilisticScore(
    const loop_closure::IdToMatches<ScoreIdType>& id_to_matches_map,
    const loop_closure::IdToNumDescriptors<ScoreIdType>&
        descriptor_count_per_id,
    size_t num_descriptors_in_database, ScoreList<ScoreIdType>* scores) {
  CHECK_NOTNULL(scores)->clear();
  static_assert(
      std::is_arithmetic<ScoreType>::value,
      "The score type must be arithmetic.");
  const size_t num_ids = id_to_matches_map.size();
  CHECK_LE(num_ids, descriptor_count_per_id.size())
      << "The number of IDs with matches cannot be greater than the map that "
      << "contains the number of descriptors associated with each ID.";

  if (num_descriptors_in_database == 0u) {
    LOG(WARNING) << "According to the arguments of this function, the database "
                 << "is empty.";
    return;
  }
  if (num_ids == 0u) {
    LOG(WARNING) << "No matches are passed to the scoring function.";
    return;
  }

  // Count the total number of votes that have been cast for the query.
  size_t total_num_matches = 0u;
  for (const typename loop_closure::IdToMatches<ScoreIdType>::value_type&
           id_matches_pair : id_to_matches_map) {
    total_num_matches += id_matches_pair.second.size();
  }

  size_t index_of_id_with_infinite_score = 0u;
  size_t num_matches_of_id_with_infinite_score = 0u;

  scores->reserve(num_ids);
  for (const typename loop_closure::IdToMatches<ScoreIdType>::value_type&
           id_matches_pair : id_to_matches_map) {
    const size_t num_matches_per_id = id_matches_pair.second.size();
    const typename loop_closure::IdToNumDescriptors<ScoreIdType>::const_iterator
        it = descriptor_count_per_id.find(id_matches_pair.first);
    CHECK(it != descriptor_count_per_id.cend());
    const size_t num_descriptors_per_id = it->second;
    CHECK_GT(num_descriptors_per_id, 0u)
        << "The matching frame must contain "
        << "at least one descriptor. Otherwise, there cannot be a match.";

    auto score = static_cast<ScoreType>(0);
    const double success_probability =
        static_cast<double>(num_descriptors_per_id) /
        static_cast<double>(num_descriptors_in_database);
    const auto lower_median_num_votes = static_cast<size_t>(
        static_cast<double>(total_num_matches) * success_probability);

    // We do not compute scores (they will have a score of 0) of IDs that have
    // fewer matches than the median/mean because we expect revisited places to
    // surpass this threshold. Essentially, we cut off the
    // few-matches-low-probability tail of the binomial distribution. We are
    // only interested in the numerous-matches-low-probability tail.
    if (num_matches_per_id > lower_median_num_votes) {
      // TODO(magehrig): Use poisson approximation for speed-up.
      boost::math::binomial bin_instance(
          total_num_matches, success_probability);
      // Probability that the number of votes/matches could be explained by
      // random matching of descriptors in the database.
      const double random_voting_probability =
          boost::math::pdf(bin_instance, num_matches_per_id);
      CHECK_GE(random_voting_probability, 0.0);
      CHECK_LE(random_voting_probability, 1.0);
      if (random_voting_probability == 0.0) {
        // This can happen if the probability would be so low that it cannot
        // be stored in doubles anymore. In that case, we remember the score
        // entry of the ID with the highest number of associated matches of all
        // IDs to which this applies.
        if (num_matches_per_id > num_matches_of_id_with_infinite_score) {
          num_matches_of_id_with_infinite_score = num_matches_per_id;
          // Take size as index because we will add this score to the vector
          // later.
          index_of_id_with_infinite_score = scores->size();
        }
        // Do not yet set score to infinity but max() because at most one score
        // will be infinity (id with the most matches).
        score = std::numeric_limits<ScoreType>::max();
      } else {
        // A higher score means that a loop is more likely.
        score = static_cast<ScoreType>(-std::log10(random_voting_probability));
      }
      CHECK_GT(score, static_cast<ScoreType>(0));
    }
    if (num_matches_of_id_with_infinite_score > 0u) {
      static_assert(std::numeric_limits<ScoreType>::has_infinity, "");
      (*scores)[index_of_id_with_infinite_score].second =
          std::numeric_limits<ScoreType>::infinity();
    }
    scores->emplace_back(id_matches_pair.first, score);
  }
};

}  // namespace scoring
}  // namespace matching_based_loopclosure

#endif  // MATCHING_BASED_LOOPCLOSURE_SCORING_H_
