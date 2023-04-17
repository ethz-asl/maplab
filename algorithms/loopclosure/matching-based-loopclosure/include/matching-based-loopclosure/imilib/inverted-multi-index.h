#ifndef INVERTED_MULTI_INDEX_INVERTED_MULTI_INDEX_H_
#define INVERTED_MULTI_INDEX_INVERTED_MULTI_INDEX_H_

#include <Eigen/Core>
#include <algorithm>
#include <aslam/common/memory.h>
#include <functional>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <limits>
#include <maplab-common/eigen-proto.h>
#include <memory>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "matching-based-loopclosure/imilib//inverted-multi-index-common.h"

DECLARE_double(lc_knn_max_radius);

namespace loop_closure {
namespace inverted_multi_index {
// Variant of the inverted multi-index, which stores the original descriptors in
// the inverted files.
// The template parameter is the dimensionality of the subvectors.
template <int kDimSubVectors>
class InvertedMultiIndex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<float, 2 * kDimSubVectors, 1> DescriptorType;
  typedef Eigen::Matrix<float, kDimSubVectors, 1> SubDescriptorType;
  typedef Eigen::Matrix<float, 2 * kDimSubVectors, Eigen::Dynamic>
      DescriptorMatrixType;
  typedef common::InvertedFile<float, 2 * kDimSubVectors> InvFile;

  // Creates the index from a given set of visual words. Each column in words_i
  // specifies a cluster center coordinate.
  InvertedMultiIndex(
      const Eigen::MatrixXf& words_1, const Eigen::MatrixXf& words_2,
      int num_closest_words_for_nn_search)
      : words_1_(words_1),
        words_2_(words_2),
        words_1_index_(common::NNSearch::createKDTreeLinearHeap(
            words_1_, kDimSubVectors, common::kCollectTouchStatistics)),
        words_2_index_(common::NNSearch::createKDTreeLinearHeap(
            words_2_, kDimSubVectors, common::kCollectTouchStatistics)),
        num_closest_words_for_nn_search_(num_closest_words_for_nn_search),
        max_db_descriptor_index_(0) {
    CHECK_EQ(words_1.rows(), kDimSubVectors);
    CHECK_GT(words_1.cols(), 0);
    CHECK_EQ(words_2.rows(), kDimSubVectors);
    CHECK_GT(words_2.cols(), 0);
    CHECK_GT(num_closest_words_for_nn_search_, 0);
  }

  void SetNumClosestWordsForNNSearch(int num_closest_words_for_nn_search) {
    CHECK_GT(num_closest_words_for_nn_search, 0);
    num_closest_words_for_nn_search_ = num_closest_words_for_nn_search;
  }

  inline int GetNumDescriptorsInIndex() const {
    return max_db_descriptor_index_;
  }

  // Clears the inverted multi-index by removing all references to the database
  // descriptors stored in it. Does NOT remove the underlying quantization.
  inline void Clear() {
    inverted_files_.clear();
    word_index_map_.clear();
    max_db_descriptor_index_ = 0;
  }

  // Adds a set of database descriptors to the inverted multi-index.
  // Each column defines a database descriptor.
  void AddDescriptors(const DescriptorMatrixType& descriptors) {
    const int num_descriptors = descriptors.cols();
    std::vector<std::pair<int, int> > closest_word;

    for (int i = 0; i < num_descriptors; ++i) {
      common::FindClosestWords<kDimSubVectors>(
          descriptors.col(i), 1, *words_1_index_, *words_2_index_,
          words_1_.cols(), words_2_.cols(), &closest_word);
      CHECK(!closest_word.empty());
      const int word_index =
          closest_word[0].first * words_2_.cols() + closest_word[0].second;

      common::AddDescriptor<float, 2 * kDimSubVectors>(
          descriptors.col(i), max_db_descriptor_index_, word_index,
          &word_index_map_, &inverted_files_);
      ++max_db_descriptor_index_;
    }
  }

  // Finds the n nearest neighbors for a given query feature.
  // This function is thread-safe.
  template <
      typename DerivedQuery, typename DerivedIndices, typename DerivedDistances>
  inline void GetNNearestNeighbors(
      const Eigen::MatrixBase<DerivedQuery>& query_feature, int num_neighbors,
      const Eigen::MatrixBase<DerivedIndices>& out_indices,
      const Eigen::MatrixBase<DerivedDistances>& out_distances) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(
        DerivedQuery, static_cast<int>(2 * kDimSubVectors), 1);
    CHECK_EQ(out_indices.cols(), 1);
    CHECK_EQ(out_distances.cols(), 1);
    CHECK_GT(num_neighbors, 0);

    CHECK_EQ(out_indices.rows(), num_neighbors)
        << "The indices parameter must be pre-allocated to hold all results.";
    CHECK_EQ(out_distances.rows(), num_neighbors)
        << "The distances parameter must be pre-allocated to hold all results.";

    // Zero copy passing of Eigen-Block Expressions.
    // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
    Eigen::MatrixBase<DerivedIndices>& indices =
        const_cast<Eigen::MatrixBase<DerivedIndices>&>(out_indices);
    Eigen::MatrixBase<DerivedDistances>& distances =
        const_cast<Eigen::MatrixBase<DerivedDistances>&>(out_distances);

    // Finds the closest visual words.
    std::vector<std::pair<int, int> > closest_words;
    common::FindClosestWords<kDimSubVectors>(
        query_feature, num_closest_words_for_nn_search_, *words_1_index_,
        *words_2_index_, words_1_.cols(), words_2_.cols(), &closest_words);

    // Performs exhaustive search through all descriptors assigned to the
    // closest words.
    std::vector<std::pair<float, int> > nearest_neighbors;
    nearest_neighbors.reserve(num_neighbors + 1);
    const int num_words_to_use = static_cast<int>(closest_words.size());
    std::unordered_map<int, int>::const_iterator word_index_map_it;

    for (int i = 0; i < num_words_to_use; ++i) {
      const int word_index =
          closest_words[i].first * words_2_.cols() + closest_words[i].second;
      word_index_map_it = word_index_map_.find(word_index);
      if (word_index_map_it == word_index_map_.end())
        continue;

      const InvFile& inverted_file = inverted_files_[word_index_map_it->second];
      const size_t num_descriptors = inverted_file.descriptors_.size();
      for (size_t j = 0; j < num_descriptors; ++j) {
        const float distance =
            (inverted_file.descriptors_[j] - query_feature).squaredNorm();
        common::InsertNeighbor(
            inverted_file.indices_[j], distance, num_neighbors,
            &nearest_neighbors);
      }
    }

    for (size_t i = 0; i < nearest_neighbors.size(); ++i) {
      indices(i, 0) = nearest_neighbors[i].second;
      distances(i, 0) = nearest_neighbors[i].first;
    }
    for (int i = nearest_neighbors.size(); i < num_neighbors; ++i) {
      indices(i, 0) = -1;
      distances(i, 0) = std::numeric_limits<float>::infinity();
    }
  }

 protected:
  // The two sets of cluster centers defining the quantization of the descriptor
  // space as the Cartesian product of the two sets of words.
  Eigen::MatrixXf words_1_;
  Eigen::MatrixXf words_2_;
  std::shared_ptr<common::NNSearch> words_1_index_;
  std::shared_ptr<common::NNSearch> words_2_index_;

  // The number of closest words from the product vocabulary that should be used
  // during nearest neighbor search.
  int num_closest_words_for_nn_search_;
  // Hashmap storing for each combined visual word the index in inverted_files_
  // in which all database descriptors assigned to that word can be found.
  // This allows us to easily add descriptors assigned to words that have not
  // been used previously without having to re-order large amounts of memory.
  std::unordered_map<int, int> word_index_map_;
  // Vector containing the inverted files, one for each visual word in the
  // product vocabulary. Each inverted file holds all descriptors assigned to
  // the corresponding word and their indices.
  Aligned<std::vector, InvFile> inverted_files_;
  // The maximum index of the descriptor indices.
  int max_db_descriptor_index_;
};
}  // namespace inverted_multi_index
}  // namespace loop_closure

#endif  // INVERTED_MULTI_INDEX_INVERTED_MULTI_INDEX_H_
