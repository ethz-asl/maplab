#ifndef MATCHING_BASED_LOOPCLOSURE_INVERTED_INDEX_H_
#define MATCHING_BASED_LOOPCLOSURE_INVERTED_INDEX_H_

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <loopclosure-common/flags.h>
#include <nabo/nabo.h>

namespace loop_closure {
namespace inverted_index {

template <int kDimVectors>
class InvertedIndex {
 public:
  typedef Eigen::Matrix<float, kDimVectors, 1> DescriptorType;
  typedef Eigen::Matrix<float, kDimVectors, Eigen::Dynamic>
      DescriptorMatrixType;
  typedef Nabo::NearestNeighbourSearch<float> NNSearch;
  // Switch touch statistics (NNSearch::TOUCH_STATISTICS) off for performance.
  static constexpr int kCollectTouchStatistics = 0;
  // Kd-tree search options. ALLOW_SELF_MATCH means that a point which is
  // equal to the query will be returned in the results.
  static constexpr unsigned kSearchOptionsDefault =
      NNSearch::ALLOW_SELF_MATCH | NNSearch::SORT_RESULTS;
  // Epsilon approximation factor for kd-tree backtracking.
  static constexpr float kSearchNNEpsilon = 0.2;
  // How many more neighbors to retrieve on either kd-tree to increase the
  // chance to get the k-nearest neighbors for the combined result.
  static constexpr int kSearchNNNumNeighborsMultiplier = 2;

  // Creates the index from a given set of visual words. Each column in words
  // specifies a cluster center coordinate.
  InvertedIndex(
      const Eigen::MatrixXf& words, int num_closest_words_for_nn_search)
      : words_(words),
        words_index_(
            NNSearch::createKDTreeLinearHeap(
                words_, kDimVectors, kCollectTouchStatistics)),
        num_closest_words_for_nn_search_(num_closest_words_for_nn_search),
        max_db_descriptor_index_(0) {
    CHECK_EQ(words_.rows(), kDimVectors);
    CHECK_GT(num_closest_words_for_nn_search_, 0);
  }

  void SetNumClosestWordsForNNSearch(int num_closest_words_for_nn_search) {
    num_closest_words_for_nn_search_ = num_closest_words_for_nn_search;
  }

  inline int GetNumDescriptorsInIndex() const {
    return db_descriptors_.size();
  }

  // Clears the inverted index by removing all references to the database
  // descriptors stored in it. Does NOT remove the underlying quantization.
  inline void Clear() {
    db_descriptors_.clear();
    db_descriptor_indices_.clear();
    word_index_map_.clear();
    max_db_descriptor_index_ = 0;
  }

  // Adds a set of database descriptors to the inverted index.
  // Each column defines a database descriptor.
  void AddDescriptors(const DescriptorMatrixType& descriptors) {
    int num_descriptors = descriptors.cols();
    std::unordered_map<int, int>::const_iterator word_index_it;

    for (int i = 0; i < num_descriptors; ++i) {
      std::vector<int> closest_word;
      FindClosestWords(descriptors.col(i), 1, &closest_word);
      CHECK(!closest_word.empty());
      word_index_it = word_index_map_.find(closest_word[0]);

      if (word_index_it == word_index_map_.end()) {
        word_index_it = word_index_map_
                            .insert(
                                std::make_pair(
                                    closest_word[0],
                                    static_cast<int>(db_descriptors_.size())))
                            .first;
        Aligned<std::vector, DescriptorType> new_word;
        new_word.emplace_back(descriptors.col(i));
        db_descriptors_.push_back(new_word);

        std::vector<int> new_indices;
        new_indices.push_back(max_db_descriptor_index_);
        db_descriptor_indices_.push_back(new_indices);
        ++max_db_descriptor_index_;
      } else {
        db_descriptors_[word_index_it->second].push_back(descriptors.col(i));
        db_descriptor_indices_[word_index_it->second].push_back(
            max_db_descriptor_index_);
        ++max_db_descriptor_index_;
      }
    }
  }

  // Finds the n nearest neighbors for a given query feature.
  // This function is thread-safe.
  template <typename DerivedQuery, typename DerivedIndices,
            typename DerivedDistances>
  inline void GetNNearestNeighbors(
      const Eigen::MatrixBase<DerivedQuery>& query_feature, int num_neighbors,
      const Eigen::MatrixBase<DerivedIndices>& out_indices,
      const Eigen::MatrixBase<DerivedDistances>& out_distances) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(
        DerivedQuery, static_cast<int>(kDimVectors), 1);
    CHECK_EQ(out_indices.cols(), 1);
    CHECK_EQ(out_distances.cols(), 1);

    CHECK_EQ(out_indices.rows(), num_neighbors)
        << "The indices parameter must be pre-allocated to hold all results.";
    CHECK_EQ(out_distances.rows(), num_neighbors)
        << "The distances parameter must be prea-llocated to hold all results.";

    // Zero copy passing of Eigen-Block Expressions.
    // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
    Eigen::MatrixBase<DerivedIndices>& indices =
        const_cast<Eigen::MatrixBase<DerivedIndices>&>(out_indices);
    Eigen::MatrixBase<DerivedDistances>& distances =
        const_cast<Eigen::MatrixBase<DerivedDistances>&>(out_distances);

    // Finds the closest visual words.
    std::vector<int> closest_words;
    FindClosestWords(
        query_feature, num_closest_words_for_nn_search_, &closest_words);

    // Performs exhaustive search through all descriptors assigned to the
    // closest words.
    std::vector<std::pair<float, int> > nearest_neighbors;
    nearest_neighbors.reserve(num_neighbors + 1);
    int num_words_to_use = static_cast<int>(closest_words.size());
    std::unordered_map<int, int>::const_iterator word_index_it;
    for (int i = 0; i < num_words_to_use; ++i) {
      int word_index = closest_words[i];
      word_index_it = word_index_map_.find(word_index);
      if (word_index_it == word_index_map_.end())
        continue;

      const Aligned<std::vector, DescriptorType>& descriptors =
          db_descriptors_[word_index_it->second];
      int num_descriptors = static_cast<int>(descriptors.size());
      for (int j = 0; j < num_descriptors; ++j) {
        float distance = (descriptors[j] - query_feature).squaredNorm();
        InsertNeighbor(
            db_descriptor_indices_[word_index_it->second][j], distance,
            num_neighbors, &nearest_neighbors);
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
  // Inserts a neighbor (defined by its id and distance to the query) in a
  // list, which is sorted based on squared Euclidean distances. num_neighbors
  // gives the maximum number of maintained neighbors. Neighbors not amongst the
  // num_neighbors closest ones are not used.
  inline static void InsertNeighbor(
      int index, float distance, int num_neighbors,
      std::vector<std::pair<float, int> >* const nearest_neighbors) {
    CHECK_NOTNULL(nearest_neighbors);
    int num_found_neighbors = static_cast<int>(nearest_neighbors->size());
    if (num_found_neighbors >= num_neighbors) {
      if (nearest_neighbors->back().first < distance)
        return;
    }

    std::pair<float, int> neighbor(distance, index);
    std::vector<std::pair<float, int> >::iterator it = std::lower_bound(
        nearest_neighbors->begin(), nearest_neighbors->end(), neighbor);
    nearest_neighbors->insert(it, neighbor);

    if (num_found_neighbors >= num_neighbors) {
      nearest_neighbors->resize(num_neighbors);
    }
  }

  // Finds the num_words closest visual words from the vocabulary. Their
  // indices are returned in closest_words in ascending order of distances.
  // closest_words is resized accordingly if less than num_words closest words
  // are available. The function calling FindClosestWords needs to ensure that
  // closest_words is not null.
  // This function is thread-safe.
  template <typename DerivedQuery>
  inline void FindClosestWords(
      const Eigen::MatrixBase<DerivedQuery>& query_feature, int num_words,
      std::vector<int>* closest_words) const {
    CHECK_NOTNULL(closest_words);
    // Computes the squared Euclidean distances to all visual words in the
    // vocabulary using the query descriptor.
    // The words are then sorted based on their distances.
    const NNSearch& words_index_ref = *words_index_;
    int num_neighbors = std::min<int>(words_.cols(), num_words);
    CHECK_GT(num_neighbors, 0) << words_.cols() << " " << num_words;

    NNSearch::IndexMatrix indices;
    indices.resize(num_neighbors, 1);
    NNSearch::Matrix distances;
    distances.resize(num_neighbors, 1);
    // Copy needed to transfer block-expression to Nabo compatible dyn. matrix.
    Eigen::MatrixXf query = query_feature;
    words_index_ref.knn(
        query, indices, distances, num_neighbors, kSearchNNEpsilon,
        kSearchOptionsDefault, FLAGS_lc_knn_max_radius);

    closest_words->reserve(std::min<int>(num_words, indices.rows()));
    for (int i = 0; i < indices.rows(); ++i) {
      if (indices(i, 0) != -1) {
        closest_words->push_back(indices(i, 0));
      } else {
        break;
      }
    }
  }

  // The set of cluster centers defining the quantization of the descriptor
  // space.
  Eigen::MatrixXf words_;
  std::shared_ptr<NNSearch> words_index_;

  // The number of closest words from the product vocabulary that should be used
  // during nearest neighbor search.
  int num_closest_words_for_nn_search_;

  // Hashmap storing for each visual word the index in db_descriptors_
  // in which all database descriptors assigned to that word can be found.
  // This allows us to easily add descriptors assigned to words that have not
  // been used previously without having to re-order large amounts of memory.
  std::unordered_map<int, int> word_index_map_;
  // Vector containing the database descriptors. Each entry of db_descriptors_
  // holds all descriptors assigned to a single visual word from the large
  // vocabulary obtained as the product of the two small ones.
  typedef Aligned<std::vector, DescriptorType> DescriptorBucket;
  Aligned<std::vector, DescriptorBucket> db_descriptors_;
  // The indices of the database descriptors for each word.
  std::vector<std::vector<int> > db_descriptor_indices_;
  // The maximum index of the descriptor indices.
  int max_db_descriptor_index_;
};
}  // namespace inverted_index
}  // namespace loop_closure

#endif  // MATCHING_BASED_LOOPCLOSURE_INVERTED_INDEX_H_
