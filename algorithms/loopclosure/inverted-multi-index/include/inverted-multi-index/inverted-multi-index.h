#ifndef INVERTED_MULTI_INDEX_INVERTED_MULTI_INDEX_H_
#define INVERTED_MULTI_INDEX_INVERTED_MULTI_INDEX_H_

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <nabo/nabo.h>

#include "inverted-multi-index/inverted-multi-index-common.h"
#include "inverted-multi-index/inverted_multi_index.pb.h"

DECLARE_double(lc_knn_max_radius);

namespace matching_based_loopclosure {
class MatchingBasedLoopDetectorSerializer;
}  // namespace matching_based_loopclosure

namespace loop_closure {
namespace inverted_multi_index {
// Variant of the inverted multi-index, which stores the original descriptors in
// the inverted files.
// The template parameter is the dimensionality of the subvectors.
template <int kDimSubVectors>
class InvertedMultiIndex {
 public:
  friend class matching_based_loopclosure::MatchingBasedLoopDetectorSerializer;
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
        words_1_index_(
            common::NNSearch::createKDTreeLinearHeap(
                words_1_, kDimSubVectors, common::kCollectTouchStatistics)),
        words_2_index_(
            common::NNSearch::createKDTreeLinearHeap(
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
  template <typename DerivedQuery, typename DerivedIndices,
            typename DerivedDistances>
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

  inline void serialize(
      proto::InvertedMultiIndex* proto_inverted_multi_index) const {
    CHECK_NOTNULL(proto_inverted_multi_index);

    for (const InvFile& inverted_file : inverted_files_) {
      proto::InvertedFile* proto_inverted_file =
          CHECK_NOTNULL(proto_inverted_multi_index->add_inverted_files());

      const size_t num_descriptors = inverted_file.descriptors_.size();
      CHECK_GT(num_descriptors, 0u);

      const size_t projected_descriptor_dimensions =
          static_cast<size_t>(inverted_file.descriptors_[0].rows());

      Eigen::MatrixXf descriptors =
          Eigen::MatrixXf(projected_descriptor_dimensions, num_descriptors);

      for (size_t descriptor_idx = 0u; descriptor_idx < num_descriptors;
           ++descriptor_idx) {
        descriptors.col(descriptor_idx) =
            inverted_file.descriptors_[descriptor_idx];
      }

      ::common::eigen_proto::serialize(
          descriptors, proto_inverted_file->mutable_descriptors());

      const size_t num_indices = inverted_file.indices_.size();
      CHECK_EQ(num_descriptors, num_indices);

      for (const int index : inverted_file.indices_) {
        proto_inverted_file->add_indices(index);
      }
    }

    proto_inverted_multi_index->set_max_db_descriptor_index(
        max_db_descriptor_index_);

    for (const std::pair<int, int>& word_index_element : word_index_map_) {
      proto::InvertedMultiIndex_WordIndexMapEntry* proto_word_index_map_entry =
          CHECK_NOTNULL(proto_inverted_multi_index->add_word_index_map());

      proto_word_index_map_entry->set_visual_word_index(
          word_index_element.first);
      proto_word_index_map_entry->set_inverted_file_index(
          word_index_element.second);
    }
  }

  inline void deserialize(
      const proto::InvertedMultiIndex proto_inverted_multi_index) {
    inverted_files_.clear();

    for (const ::loop_closure::proto::InvertedFile& proto_inverted_file :
         proto_inverted_multi_index.inverted_files()) {
      Eigen::MatrixXf descriptors;
      ::common::eigen_proto::deserialize(
          proto_inverted_file.descriptors(), &descriptors);

      const int projected_descriptor_dimensions = descriptors.rows();
      CHECK_EQ(projected_descriptor_dimensions, 2 * kDimSubVectors);
      const int num_descriptors = descriptors.cols();

      const int num_indices = proto_inverted_file.indices_size();
      CHECK_EQ(num_indices, num_descriptors);

      common::InvertedFile<float, 2 * kDimSubVectors> inverted_file;

      inverted_file.descriptors_.resize(num_indices);
      inverted_file.indices_.resize(num_indices);
      for (int idx = 0; idx < num_indices; ++idx) {
        inverted_file.descriptors_[idx] = descriptors.col(idx);
        inverted_file.indices_[idx] = proto_inverted_file.indices(idx);
      }

      inverted_files_.push_back(inverted_file);
    }

    max_db_descriptor_index_ =
        proto_inverted_multi_index.max_db_descriptor_index();

    word_index_map_.clear();

    for (const ::loop_closure::proto::InvertedMultiIndex_WordIndexMapEntry&
             word_index_map_entry :
         proto_inverted_multi_index.word_index_map()) {
      const int visual_word_index = word_index_map_entry.visual_word_index();
      const int inverted_file_index =
          word_index_map_entry.inverted_file_index();
      word_index_map_.emplace(visual_word_index, inverted_file_index);
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
