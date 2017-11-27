#ifndef INVERTED_MULTI_INDEX_INVERTED_MULTI_PRODUCT_QUANTIZATION_INDEX_H_
#define INVERTED_MULTI_INDEX_INVERTED_MULTI_PRODUCT_QUANTIZATION_INDEX_H_

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
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nabo/nabo.h>
#include <product-quantization/product-quantization.h>

#include <inverted-multi-index/inverted-multi-index-common.h>

DECLARE_double(lc_knn_max_radius);

namespace loop_closure {
namespace inverted_multi_index {
// Variant of the inverted multi-index, which stores the product quantized
// versions of the original descriptors to save memory and accelerate the search
// for large datasets. The index uses one product quantizer for each word from
// each lower dimensional vocabulary.
// The template parameters are the data type used to store the product quantized
// descriptors, the number of components used for product quantization, the
// number of dimension used for each component, and the number of cluster
// centers used for each product quantizer (see also
// product-quantization/include/product-quantization.h for a more detailed
// explanation of the parameters). The inverted multi-index splits descriptors
// into two parts of dimension kNumComponents / 2 * kNumDimPerComp each. Thus,
// kNumComponents needs to be a multiple of 2.
template <typename DataType, int kNumComponents, int kNumDimPerComp,
          int kNumCenters>
class InvertedMultiProductQuantizationIndex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // The dimensionality of the original descriptors.
  static constexpr int kOriginalDescDim = kNumDimPerComp * kNumComponents;
  static constexpr int kHalfNumComponents = kNumComponents / 2;
  // The type of the input descriptors.
  typedef Eigen::Matrix<float, kOriginalDescDim, 1> InputDescriptorType;
  typedef Eigen::Matrix<float, kOriginalDescDim, Eigen::Dynamic>
      InputDescriptorMatrixType;
  typedef Eigen::Matrix<float, kOriginalDescDim / 2, 1> HalfInputDescriptorType;
  // The type of the quantized descriptors that is stored.
  typedef Eigen::Matrix<DataType, kNumComponents, 1> StoredDescriptorType;

  typedef common::InvertedFile<DataType, kNumComponents> InvFile;
  typedef product_quantization::ProductQuantization<
      kHalfNumComponents, kNumDimPerComp, kNumCenters, DataType>
      ProductQuantizer;

  // Creates the index from a given set of visual words. Each column in words_i
  // specifies a cluster center coordinate. quantizer_centers_i define the
  // cluster centers used by the product quantizer for the i-th lower
  // dimensional vocabulary. quantizer_centers_i is required to have size
  // kNumDimPerComp / 2 x (kNumComponents * kNumCenters * words_i.cols()).
  // The first kNumComponents * kNumCenters columns of quantizer_centers_i
  // define the cluster centers of the product quantizer used for the 1st word
  // in the i-th vocabulary. The next kNumComponents * kNumCenters columns give
  // the centers for the second quantizer, etc.
  InvertedMultiProductQuantizationIndex(
      const Eigen::MatrixXf& words_1, const Eigen::MatrixXf& words_2,
      const Eigen::MatrixXf& quantizer_centers_1,
      const Eigen::MatrixXf& quantizer_centers_2,
      int num_closest_words_for_nn_search)
      : words_1_(words_1),
        words_2_(words_2),
        words_1_index_(
            common::NNSearch::createKDTreeLinearHeap(
                words_1_, kOriginalDescDim / 2,
                common::kCollectTouchStatistics)),
        words_2_index_(
            common::NNSearch::createKDTreeLinearHeap(
                words_2_, kOriginalDescDim / 2,
                common::kCollectTouchStatistics)),
        num_closest_words_for_nn_search_(num_closest_words_for_nn_search),
        max_db_descriptor_index_(0) {
    static_assert(
        kNumComponents % 2 == 0,
        "The number of components needs to be a multiple of 2.");

    CHECK_EQ(words_1.rows(), kOriginalDescDim / 2);
    CHECK_GT(words_1.cols(), 0);
    CHECK_EQ(words_2.rows(), kOriginalDescDim / 2);
    CHECK_GT(words_2.cols(), 0);

    // The number of columns corresponding to the cluster centers of a product
    // quantizer (pq).
    const int num_cols_per_pq = kHalfNumComponents * kNumCenters;

    // The number of components per product quantizer is half the total number
    // of components since each product quantizer is used to quantize one half
    // of the original descriptor.
    CHECK_EQ(quantizer_centers_1.rows(), kNumDimPerComp);
    CHECK_EQ(quantizer_centers_2.rows(), kNumDimPerComp);
    CHECK_EQ(quantizer_centers_1.cols(), num_cols_per_pq * words_1.cols());
    CHECK_EQ(quantizer_centers_1.cols(), num_cols_per_pq * words_1.cols());

    CHECK_GT(num_closest_words_for_nn_search_, 0);

    quantizers_words_1_.resize(words_1.cols());
    int index = 0;
    for (int i = 0; i < words_1.cols(); ++i, index += num_cols_per_pq) {
      quantizers_words_1_[i].SetClusterCenters(
          quantizer_centers_1.block<kNumDimPerComp, num_cols_per_pq>(0, index));
    }

    quantizers_words_2_.resize(words_2.cols());
    index = 0;
    for (int i = 0; i < words_1.cols(); ++i, index += num_cols_per_pq) {
      quantizers_words_2_[i].SetClusterCenters(
          quantizer_centers_2.block<kNumDimPerComp, num_cols_per_pq>(0, index));
    }
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
  void AddDescriptors(const InputDescriptorMatrixType& descriptors) {
    const int num_descriptors = descriptors.cols();
    std::vector<std::pair<int, int> > closest_word;

    for (int i = 0; i < num_descriptors; ++i) {
      common::FindClosestWords<kOriginalDescDim / 2>(
          descriptors.col(i), 1, *words_1_index_, *words_2_index_,
          words_1_.cols(), words_2_.cols(), &closest_word);
      CHECK(!closest_word.empty());
      const int word1 = closest_word[0].first;
      const int word2 = closest_word[0].second;
      const int word_index = word1 * words_2_.cols() + word2;

      // Quantizes the descriptor using the product quantizers belonging to the
      // product word: We first compute the residual between the descriptor and
      // the cluster center before quantizing the residual.
      HalfInputDescriptorType residual_part_1;
      ComputeResidual(
          descriptors.col(i).template head<kOriginalDescDim / 2>(), words_1_,
          word1, &residual_part_1);
      HalfInputDescriptorType residual_part_2;
      ComputeResidual(
          descriptors.col(i).template tail<kOriginalDescDim / 2>(), words_2_,
          word2, &residual_part_2);

      StoredDescriptorType quantized_residual;
      Eigen::Matrix<DataType, kHalfNumComponents, 1> quantized_part;
      quantizers_words_1_[closest_word[0].first].Quantize(
          residual_part_1, &quantized_part);
      quantized_residual.template head<kHalfNumComponents>() = quantized_part;
      quantizers_words_2_[closest_word[0].second].Quantize(
          residual_part_2, &quantized_part);
      quantized_residual.template tail<kHalfNumComponents>() = quantized_part;

      common::AddDescriptor<DataType, kNumComponents>(
          quantized_residual, max_db_descriptor_index_, word_index,
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
        DerivedQuery, static_cast<int>(kOriginalDescDim), 1);
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
    common::FindClosestWords<kOriginalDescDim / 2>(
        query_feature, num_closest_words_for_nn_search_, *words_1_index_,
        *words_2_index_, words_1_.cols(), words_2_.cols(), &closest_words);

    // Performs exhaustive search through all descriptors assigned to the
    // closest words, using product quantization to compute the distances.
    std::vector<std::pair<float, int> > nearest_neighbors;
    nearest_neighbors.reserve(num_neighbors + 1);

    const int num_words_to_use = static_cast<int>(closest_words.size());
    std::unordered_map<int, int>::const_iterator word_index_map_it;

    // In order to avoid re-computing the distances between the descriptor and
    // the cluster centers of a product quantizer (which are stored in look-up
    // tables), we cache them independently for each of the two lower
    // dimensional vocabularies.
    typedef Eigen::Matrix<float, kHalfNumComponents, kNumCenters> LookUpTable;
    AlignedUnorderedMap<int, LookUpTable> table_cache_words_1;
    AlignedUnorderedMap<int, LookUpTable> table_cache_words_2;

    for (int i = 0; i < num_words_to_use; ++i) {
      const int word1 = closest_words[i].first;
      const int word2 = closest_words[i].second;
      const int word_index = word1 * words_2_.cols() + word2;
      word_index_map_it = word_index_map_.find(word_index);
      if (word_index_map_it == word_index_map_.end())
        continue;

      // Computes the look-up table for the product quantizer corresponding to
      // the word in the first vocabulary.
      typename AlignedUnorderedMap<int, LookUpTable>::iterator table_it =
          table_cache_words_1.find(word1);
      if (table_it == table_cache_words_1.end()) {
        HalfInputDescriptorType residual_part;
        ComputeResidual(
            query_feature.template head<kOriginalDescDim / 2>(), words_1_,
            word1, &residual_part);
        LookUpTable lut;
        quantizers_words_1_[word1].FillLUT(residual_part, &lut);
        table_it = table_cache_words_1.insert(std::make_pair(word1, lut)).first;
      }
      const LookUpTable& lut1 = table_it->second;

      // Computes the look-up table for the product quantizer corresponding to
      // the word in the first vocabulary.
      table_it = table_cache_words_2.find(word2);
      if (table_it == table_cache_words_2.end()) {
        HalfInputDescriptorType residual_part;
        ComputeResidual(
            query_feature.template tail<kOriginalDescDim / 2>(), words_2_,
            word2, &residual_part);
        LookUpTable lut;
        quantizers_words_2_[word2].FillLUT(residual_part, &lut);
        table_it = table_cache_words_2.insert(std::make_pair(word2, lut)).first;
      }
      const LookUpTable& lut2 = table_it->second;

      const InvFile& inverted_file = inverted_files_[word_index_map_it->second];
      const int num_descriptors =
          static_cast<int>(inverted_file.descriptors_.size());
      for (int j = 0; j < num_descriptors; ++j) {
        // Computes the distance between the query and the quantized descriptors
        // using the look-up tables.
        float distance = quantizers_words_1_[word1].ComputeDistance(
            lut1,
            inverted_file.descriptors_[j].template head<kHalfNumComponents>());
        distance += quantizers_words_2_[word2].ComputeDistance(
            lut2,
            inverted_file.descriptors_[j].template tail<kHalfNumComponents>());
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
  // Given a half of a original descriptor, a vocabulary, and the word from this
  // vocabulary that is closest to the half, computes between residual the
  // descriptor and the cluster center of the word.
  // For performance reasons, we do not CHECK_NOTNULL as both AddDescriptors and
  // GetNNearestNeighbors guarantee that the pointer is not null.
  // Similar, both functions guarantee that word is valid.
  inline void ComputeResidual(
      const HalfInputDescriptorType& descriptor_half,
      const Eigen::MatrixXf& words, int word,
      HalfInputDescriptorType* residual) const {
    *residual = descriptor_half - words.col(word);
  }

  // The two sets of cluster centers defining the quantization of the descriptor
  // space as the Cartesian product of the two sets of words.
  Eigen::MatrixXf words_1_;
  Eigen::MatrixXf words_2_;
  std::shared_ptr<common::NNSearch> words_1_index_;
  std::shared_ptr<common::NNSearch> words_2_index_;

  // A set of product quantizers, one for each word from the lower dimensional
  // vocabularies.
  Aligned<std::vector, ProductQuantizer> quantizers_words_1_;
  Aligned<std::vector, ProductQuantizer> quantizers_words_2_;

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

#endif  // INVERTED_MULTI_INDEX_INVERTED_MULTI_PRODUCT_QUANTIZATION_INDEX_H_
