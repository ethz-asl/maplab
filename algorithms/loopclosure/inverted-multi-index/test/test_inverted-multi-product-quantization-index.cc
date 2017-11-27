#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <inverted-multi-index/inverted-multi-index-common.h>
#include <inverted-multi-index/inverted-multi-product-quantization-index.h>

namespace loop_closure {
namespace inverted_multi_index {
namespace {
class TestableInvertedMultiPQIndex
    : public InvertedMultiProductQuantizationIndex<int, 4, 1, 2> {
 public:
  TestableInvertedMultiPQIndex(
      const Eigen::MatrixXf& words1, const Eigen::MatrixXf& words2,
      const Eigen::MatrixXf& quantizer_centers_1,
      const Eigen::MatrixXf& quantizer_centers_2,
      int num_closest_words_for_nn_search)
      : InvertedMultiProductQuantizationIndex<int, 4, 1, 2>(
            words1, words2, quantizer_centers_1, quantizer_centers_2,
            num_closest_words_for_nn_search) {}

  using InvertedMultiProductQuantizationIndex<int, 4, 1, 2>::words_1_index_;
  using InvertedMultiProductQuantizationIndex<int, 4, 1, 2>::words_2_index_;
  using InvertedMultiProductQuantizationIndex<int, 4, 1, 2>::word_index_map_;
  using InvertedMultiProductQuantizationIndex<int, 4, 1, 2>::inverted_files_;
  using InvertedMultiProductQuantizationIndex<int, 4, 1,
                                              2>::max_db_descriptor_index_;
};

class InvertedMultiProductQuantizationIndexTest : public ::testing::Test {
 public:
  void SetUp() {
    descriptors_.resize(4, 5);
    // The descriptors are stored column-wise and the corresponding indices of
    // the closest word from the product vocabulary are 0, 0, 14, 5, 11.
    descriptors_ << 2.5, 1.8, -2.1, 0.5, -1.0, 2.5, 2.2, 1.8, -0.5, -1.0, 0.1,
        0.3, 0.05, 1.0, -2.5, 1.9, 2.6, -2.4, 0.1, 0.5;

    words1_.resize(2, 4);
    words1_ << 2.0, 2.0, -2.0, -2.0, 2.0, -2.0, -2.0, 2.0;
    words2_.resize(2, 4);
    words2_ << 0.0, 2.0, 0.0, -2.0, 2.0, 0.0, -2.0, 0.0;

    // For simplicity, use the same cluster centers for each product quantizer.
    quantizer_centers_1_.resize(1, 16);
    quantizer_centers_1_ << -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0,
        1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0;
    quantizer_centers_2_.resize(1, 16);
    quantizer_centers_2_ << -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5,
        0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5;

    // FLAG used to control the amount of backtracking done during kd-tree-based
    // nearest neighbor search. In order to work, this test needs a certain
    // amount of backtracking.
    FLAGS_lc_knn_epsilon = 0.2;
  }

  Eigen::MatrixXf descriptors_;
  Eigen::MatrixXf words1_;
  Eigen::MatrixXf words2_;
  Eigen::MatrixXf quantizer_centers_1_;
  Eigen::MatrixXf quantizer_centers_2_;
};

using ::testing::ContainerEq;

TEST_F(InvertedMultiProductQuantizationIndexTest, AddDescriptorsWorks) {
  TestableInvertedMultiPQIndex index(
      words1_, words2_, quantizer_centers_1_, quantizer_centers_2_, 16);
  index.AddDescriptors(descriptors_);

  // Tests whether the map used to index the inverted files is correct.
  std::vector<int> activated_product_words = {0, 5, 11, 14};
  std::vector<int> expected_map_entries = {0, 2, 3, 1};
  for (size_t i = 0; i < activated_product_words.size(); ++i) {
    ASSERT_NE(
        index.word_index_map_.find(activated_product_words[i]),
        index.word_index_map_.end());
    EXPECT_EQ(
        expected_map_entries[i],
        index.word_index_map_.find(activated_product_words[i])->second);
  }

  // Tests whether the descriptors are quantized and stored correctly.
  Aligned<std::vector, Eigen::Matrix<int, 4, 1> >
      expected_quantized_descriptors(5);
  expected_quantized_descriptors[0] << 1, 1, 1, 0;
  expected_quantized_descriptors[1] << 0, 1, 1, 1;
  expected_quantized_descriptors[2] << 0, 0, 1, 0;
  expected_quantized_descriptors[3] << 0, 1, 0, 1;
  expected_quantized_descriptors[4] << 1, 1, 0, 1;

  std::vector<int> expected_num_entries_per_inverted_file = {2, 1, 1, 1};

  ASSERT_EQ(4, index.inverted_files_.size());
  int counter = 0;
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(
        index.inverted_files_[i].descriptors_.size(),
        index.inverted_files_[i].indices_.size());
    ASSERT_EQ(
        expected_num_entries_per_inverted_file[i],
        index.inverted_files_[i].descriptors_.size());
    for (int j = 0; j < expected_num_entries_per_inverted_file[i]; ++j) {
      EXPECT_EQ(counter, index.inverted_files_[i].indices_[j]);
      EXPECT_TRUE(
          ::common::MatricesEqual(
              expected_quantized_descriptors[counter],
              index.inverted_files_[i].descriptors_[j]))
          << "The quantized representation for descriptor " << counter << " ( "
          << index.inverted_files_[i].descriptors_[j].transpose()
          << " ) does not match the expected quantized representation ( "
          << expected_quantized_descriptors[counter].transpose() << " ).";
      ++counter;
    }
  }

  // After filling the index, tests whether the index gets cleared correctly.
  index.Clear();
  EXPECT_EQ(0, index.max_db_descriptor_index_);
  EXPECT_TRUE(index.word_index_map_.empty());
  EXPECT_TRUE(index.inverted_files_.empty());
}

TEST_F(InvertedMultiProductQuantizationIndexTest, GetNNearestNeighborsWorks) {
  TestableInvertedMultiPQIndex index(
      words1_, words2_, quantizer_centers_1_, quantizer_centers_2_, 16);
  index.AddDescriptors(descriptors_);

  Eigen::Matrix<float, 4, 1> query_descriptor;
  query_descriptor << 1.0, 0.5, 0.5, 1.0;

  static constexpr int kNumNeighbors = 8;
  Eigen::VectorXi indices(kNumNeighbors, 1);
  Eigen::VectorXf distances(kNumNeighbors, 1);
  index.GetNNearestNeighbors(
      query_descriptor, kNumNeighbors, indices, distances);

  constexpr float infinity = std::numeric_limits<float>::infinity();
  std::vector<int> expected_indices = {3, 1, 0, 4, 2, -1, -1, -1};
  std::vector<float> expected_distances = {3.5,  8.5,      10.5,     15.5,
                                           28.5, infinity, infinity, infinity};

  std::vector<int> actual_indices;
  std::vector<float> actual_distances;

  for (int i = 0; i < kNumNeighbors; ++i) {
    actual_distances.push_back(distances[i]);
    actual_indices.push_back(indices[i]);
  }

  EXPECT_THAT(actual_distances, ContainerEq(expected_distances));
  EXPECT_THAT(actual_indices, ContainerEq(expected_indices));
}
}  // namespace
}  // namespace inverted_multi_index
}  // namespace loop_closure

MAPLAB_MOCKTEST_ENTRYPOINT
