#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <nabo/nabo.h>

#include <inverted-multi-index/inverted-multi-index-common.h>
#include <inverted-multi-index/inverted-multi-index.h>

namespace loop_closure {
namespace inverted_multi_index {
namespace common {

using ::testing::ContainerEq;

TEST(InvertedMultiIndexCommonTest, InsertNeighborWorks) {
  std::vector<std::pair<float, int> > neighbors;
  InsertNeighbor(5, 0.1, 5, &neighbors);
  InsertNeighbor(2, 0.2, 5, &neighbors);
  InsertNeighbor(1, 0.2, 5, &neighbors);
  InsertNeighbor(50, 5.0, 5, &neighbors);
  InsertNeighbor(3, 0.05, 5, &neighbors);
  InsertNeighbor(10, 9.0, 5, &neighbors);
  InsertNeighbor(4, 4.0, 5, &neighbors);
  InsertNeighbor(6, 0.21, 5, &neighbors);
  InsertNeighbor(8, 0.11, 5, &neighbors);

  std::vector<std::pair<float, int> > expected_neighbors = {
      std::make_pair(0.05, 3), std::make_pair(0.1, 5), std::make_pair(0.11, 8),
      std::make_pair(0.2, 1), std::make_pair(0.2, 2)};
  EXPECT_THAT(neighbors, ContainerEq(expected_neighbors));

  neighbors.clear();
  InsertNeighbor(31, 0.349399, 10, &neighbors);
  InsertNeighbor(3, 0.409949, 10, &neighbors);
  InsertNeighbor(24, 0.183158, 10, &neighbors);
  InsertNeighbor(28, 0.881889, 10, &neighbors);
  InsertNeighbor(43, 0.819767, 10, &neighbors);
  InsertNeighbor(21, 0.529346, 10, &neighbors);
  InsertNeighbor(34, 2.01685, 10, &neighbors);
  InsertNeighbor(47, 0.232519, 10, &neighbors);
  InsertNeighbor(0, 1.40066, 10, &neighbors);
  InsertNeighbor(15, 1.014, 10, &neighbors);
  InsertNeighbor(40, 0.983704, 10, &neighbors);

  std::vector<std::pair<float, int> > expected_neighbors2 = {
      std::make_pair(0.183158, 24), std::make_pair(0.232519, 47),
      std::make_pair(0.349399, 31), std::make_pair(0.409949, 3),
      std::make_pair(0.529346, 21), std::make_pair(0.819767, 43),
      std::make_pair(0.881889, 28), std::make_pair(0.983704, 40),
      std::make_pair(1.014, 15),    std::make_pair(1.40066, 0)};
  EXPECT_THAT(neighbors, ContainerEq(expected_neighbors2));
}

TEST(InvertedMultiIndexCommonTest, MultiSequenceAlgorithmWorks) {
  Eigen::VectorXf distances_1(6);
  Eigen::VectorXi indices_1(6);
  Eigen::VectorXf distances_2(4);
  Eigen::VectorXi indices_2(4);

  distances_1 << 0.0, 0.1, 1.0, 2.5, 3.0, 3.1;
  indices_1 << 1, 5, 2, 4, 3, 0;

  distances_2 << 0.3, 10.2, 10.20000001, 10.20000002;
  indices_2 << 0, 3, 2, 1;

  std::vector<std::pair<int, int> > expected_closest_words = {
      std::make_pair(1, 0), std::make_pair(5, 0), std::make_pair(2, 0),
      std::make_pair(4, 0), std::make_pair(3, 0), std::make_pair(0, 0),
      std::make_pair(1, 3), std::make_pair(1, 2), std::make_pair(1, 1),
      std::make_pair(5, 3), std::make_pair(5, 2), std::make_pair(5, 1),
      std::make_pair(2, 3), std::make_pair(2, 2), std::make_pair(2, 1),
      std::make_pair(4, 3), std::make_pair(4, 2), std::make_pair(4, 1),
      std::make_pair(3, 3), std::make_pair(3, 2), std::make_pair(3, 1),
      std::make_pair(0, 3), std::make_pair(0, 2), std::make_pair(0, 1)};

  std::vector<std::pair<int, int> > closest_words;
  MultiSequenceAlgorithm(
      indices_1, distances_1, indices_2, distances_2, 24, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words));

  expected_closest_words.resize(10);
  closest_words.clear();
  MultiSequenceAlgorithm(
      indices_1, distances_1, indices_2, distances_2, 10, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words));

  expected_closest_words.resize(5);
  closest_words.clear();
  MultiSequenceAlgorithm(
      indices_1, distances_1, indices_2, distances_2, 5, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words));

  expected_closest_words.resize(3);
  closest_words.clear();
  MultiSequenceAlgorithm(
      indices_1, distances_1, indices_2, distances_2, 3, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words));

  expected_closest_words.resize(2);
  closest_words.clear();
  MultiSequenceAlgorithm(
      indices_1, distances_1, indices_2, distances_2, 2, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words));

  expected_closest_words.resize(1);
  closest_words.clear();
  MultiSequenceAlgorithm(
      indices_1, distances_1, indices_2, distances_2, 1, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words));

  expected_closest_words.resize(0);
  MultiSequenceAlgorithm(
      indices_1, distances_1, indices_2, distances_2, 0, &closest_words);
  EXPECT_TRUE(closest_words.empty());
}

TEST(InvertedMultiIndexCommonTest, FindClosestWordsWorks) {
  Eigen::MatrixXf words1(3, 10);
  words1 << 0.751267059305653, 0.547215529963803, 0.814284826068816,
      0.616044676146639, 0.917193663829810, 0.075854289563064,
      0.568823660872193, 0.311215042044805, 0.689214503140008,
      0.152378018969223, 0.255095115459269, 0.138624442828679,
      0.243524968724989, 0.473288848902729, 0.285839018820374,
      0.053950118666607, 0.469390641058206, 0.528533135506213,
      0.748151592823709, 0.825816977489547, 0.505957051665142,
      0.149294005559057, 0.929263623187228, 0.351659507062997,
      0.757200229110721, 0.530797553008973, 0.011902069501241,
      0.165648729499781, 0.450541598502498, 0.538342435260057;
  Eigen::MatrixXf words2(3, 5);
  words2 << 0.699076722656686, 0.257508254123736, 0.349983765984809,
      0.830828627896291, 0.753729094278495, 0.890903252535798,
      0.840717255983663, 0.196595250431208, 0.585264091152724,
      0.380445846975357, 0.959291425205444, 0.254282178971531,
      0.251083857976031, 0.549723608291140, 0.567821640725221;

  // Constructs two kd-trees.
  std::shared_ptr<NNSearch> words_1_tree(
      NNSearch::createKDTreeLinearHeap(words1, 3, kCollectTouchStatistics));
  std::shared_ptr<NNSearch> words_2_tree(
      NNSearch::createKDTreeLinearHeap(words2, 3, kCollectTouchStatistics));

  Eigen::Matrix<float, 6, 1> query1;
  query1 << 0.899623768879227, 0.580171289251844, 0.713341306219580,
      0.843687251006573, 0.688287018946517, 0.162469693724691;
  std::vector<std::pair<int, int> > expected_closest_words1 = {
      std::make_pair(4, 3), std::make_pair(8, 3), std::make_pair(2, 3),
      std::make_pair(0, 3), std::make_pair(4, 4), std::make_pair(3, 3),
      std::make_pair(8, 4), std::make_pair(2, 4), std::make_pair(0, 4),
      std::make_pair(4, 1), std::make_pair(3, 4), std::make_pair(8, 1),
      std::make_pair(2, 1), std::make_pair(0, 1), std::make_pair(4, 2)};
  std::vector<std::pair<int, int> > closest_words;
  FindClosestWords<3>(
      query1, 15, *words_1_tree, *words_2_tree, 10, 5, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words1));

  Eigen::Matrix<float, 6, 1> query2;
  query2 << 0.140397592757592, 0.345575701658016, 0.812736932727134,
      0.004961049760113, 0.237733728973631, 0.031866037439546;
  std::vector<std::pair<int, int> > expected_closest_words2 = {
      std::make_pair(5, 2), std::make_pair(9, 2), std::make_pair(3, 2),
      std::make_pair(0, 2), std::make_pair(5, 1), std::make_pair(2, 2),
      std::make_pair(7, 2), std::make_pair(8, 2), std::make_pair(4, 2),
      std::make_pair(9, 1), std::make_pair(1, 2), std::make_pair(3, 1),
      std::make_pair(0, 1), std::make_pair(2, 1), std::make_pair(7, 1),
      std::make_pair(6, 2), std::make_pair(5, 4), std::make_pair(8, 1),
      std::make_pair(4, 1), std::make_pair(1, 1), std::make_pair(9, 4),
      std::make_pair(5, 3), std::make_pair(6, 1), std::make_pair(3, 4),
      std::make_pair(0, 4), std::make_pair(2, 4), std::make_pair(7, 4),
      std::make_pair(9, 3), std::make_pair(8, 4), std::make_pair(4, 4),
      std::make_pair(1, 4), std::make_pair(3, 3), std::make_pair(0, 3),
      std::make_pair(2, 3), std::make_pair(7, 3), std::make_pair(8, 3),
      std::make_pair(4, 3), std::make_pair(6, 4), std::make_pair(1, 3),
      std::make_pair(6, 3), std::make_pair(5, 0), std::make_pair(9, 0),
      std::make_pair(3, 0), std::make_pair(0, 0), std::make_pair(2, 0),
      std::make_pair(7, 0), std::make_pair(8, 0), std::make_pair(4, 0),
      std::make_pair(1, 0), std::make_pair(6, 0)};
  FindClosestWords<3>(
      query2, 200, *words_1_tree, *words_2_tree, 10, 5, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words2));

  Eigen::Matrix<float, 6, 1> query3;
  query3 << 0.676942673662204, 0.851611155877681, 0.769413386896462,
      0.576447128449685, 0.268798395903650, 0.916118918355307;
  std::vector<std::pair<int, int> > expected_closest_words3 = {
      std::make_pair(8, 4), std::make_pair(8, 3), std::make_pair(3, 4)};
  FindClosestWords<3>(
      query3, 3, *words_1_tree, *words_2_tree, 10, 5, &closest_words);
  EXPECT_THAT(closest_words, ContainerEq(expected_closest_words3));
}

TEST(InvertedMultiIndexCommonTest, AddDescriptorWorks) {
  std::unordered_map<int, int> word_index_map;
  Aligned<std::vector, InvertedFile<float, 6> > inverted_files;

  Eigen::Matrix<float, 6, 5> descriptors;
  descriptors << 0.837, 0.298, 0.071, 0.971, 0.205, 0.170, 0.236, 0.043, 0.087,
      0.638, 0.583, 0.727, 0.013, 0.741, 0.088, 0.465, 0.514, 0.948, 0.552,
      0.118, 0.018, 0.719, 0.482, 0.343, 0.599, 0.984, 0.455, 0.568, 0.576,
      0.678;
  std::vector<int> word_index_per_descriptor = {43, 38, 43, 26, 38};
  std::vector<int> descriptor_ids = {19, 5, 17, 6, 4};

  for (int i = 0; i < 5; ++i) {
    AddDescriptor<float, 6>(
        descriptors.col(i), descriptor_ids[i], word_index_per_descriptor[i],
        &word_index_map, &inverted_files);
  }

  ASSERT_EQ(3, word_index_map.size());
  std::vector<int> expected_word_indices = {26, 38, 43};
  std::vector<int> expected_word_index_mapped_values = {2, 1, 0};
  for (int i = 0; i < 3; ++i) {
    ASSERT_NE(
        word_index_map.find(expected_word_indices[i]), word_index_map.end());
    EXPECT_EQ(
        expected_word_index_mapped_values[i],
        word_index_map.find(expected_word_indices[i])->second);
  }
  ASSERT_EQ(3, inverted_files.size());
  std::vector<std::vector<int> > expected_indices = {{19, 17}, {5, 4}, {6}};
  std::vector<Eigen::MatrixXf> expected_descriptors(3);
  expected_descriptors[0].resize(6, 2);
  expected_descriptors[0].col(0) = descriptors.col(0);
  expected_descriptors[0].col(1) = descriptors.col(2);
  expected_descriptors[1].resize(6, 2);
  expected_descriptors[1].col(0) = descriptors.col(1);
  expected_descriptors[1].col(1) = descriptors.col(4);
  expected_descriptors[2].resize(6, 1);
  expected_descriptors[2].col(0) = descriptors.col(3);

  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(
        inverted_files[i].indices_.size(),
        inverted_files[i].descriptors_.size());
    EXPECT_THAT(inverted_files[i].indices_, ContainerEq(expected_indices[i]));
    Eigen::MatrixXf stored_descriptors;
    stored_descriptors.resize(6, inverted_files[i].descriptors_.size());
    for (size_t j = 0; j < inverted_files[i].descriptors_.size(); ++j) {
      stored_descriptors.col(j) = inverted_files[i].descriptors_[j];
    }
    EXPECT_TRUE(
        gtest_catkin::MatricesEqual(
            expected_descriptors[i], stored_descriptors));
  }
}

}  // namespace common
}  // namespace inverted_multi_index
}  // namespace loop_closure

MAPLAB_MOCKTEST_ENTRYPOINT
