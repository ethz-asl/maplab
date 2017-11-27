#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <inverted-multi-index/inverted-multi-index-common.h>
#include <inverted-multi-index/inverted-multi-index.h>

namespace loop_closure {
namespace inverted_multi_index {
namespace {
class TestableInvertedMultiIndex : public InvertedMultiIndex<3> {
 public:
  TestableInvertedMultiIndex(
      const Eigen::MatrixXf& words1, const Eigen::MatrixXf& words2,
      int num_closest_words_for_nn_search)
      : InvertedMultiIndex<3>(words1, words2, num_closest_words_for_nn_search) {
  }
  using InvertedMultiIndex<3>::words_1_index_;
  using InvertedMultiIndex<3>::words_2_index_;
  using InvertedMultiIndex<3>::word_index_map_;
  using InvertedMultiIndex<3>::inverted_files_;
  using InvertedMultiIndex<3>::max_db_descriptor_index_;
};

class InvertedMultiIndexTest : public ::testing::Test {
 public:
  void SetUp() {
    words1_.resize(3, 10);
    words1_ << 0.751267059305653, 0.547215529963803, 0.814284826068816,
        0.616044676146639, 0.917193663829810, 0.075854289563064,
        0.568823660872193, 0.311215042044805, 0.689214503140008,
        0.152378018969223, 0.255095115459269, 0.138624442828679,
        0.243524968724989, 0.473288848902729, 0.285839018820374,
        0.053950118666607, 0.469390641058206, 0.528533135506213,
        0.748151592823709, 0.825816977489547, 0.505957051665142,
        0.149294005559057, 0.929263623187228, 0.351659507062997,
        0.757200229110721, 0.530797553008973, 0.011902069501241,
        0.165648729499781, 0.450541598502498, 0.538342435260057;
    words2_.resize(3, 5);
    words2_ << 0.699076722656686, 0.257508254123736, 0.349983765984809,
        0.830828627896291, 0.753729094278495, 0.890903252535798,
        0.840717255983663, 0.196595250431208, 0.585264091152724,
        0.380445846975357, 0.959291425205444, 0.254282178971531,
        0.251083857976031, 0.549723608291140, 0.567821640725221;
  }

  Eigen::MatrixXf words1_;
  Eigen::MatrixXf words2_;
};

TEST_F(InvertedMultiIndexTest, AddDescriptorsWorks) {
  // FLAG used to control the amount of backtracking done during kd-tree-based
  // nearest neighbor search. In order to work, this test needs a certain amount
  // of backtracking.
  FLAGS_lc_knn_epsilon = 0.2;

  Eigen::MatrixXf descriptors(6, 50);
  descriptors << 0.837, 0.298, 0.071, 0.971, 0.205, 0.170, 0.236, 0.043, 0.087,
      0.638, 0.583, 0.727, 0.013, 0.741, 0.088, 0.465, 0.514, 0.948, 0.552,
      0.118, 0.018, 0.719, 0.482, 0.343, 0.599, 0.984, 0.455, 0.568, 0.576,
      0.678, 0.370, 0.506, 0.600, 0.867, 0.888, 0.520, 0.736, 0.227, 0.072,
      0.176, 0.858, 0.497, 0.403, 0.861, 0.921, 0.425, 0.671, 0.998, 0.903,
      0.938, 0.930, 0.721, 0.448, 0.799, 0.284, 0.239, 0.616, 0.298, 0.906,
      0.573, 0.741, 0.156, 0.148, 0.379, 0.706, 0.703, 0.355, 0.351, 0.068,
      0.969, 0.354, 0.875, 0.930, 0.869, 0.565, 0.213, 0.309, 0.106, 0.854,
      0.163, 0.677, 0.443, 0.087, 0.276, 0.981, 0.792, 0.251, 0.918, 0.980,
      0.026, 0.769, 0.473, 0.339, 0.565, 0.326, 0.986, 0.031, 0.662, 0.249,
      0.035, 0.886, 0.901, 0.071, 0.524, 0.733, 0.091, 0.311, 0.473, 0.904,
      0.531, 0.526, 0.849, 0.618, 0.361, 0.899, 0.763, 0.485, 0.668, 0.575,
      0.861, 0.633, 0.427, 0.447, 0.930, 0.124, 0.250, 0.713, 0.405, 0.029,
      0.250, 0.432, 0.096, 0.117, 0.666, 0.898, 0.608, 0.097, 0.468, 0.062,
      0.943, 0.889, 0.583, 0.268, 0.101, 0.953, 0.780, 0.558, 0.219, 0.034,
      0.378, 0.765, 0.044, 0.904, 0.320, 0.570, 0.855, 0.452, 0.344, 0.235,
      0.919, 0.958, 0.052, 0.934, 0.927, 0.830, 0.992, 0.711, 0.888, 0.199,
      0.056, 0.362, 0.465, 0.311, 0.337, 0.567, 0.951, 0.908, 0.336, 0.734,
      0.975, 0.654, 0.448, 0.991, 0.075, 0.128, 0.767, 0.775, 0.565, 0.282,
      0.388, 0.698, 0.721, 0.439, 0.631, 0.733, 0.555, 0.991, 0.568, 0.137,
      0.178, 0.608, 0.342, 0.451, 0.838, 0.624, 0.879, 0.163, 0.829, 0.508,
      0.016, 0.986, 0.644, 0.639, 0.408, 0.485, 0.720, 0.818, 0.689, 0.518,
      0.810, 0.513, 0.134, 0.244, 0.271, 0.473, 0.743, 0.445, 0.259, 0.231,
      0.772, 0.679, 0.817, 0.654, 0.055, 0.155, 0.941, 0.707, 0.637, 0.460,
      0.132, 0.651, 0.524, 0.103, 0.482, 0.090, 0.633, 0.265, 0.352, 0.250,
      0.035, 0.748, 0.238, 0.528, 0.061, 0.257, 0.691, 0.227, 0.143, 0.551,
      0.207, 0.798, 0.442, 0.920, 0.016, 0.275, 0.457, 0.737, 0.763, 0.951,
      0.274, 0.999, 0.092, 0.940, 0.526, 0.270, 0.512, 0.774, 0.466, 0.781,
      0.848, 0.947, 0.413, 0.636, 0.867, 0.813, 0.664, 0.702, 0.687, 0.818,
      0.825, 0.437, 0.302, 0.145, 0.855, 0.363, 0.091, 0.329, 0.044, 0.380,
      0.876;
  std::vector<int> nearest_word_per_descriptor = {
      43, 47, 38, 41, 26, 35, 37, 26, 46, 44, 40, 11, 25, 18, 48, 43, 15,
      23, 0,  46, 25, 42, 44, 47, 32, 3,  4,  2,  34, 5,  45, 31, 8,  22,
      42, 40, 8,  48, 49, 29, 43, 18, 37, 34, 14, 46, 4,  42, 7,  2};
  std::vector<bool> word_used(50, false);
  std::vector<int> word_index(50, -1);
  int counter = 0;
  for (int i = 0; i < 50; ++i) {
    if (!word_used[nearest_word_per_descriptor[i]]) {
      word_index[nearest_word_per_descriptor[i]] = counter;
      word_used[nearest_word_per_descriptor[i]] = true;
      ++counter;
    }
  }

  TestableInvertedMultiIndex index(words1_, words2_, 10);
  index.AddDescriptors(descriptors);
  ASSERT_EQ(50, index.max_db_descriptor_index_);
  ASSERT_EQ(32u, index.word_index_map_.size());
  ASSERT_EQ(32u, index.inverted_files_.size());

  // Ensures that every descriptor is stored in its correct place.
  std::vector<int> word_counts(50, 0);
  for (int i = 0; i < 50; ++i) {
    int word = word_index[nearest_word_per_descriptor[i]];
    ASSERT_GE(word, 0);
    ASSERT_LT(word, 32);
    ASSERT_EQ(
        index.inverted_files_[word].descriptors_.size(),
        index.inverted_files_[word].indices_.size());
    ASSERT_GE(
        static_cast<int>(index.inverted_files_[word].descriptors_.size()),
        word_counts[word]);
    EXPECT_EQ(index.inverted_files_[word].indices_[word_counts[word]], i);
    EXPECT_NEAR_EIGEN(
        index.inverted_files_[word].descriptors_[word_counts[word]],
        descriptors.col(i), 1e-12);
    word_counts[word] += 1;
  }

  index.Clear();
  EXPECT_EQ(0, index.max_db_descriptor_index_);
}

TEST_F(InvertedMultiIndexTest, GetNNearestNeighborsWorks) {
  Eigen::MatrixXf descriptors(6, 50);
  descriptors << 0.837, 0.298, 0.071, 0.971, 0.205, 0.170, 0.236, 0.043, 0.087,
      0.638, 0.583, 0.727, 0.013, 0.741, 0.088, 0.465, 0.514, 0.948, 0.552,
      0.118, 0.018, 0.719, 0.482, 0.343, 0.599, 0.984, 0.455, 0.568, 0.576,
      0.678, 0.370, 0.506, 0.600, 0.867, 0.888, 0.520, 0.736, 0.227, 0.072,
      0.176, 0.858, 0.497, 0.403, 0.861, 0.921, 0.425, 0.671, 0.998, 0.903,
      0.938, 0.930, 0.721, 0.448, 0.799, 0.284, 0.239, 0.616, 0.298, 0.906,
      0.573, 0.741, 0.156, 0.148, 0.379, 0.706, 0.703, 0.355, 0.351, 0.068,
      0.969, 0.354, 0.875, 0.930, 0.869, 0.565, 0.213, 0.309, 0.106, 0.854,
      0.163, 0.677, 0.443, 0.087, 0.276, 0.981, 0.792, 0.251, 0.918, 0.980,
      0.026, 0.769, 0.473, 0.339, 0.565, 0.326, 0.986, 0.031, 0.662, 0.249,
      0.035, 0.886, 0.901, 0.071, 0.524, 0.733, 0.091, 0.311, 0.473, 0.904,
      0.531, 0.526, 0.849, 0.618, 0.361, 0.899, 0.763, 0.485, 0.668, 0.575,
      0.861, 0.633, 0.427, 0.447, 0.930, 0.124, 0.250, 0.713, 0.405, 0.029,
      0.250, 0.432, 0.096, 0.117, 0.666, 0.898, 0.608, 0.097, 0.468, 0.062,
      0.943, 0.889, 0.583, 0.268, 0.101, 0.953, 0.780, 0.558, 0.219, 0.034,
      0.378, 0.765, 0.044, 0.904, 0.320, 0.570, 0.855, 0.452, 0.344, 0.235,
      0.919, 0.958, 0.052, 0.934, 0.927, 0.830, 0.992, 0.711, 0.888, 0.199,
      0.056, 0.362, 0.465, 0.311, 0.337, 0.567, 0.951, 0.908, 0.336, 0.734,
      0.975, 0.654, 0.448, 0.991, 0.075, 0.128, 0.767, 0.775, 0.565, 0.282,
      0.388, 0.698, 0.721, 0.439, 0.631, 0.733, 0.555, 0.991, 0.568, 0.137,
      0.178, 0.608, 0.342, 0.451, 0.838, 0.624, 0.879, 0.163, 0.829, 0.508,
      0.016, 0.986, 0.644, 0.639, 0.408, 0.485, 0.720, 0.818, 0.689, 0.518,
      0.810, 0.513, 0.134, 0.244, 0.271, 0.473, 0.743, 0.445, 0.259, 0.231,
      0.772, 0.679, 0.817, 0.654, 0.055, 0.155, 0.941, 0.707, 0.637, 0.460,
      0.132, 0.651, 0.524, 0.103, 0.482, 0.090, 0.633, 0.265, 0.352, 0.250,
      0.035, 0.748, 0.238, 0.528, 0.061, 0.257, 0.691, 0.227, 0.143, 0.551,
      0.207, 0.798, 0.442, 0.920, 0.016, 0.275, 0.457, 0.737, 0.763, 0.951,
      0.274, 0.999, 0.092, 0.940, 0.526, 0.270, 0.512, 0.774, 0.466, 0.781,
      0.848, 0.947, 0.413, 0.636, 0.867, 0.813, 0.664, 0.702, 0.687, 0.818,
      0.825, 0.437, 0.302, 0.145, 0.855, 0.363, 0.091, 0.329, 0.044, 0.380,
      0.876;
  std::vector<int> nearest_word_per_descriptor = {
      43, 47, 38, 41, 26, 35, 37, 26, 46, 44, 40, 11, 25, 18, 48, 43, 15,
      23, 0,  46, 25, 42, 44, 47, 32, 3,  4,  2,  34, 5,  45, 31, 8,  22,
      42, 40, 8,  48, 49, 29, 43, 18, 37, 34, 14, 46, 4,  42, 7,  2};

  Eigen::MatrixXf query_descriptors(6, 10);
  query_descriptors << 0.971, 0.890, 0.610, 0.509, 0.017, 0.922, 0.901, 0.323,
      0.321, 0.745, 0.581, 0.179, 0.900, 0.622, 0.827, 0.945, 0.020, 0.921,
      0.409, 0.737, 0.369, 0.800, 0.027, 0.497, 0.493, 0.556, 0.168, 0.705,
      0.107, 0.012, 0.913, 0.912, 0.232, 0.611, 0.513, 0.625, 0.543, 0.639,
      0.716, 0.584, 0.638, 0.791, 0.944, 0.111, 0.226, 0.626, 0.105, 0.086,
      0.834, 0.696, 0.470, 0.813, 0.640, 0.236, 0.182, 0.292, 0.039, 0.230,
      0.707, 0.006;

  TestableInvertedMultiIndex index(words1_, words2_, 10);
  index.AddDescriptors(descriptors);

  for (int i = 0; i < 10; ++i) {
    std::vector<std::pair<int, int> > ten_closest_words;
    common::FindClosestWords<3>(
        query_descriptors.col(i), 10, *(index.words_1_index_),
        *(index.words_2_index_), words1_.cols(), words2_.cols(),
        &ten_closest_words);
    std::vector<bool> word_activated(50, 0);
    for (int j = 0; j < 10; ++j) {
      int word_index = ten_closest_words[j].first * words2_.cols() +
                       ten_closest_words[j].second;
      word_activated[word_index] = true;
    }

    static constexpr int kNumNeighbors = 10;
    Eigen::VectorXi indices(kNumNeighbors, 1);
    Eigen::VectorXf distances(kNumNeighbors, 1);
    index.GetNNearestNeighbors(
        query_descriptors.block<6, 1>(0, i), kNumNeighbors, indices, distances);

    // Verifies that the nearest neighbors are correct through linear search.
    std::vector<std::pair<float, int> > gt_distances;
    int num_neighbors = 0;
    for (int j = 0; j < 50; ++j) {
      if (!word_activated[nearest_word_per_descriptor[j]])
        continue;

      float d = (descriptors.col(j) - query_descriptors.col(i)).squaredNorm();
      gt_distances.push_back(std::make_pair(d, j));
      ++num_neighbors;
    }
    std::sort(gt_distances.begin(), gt_distances.end());

    Eigen::VectorXi expected_indices(kNumNeighbors, 1);
    int num_elements = std::min(kNumNeighbors, num_neighbors);
    for (int j = 0; j < num_elements; ++j) {
      EXPECT_FLOAT_EQ(gt_distances[j].first, distances[j]);
      expected_indices(j, 0) = gt_distances[j].second;
    }
    EXPECT_TRUE(
        ::common::MatricesEqual(
            indices.block(0, 0, num_elements, 1),
            expected_indices.block(0, 0, num_elements, 1), 1e-9));
  }
}
}  // namespace
}  // namespace inverted_multi_index
}  // namespace loop_closure

MAPLAB_UNITTEST_ENTRYPOINT
