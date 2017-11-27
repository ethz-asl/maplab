#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <loopclosure-common/types.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vocabulary-tree/simple-kmeans.h"
#include "vocabulary-tree/tree-builder.h"
#include "vocabulary-tree/types.h"

static const int kDescriptorBytes = 64;

TEST(VocabularyTree, VocabularyTree_BinaryDescriptorsBuilder) {
  std::mt19937 generator(42);
  static const uint32_t K = 4;
  static const uint32_t LEVELS = 3;

  using loop_closure::DescriptorType;

  loop_closure::DescriptorContainer descriptors;
  std::vector<DescriptorType> descriptor_refs;

  descriptors.resize(kDescriptorBytes, 30 * pow(K, LEVELS));

  VLOG(3) << "Testing with " << descriptors.cols() << " descriptors";
  for (int i = 0; i < descriptors.cols(); ++i) {
    DescriptorType descriptor_ref(
        &descriptors(0, i), descriptors.rows(), false);
    descriptor_ref.SetRandom(generator());
    descriptor_refs.push_back(descriptor_ref);
  }

  DescriptorType descriptor_zero(kDescriptorBytes);
  descriptor_zero.SetZero();

  typedef loop_closure::TreeBuilder<
      DescriptorType, loop_closure::distance::Hamming<DescriptorType> >
      TreeBuilder;
  TreeBuilder builder(descriptor_zero);
  builder.kmeans().SetRestarts(5);
  builder.Build(descriptor_refs, K, LEVELS);

  const TreeBuilder::Tree& tree = builder.tree();
  printf("%lu centers\n", tree.centers().size());

  const std::vector<DescriptorType>& centers = tree.centers();

  for (const DescriptorType& center : centers) {
    CHECK_EQ(center.size(), kDescriptorBytes);
  }

  unsigned int bad_assignments = 0;

  std::vector<int> wordcounts;
  wordcounts.resize(tree.centers().size(), 0);

  using loop_closure::distance::Hamming;
  Hamming<DescriptorType> hamming_distance;

  for (int descriptor_idx = 0; descriptor_idx < descriptors.cols();
       ++descriptor_idx) {
    DescriptorType descriptor(
        &descriptors(0, descriptor_idx), descriptors.rows(), false);
    int closest_center = -1;
    unsigned int closest_distance = std::numeric_limits<unsigned int>::max();
    unsigned int second_closest_distance =
        std::numeric_limits<unsigned int>::max();
    for (size_t center_idx = 0; center_idx < centers.size(); ++center_idx) {
      unsigned int distance =
          hamming_distance(descriptor, centers.at(center_idx));
      if (distance < closest_distance) {
        second_closest_distance = closest_distance;
        closest_distance = distance;
        closest_center = center_idx;
      }
    }
    // Check that we don't have a trivial solution.
    EXPECT_NE(second_closest_distance, closest_distance);
    EXPECT_NE(closest_center, -1);

    unsigned int leaf_idx = tree.Quantize(descriptor);
    unsigned int other_distance =
        hamming_distance(descriptor, centers.at(leaf_idx));
    if (static_cast<double>(closest_distance) / other_distance < 0.2) {
      ++bad_assignments;
    }
    ++wordcounts[leaf_idx];
  }
  ASSERT_LT(bad_assignments, 0.01 * descriptors.cols());

  // Check that we haven't assigned all of the descriptors to one word.
  int words_used = 0;
  for (int word : wordcounts) {
    if (word) {
      ++words_used;
    }
  }
  int num_leaves = tree.CountLeaves();
  ASSERT_GT(words_used, num_leaves * 0.5);
}

MAPLAB_UNITTEST_ENTRYPOINT
