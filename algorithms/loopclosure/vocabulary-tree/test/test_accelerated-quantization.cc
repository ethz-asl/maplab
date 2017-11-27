#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <maplab-common/test/testing-entrypoint.h>
#include <vocabulary-tree/distance.h>
#include <vocabulary-tree/simple-kmeans.h>
#include <vocabulary-tree/tree-builder.h>
#include <vocabulary-tree/types.h>

#include "./floating-point-test-helpers.h"

// This tests checks the accelerated search on the kd-tree accelerated
// vocabulary.
TEST(VocabularyTree, VocabularyTree_AcceleratedQuantization) {
  std::mt19937 generator(40);
  static const size_t kNumfeaturesPerCluster = 10;
  static const size_t kNumClusters = 1000;
  static const size_t kNumClustersTree = 10;
  static const size_t kNumLevels = 5;
  DescriptorVector gt_centers;
  DescriptorVector descriptors;
  std::vector<unsigned int> membership;
  std::vector<unsigned int> gt_membership;

  GenerateTestData(
      kNumfeaturesPerCluster, kNumClusters, generator(), &gt_centers,
      &descriptors, &membership, &gt_membership);

  DescriptorType descriptor_zero;
  descriptor_zero.setConstant(
      kDescriptorDimensionality, 1, static_cast<Scalar>(0));

  typedef loop_closure::distance::L2<DescriptorType> DistanceType;
  typedef loop_closure::TreeBuilder<DescriptorType, DistanceType> TreeBuilder;

  TreeBuilder builder(descriptor_zero);
  builder.kmeans().SetRestarts(2);
  builder.Build(descriptors, kNumClustersTree, kNumLevels);

  const TreeBuilder::Tree& tree = builder.tree();
  const DescriptorVector& centers = tree.centers();
  const std::vector<uint8_t>& valid_centers = tree.validCenters();
  const int splits = tree.splits();

  loop_closure::distance::L2<DescriptorType> l2_distance;

  for (const DescriptorType& feature : descriptors) {
    int32_t index = -1;  // Virtual root index, which has no associated center.
    for (unsigned level = 0; level < kNumLevels; ++level) {
      // Calculate the offset to the first child of the current index.
      int32_t first_child = (index + 1) * splits;
      // Find the child center closest to the query.
      int32_t best_child = first_child;
      typename DistanceType::result_type best_distance =
          std::numeric_limits<typename DistanceType::result_type>::max();
      for (int32_t child = first_child; child < first_child + splits; ++child) {
        if (!valid_centers[child])
          break;  // Fewer than splits() children.
        typename DistanceType::result_type child_distance =
            l2_distance(feature, centers[child]);
        if (child_distance < best_distance) {
          best_child = child;
          best_distance = child_distance;
        }
      }
      index = best_child;
    }
    loop_closure::Word word_gt = index - tree.wordstart();
    loop_closure::Word word = tree.Quantize(feature);
    EXPECT_EQ(word_gt, word);
  }
}

MAPLAB_UNITTEST_ENTRYPOINT
