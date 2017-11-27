#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <maplab-common/test/testing-entrypoint.h>

#include "vocabulary-tree/distance.h"
#include "vocabulary-tree/simple-kmeans.h"
#include "vocabulary-tree/tree-builder.h"
#include "vocabulary-tree/types.h"

#include "./floating-point-test-helpers.h"

// This tests checks the kNearestNeighbor search on the kd-tree top level.
TEST(BucketizedTree, KNearestNeighbors) {
  std::mt19937 generator(40);
  static const size_t kNumfeaturesPerCluster = 20;
  static const size_t kNumClusters = 100;
  static const size_t kNumLevels = 1;

  FLAGS_lc_kdtree_accelerator_eps = 0.2;

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
  builder.kmeans().SetRestarts(5);
  builder.Build(descriptors, kNumClusters, kNumLevels);

  const TreeBuilder::Tree& tree = builder.tree();
  const DescriptorVector& centers = tree.centers();

  loop_closure::distance::L2<DescriptorType> l2_distance;

  // Search all descriptors, compute nearest neighbors.
  unsigned int num_nearest_neighbors = 4;
  for (const DescriptorType& feature : descriptors) {
    typedef std::pair<int, typename DistanceType::result_type> Candidate;
    std::vector<Candidate> candidates;
    for (size_t i = 0; i < centers.size(); ++i) {
      const DescriptorType& center = centers[i];
      double distance = l2_distance(feature, center);
      candidates.emplace_back(i, distance);
    }
    std::sort(
        candidates.begin(), candidates.end(),
        [](const Candidate& lhs, const Candidate& rhs) {
          return lhs.second < rhs.second;
        });

    std::vector<loop_closure::Word> nearest_neighbors;
    std::vector<typename DistanceType::result_type> distances;
    tree.GetNearestNeighborTopLevel(
        feature, num_nearest_neighbors, &nearest_neighbors, &distances);

    EXPECT_EQ(nearest_neighbors.size(), num_nearest_neighbors);
    EXPECT_EQ(nearest_neighbors.size(), distances.size());

    for (unsigned int i = 0; i < num_nearest_neighbors; ++i) {
      EXPECT_EQ(nearest_neighbors[i], candidates[i].first);
      EXPECT_NEAR(distances[i], candidates[i].second, 1e-4);
    }
  }
}

MAPLAB_UNITTEST_ENTRYPOINT
