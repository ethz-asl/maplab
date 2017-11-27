#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <loopclosure-common/types.h>
#include <maplab-common/feature-descriptor-ref.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vocabulary-tree/simple-kmeans.h"
#include "vocabulary-tree/types.h"

static const int kDescriptorBytes = 64;

TEST(VocabularyTree, SimpleKMeans_BinaryClustering) {
  std::mt19937 generator(40);
  static const size_t K = 10;

  loop_closure::DescriptorContainer descriptors;

  descriptors.resize(kDescriptorBytes, 50 * K);

  using loop_closure::DescriptorType;
  typedef std::vector<DescriptorType> FeatureVector;
  std::shared_ptr<FeatureVector> centers(new FeatureVector);
  std::vector<unsigned int> membership;

  std::vector<DescriptorType> descriptor_refs;
  for (int i = 0; i < descriptors.cols(); ++i) {
    DescriptorType descriptor_ref(
        &descriptors(0, i), descriptors.rows(), false);
    descriptor_ref.SetRandom(generator());
    descriptor_refs.push_back(descriptor_ref);
  }

  DescriptorType descriptor_zero(kDescriptorBytes);
  descriptor_zero.SetZero();

  loop_closure::SimpleKmeans<DescriptorType,
                             loop_closure::distance::Hamming<DescriptorType> >
      kmeans(descriptor_zero);
  kmeans.Cluster(descriptor_refs, K, generator(), &membership, &centers);

  std::vector<unsigned int> membercnt;
  membercnt.resize(centers->size(), 0);
  for (unsigned int member : membership) {
    ++membercnt[member];
  }
  for (unsigned int count : membercnt) {
    EXPECT_NE(count, static_cast<unsigned int>(0));
  }

  loop_closure::distance::Hamming<DescriptorType> hamming_distance;

  for (int descriptor_idx = 0; descriptor_idx < descriptors.cols();
       ++descriptor_idx) {
    DescriptorType descriptor(
        &descriptors(0, descriptor_idx), descriptors.rows(), false);
    int closest_center = -1;
    unsigned int closest_distance = std::numeric_limits<unsigned int>::max();
    unsigned int second_closest_distance =
        std::numeric_limits<unsigned int>::max();
    for (size_t center_idx = 0; center_idx < centers->size(); ++center_idx) {
      unsigned int distance =
          hamming_distance(descriptor, centers->at(center_idx));
      if (distance < closest_distance) {
        second_closest_distance = closest_distance;
        closest_distance = distance;
        closest_center = center_idx;
      }
    }
    // Check that we don't have a trivial solution.
    EXPECT_NE(second_closest_distance, closest_distance);
    EXPECT_NE(closest_center, -1);
    unsigned int dist_closest_center =
        hamming_distance(descriptor, centers->at(membership[descriptor_idx]));
    EXPECT_LE(dist_closest_center, closest_distance + kDescriptorBytes * 2);
  }
}

MAPLAB_UNITTEST_ENTRYPOINT
