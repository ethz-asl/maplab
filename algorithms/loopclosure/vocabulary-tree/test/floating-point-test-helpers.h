#ifndef VOCABULARY_TREE_FLOATING_POINT_TEST_HELPERS_H_
#define VOCABULARY_TREE_FLOATING_POINT_TEST_HELPERS_H_
#include <cstdio>
#include <random>
#include <vector>

typedef float Scalar;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> DescriptorType;
static const int kDescriptorDimensionality = 10;
typedef Aligned<std::vector, DescriptorType> DescriptorVector;

void GenerateTestData(
    size_t kNumfeaturesPerCluster, size_t kNumClusters, size_t seed,
    DescriptorVector* const gt_centers, DescriptorVector* const descriptors,
    std::vector<unsigned int>* const membership,
    std::vector<unsigned int>* const gt_membership) {
  CHECK_NOTNULL(gt_centers);
  CHECK_NOTNULL(descriptors);
  CHECK_NOTNULL(membership);
  CHECK_NOTNULL(gt_membership);
  std::mt19937 generator(seed);

  double lower_bound = -1.0;
  double upper_bound = 1.0;
  std::uniform_real_distribution<double> double_generator(
      lower_bound, upper_bound);
  std::default_random_engine engine;

  for (size_t i = 0; i < kNumClusters; ++i) {
    DescriptorType center;
    center.resize(kDescriptorDimensionality, Eigen::NoChange);
    center.setConstant(i);
    center((1 * i) % kDescriptorDimensionality, 0) += 5;
    center((2 * i) % kDescriptorDimensionality, 0) -= 5;
    center((3 * i) % kDescriptorDimensionality, 0) += 5;
    center((4 * i) % kDescriptorDimensionality, 0) -= 5;
    for (int j = 0; j < kDescriptorDimensionality; ++j) {
      center(j, 0) += double_generator(engine);
    }
    gt_centers->push_back(center);

    for (size_t j = 0; j < kNumfeaturesPerCluster; ++j) {
      DescriptorType sample = center;
      for (int k = 0; k < kDescriptorDimensionality; ++k) {
        sample(k, 0) += double_generator(engine);
      }
      descriptors->push_back(sample);
      gt_membership->push_back(i);
    }
  }
}
#endif  // VOCABULARY_TREE_FLOATING_POINT_TEST_HELPERS_H_
