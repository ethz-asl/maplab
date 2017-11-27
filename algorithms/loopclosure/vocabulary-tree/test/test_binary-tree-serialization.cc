#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <loopclosure-common/types.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vocabulary-tree/simple-kmeans.h"
#include "vocabulary-tree/tree-builder.h"

static const int kDescriptorBytes = 64;

TEST(VocabularyTree, VocabularyTreeSerialization) {
  std::mt19937 generator(42);
  static const uint32_t K = 5;
  static const uint32_t LEVELS = 2;

  using loop_closure::DescriptorType;
  std::vector<DescriptorType> descriptor_refs;
  loop_closure::DescriptorContainer descriptors;
  descriptors.resize(kDescriptorBytes, 40 * pow(K, LEVELS));
  for (int i = 0; i < descriptors.cols(); ++i) {
    DescriptorType descriptor_ref(
        &descriptors(0, i), descriptors.rows(), false);
    descriptor_ref.SetRandom(generator());
    descriptor_refs.push_back(descriptor_ref);
  }

  typedef loop_closure::TreeBuilder<
      DescriptorType, loop_closure::distance::Hamming<DescriptorType> >
      TreeBuilder;
  DescriptorType descriptor_zero(kDescriptorBytes);
  descriptor_zero.SetZero();
  TreeBuilder builder(descriptor_zero);
  builder.kmeans().SetRestarts(5);
  builder.Build(descriptor_refs, K, LEVELS);

  const typename TreeBuilder::Tree& tree_save = builder.tree();
  std::string filename = "./binary_vt.tree";
  tree_save.Save(filename);

  ASSERT_GT(tree_save.levels(), static_cast<unsigned int>(0));
  ASSERT_GT(tree_save.splits(), static_cast<unsigned int>(0));
  ASSERT_GT(tree_save.words(), static_cast<unsigned int>(0));

  TreeBuilder::Tree load_tree;

  ASSERT_FALSE(tree_save.IsBinaryEqual(load_tree));

  load_tree.Load(filename);

  ASSERT_TRUE(tree_save.IsBinaryEqual(load_tree));
}

MAPLAB_UNITTEST_ENTRYPOINT
