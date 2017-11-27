// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_TREE_BUILDER_H_
#define VOCABULARY_TREE_TREE_BUILDER_H_

#include <deque>
#include <vector>

#include <vocabulary-tree/mutable-tree.h>
#include <vocabulary-tree/simple-kmeans.h>

namespace loop_closure {

// Class for building a new vocabulary by hierarchically clustering
// a set of training features.
template <class Feature, class Distance = distance::L2<Feature>,
          class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class TreeBuilder {
 public:
  typedef MutableVocabularyTree<Feature, Distance, FeatureAllocator> Tree;
  typedef SimpleKmeans<Feature, Distance, FeatureAllocator> Kmeans;
  typedef std::vector<Feature, FeatureAllocator> FeatureVector;

  // - zero Object representing zero in the feature space
  // - d    Functor for calculating squared distance
  TreeBuilder(const Feature& zero = Feature(), Distance d = Distance());

  // Build a new vocabulary tree.
  // The number of words in the resulting vocabulary is at most k ^ levels.
  // - training_features The set of training features to cluster.
  // - k                 The branching factor, or max children of any node.
  // - levels            The number of levels in the tree.
  void Build(
      const FeatureVector& training_features, uint32_t k, uint32_t levels);

  // Get the built vocabulary tree.
  const Tree& tree() const {
    return tree_;
  }

  // Get the k-means clusterer.
  Kmeans& kmeans() {
    return kmeans_;
  }

  // Get the k-means clusterer.
  const Kmeans& kmeans() const {
    return kmeans_;
  }

 protected:
  Tree tree_;
  Kmeans kmeans_;
  Feature zero_;
  Distance distance_;
};
}  // namespace loop_closure

#include "impl/tree-builder-inl.h"
#endif  // VOCABULARY_TREE_TREE_BUILDER_H_
