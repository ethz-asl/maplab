// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_VOCABULARY_TREE_H_
#define VOCABULARY_TREE_VOCABULARY_TREE_H_

#include <cassert>
#include <fstream>  // NOLINT
#include <limits>
#include <stdexcept>
#include <stdint.h>
#include <string>
#include <vector>

#include <vocabulary-tree/distance.h>
#include <vocabulary-tree/feature-allocator.h>
#include <vocabulary-tree/simple-kmeans.h>
#include <vocabulary-tree/types.h>

namespace loop_closure {
// Optimized vocabulary tree quantizer, templated on feature type and distance
// metric for maximum efficiency.
// - Feature is the data type of one feature. It has no requirements except
//   compatibility with the distance metric.
//
// - Distance is a functor that computes the distance between two Feature
//   objects. It must have a \c result_type typedef specifying the type of the
//   returned distance. For the purposes of
// VocabularyTree, this need not even be
// a metric; distances simply need to be comparable.
//
// - FeatureAllocator is an STL-compatible allocator used to allocate Features
//   internally.
template <class Feature, class Distance = distance::L2<Feature>,
          class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class VocabularyTree {
 public:
  typedef typename Distance::result_type DistanceType;
  enum { kSerializationVersion = 2 };

  // - d Functor for computing the distance between two features.
  VocabularyTree(Distance d = Distance());

  // - file Saved vocabulary file.
  // - d    Functor for computing the distance between two features.
  VocabularyTree(const std::string& file, Distance d = Distance());

  // Quantizes a feature into a discrete word.
  Word Quantize(const Feature& f) const;

  // Gets the n nearest buckets.
  void GetNearestNeighborTopLevel(
      const Feature& f, unsigned int num_nearest_neighbors,
      std::vector<Word>* nearest_neighbors,
      std::vector<DistanceType>* distances) const;

  void GetNearestNeighborTopLevel(
      const Eigen::MatrixXf& features, unsigned int num_nearest_neighbors,
      std::vector<Word>* nearest_neighbors) const;

  // Get the depth (number of levels) of the tree.
  uint32_t levels() const;
  // Get the branching factor (max splits at each node) of the tree.
  uint32_t splits() const;
  // Get the number of words the tree contains.
  uint32_t words() const;

  // Count the leaves.
  uint32_t CountLeaves() const;

  // Clears vocabulary, leaving an empty tree.
  void Clear();

  // Save vocabulary to a file.
  void Save(const std::string& file) const;
  // Load vocabulary from a file.
  bool Load(const std::string& file);

  // Save vocabulary to a file.
  void Save(std::ofstream* out_stream) const;
  // Load vocabulary from a file.
  bool Load(std::ifstream* in_stream);

  bool IsBinaryEqual(
      const VocabularyTree<Feature, Distance, FeatureAllocator>& other) const;

  void SetNodeCounts();

 protected:
  typedef typename Distance::result_type distance_type;

  std::vector<Feature, FeatureAllocator> centers_;
  std::vector<uint8_t> valid_centers_;
  Distance distance_;
  typedef
      typename GetSearchAccelerator<Feature, Distance, FeatureAllocator>::type
          SearchAccelerator;
  std::vector<SearchAccelerator> search_accelerators_;

  uint32_t num_splits_;  // Splits, or branching factor.
  uint32_t levels_;
  uint32_t num_words_;  // Number of leaf nodes.
  // Number of non-leaf nodes, or offset to the first leaf node.
  uint32_t word_start_;

  void InitializeSearchAccelerators();

  bool Initialized() const {
    return num_words_ != 0;
  }
};
}  // namespace loop_closure

#include "impl/vocabulary-tree-inl.h"
#endif  // VOCABULARY_TREE_VOCABULARY_TREE_H_
