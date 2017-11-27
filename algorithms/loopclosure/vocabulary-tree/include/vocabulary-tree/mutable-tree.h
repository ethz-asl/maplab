// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_MUTABLE_TREE_H_
#define VOCABULARY_TREE_MUTABLE_TREE_H_

#include <vector>

#include <vocabulary-tree/vocabulary-tree.h>

namespace loop_closure {

// Vocabulary tree that exposes the hierarchical clustering centers. Mainly
// intended for building a new tree.
//
// When loading and using an existing vocabulary tree, use VocabularyTree
// instead.
template <class Feature, class Distance = loop_closure::distance::L2<Feature>,
          class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class MutableVocabularyTree
    : public loop_closure::VocabularyTree<Feature, Distance, FeatureAllocator> {
  typedef loop_closure::VocabularyTree<Feature, Distance, FeatureAllocator>
      BaseClass;

 public:
  MutableVocabularyTree(Distance d = Distance()) : BaseClass(d) {}

  void SetSize(uint32_t levels, uint32_t splits) {
    this->levels_ = levels;
    this->num_splits_ = splits;
    this->SetNodeCounts();
  }

  uint32_t nodes() const {
    return this->word_start_ + this->num_words_;
  }

  uint32_t wordstart() const {
    return this->word_start_;
  }

  std::vector<Feature, FeatureAllocator>& centers() {
    return this->centers_;
  }
  const std::vector<Feature, FeatureAllocator>& centers() const {
    return this->centers_;
  }

  std::vector<uint8_t>& validCenters() {
    return this->valid_centers_;
  }
  const std::vector<uint8_t>& validCenters() const {
    return this->valid_centers_;
  }
};

}  // namespace loop_closure
#endif  // VOCABULARY_TREE_MUTABLE_TREE_H_
