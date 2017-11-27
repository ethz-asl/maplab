#ifndef VOCABULARY_TREE_VOCABULARY_TREE_MAKER_H_
#define VOCABULARY_TREE_VOCABULARY_TREE_MAKER_H_

#include <memory>
#include <vector>

#include <loopclosure-common/types.h>
#include <vocabulary-tree/tree-builder.h>

namespace vi_map {
class Vertex;
}  // namespace vi_map

namespace loop_closure {

typedef loop_closure::TreeBuilder<
    DescriptorType, loop_closure::distance::Hamming<DescriptorType> >
    VocabularyTreeBuilder;

void trainVocabularyTree(
    const std::vector<std::shared_ptr<vi_map::Vertex> >& vertices,
    typename VocabularyTreeBuilder::Tree* tree);
}  // namespace loop_closure

#endif  // VOCABULARY_TREE_VOCABULARY_TREE_MAKER_H_
