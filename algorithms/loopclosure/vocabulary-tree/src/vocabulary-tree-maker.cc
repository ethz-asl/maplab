#include <vocabulary-tree/vocabulary-tree-maker.h>

#include <algorithm>
#include <bitset>
#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <loopclosure-common/types.h>
#include <vi-map/vertex.h>

#include "vocabulary-tree/tree-builder.h"
#include "vocabulary-tree/types.h"

DEFINE_int32(
    lc_kmeans_splits, 10, "Number of splits in the kmeans step per level.");

DEFINE_int32(lc_kmeans_levels, 4, "Number of levels in the vocabulary tree.");

DEFINE_int32(lc_num_kmeans_restarts, 5, "Number of restarts for the kmeans.");

namespace loop_closure {
void trainVocabularyTree(
    const std::vector<std::shared_ptr<vi_map::Vertex> >& vertices,
    typename VocabularyTreeBuilder::Tree* tree) {
  CHECK_NOTNULL(tree);
  tree->Clear();

  using loop_closure::DescriptorType;

  // Make a list of references into the binary feature store for the
  // clusterer to use.
  std::vector<DescriptorType> descriptor_refs;
  constexpr int kExpectedDescriptors = 10000;
  descriptor_refs.reserve(kExpectedDescriptors);

  int descriptor_size = -1;
  for (const std::shared_ptr<vi_map::Vertex>& vertex_ptr : vertices) {
    CHECK(vertex_ptr != nullptr);
    const vi_map::Vertex& vertex = *vertex_ptr;
    const unsigned int num_frames = vertex.numFrames();
    for (unsigned int frame_idx = 0; frame_idx < num_frames; ++frame_idx) {
      const aslam::VisualFrame& visual_frame = vertex.getVisualFrame(frame_idx);
      const loop_closure::DescriptorContainer& descriptors =
          visual_frame.getDescriptors();
      if (descriptors.cols() != 0) {
        descriptor_size = descriptors.rows();
        for (int i = 0; i < descriptors.cols(); ++i) {
          DescriptorType descriptor(
              const_cast<unsigned char*>(&descriptors(0, i)),
              descriptors.rows(), false);
          descriptor_refs.push_back(descriptor);
        }
      }
    }
  }

  VLOG(3) << "Creating tree with " << FLAGS_lc_kmeans_splits << " splits on "
          << FLAGS_lc_kmeans_levels << " levels.";

  // Create tree.

  CHECK(!descriptor_refs.empty());
  DescriptorType descriptor_zero(descriptor_size);
  descriptor_zero.SetZero();
  VocabularyTreeBuilder builder(descriptor_zero);
  builder.kmeans().SetRestarts(FLAGS_lc_num_kmeans_restarts);

  VLOG(3) << "Got " << descriptor_refs.size() << " descriptors to train the "
                                                 "vocabulary tree from.";

  builder.Build(
      descriptor_refs, FLAGS_lc_kmeans_splits, FLAGS_lc_kmeans_levels);
  printf("%lu centers\n", builder.tree().centers().size());
  *tree = builder.tree();
}
}  // namespace loop_closure
