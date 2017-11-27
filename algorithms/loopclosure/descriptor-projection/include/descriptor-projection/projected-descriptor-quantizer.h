#ifndef DESCRIPTOR_PROJECTION_PROJECTED_DESCRIPTOR_QUANTIZER_H_
#define DESCRIPTOR_PROJECTION_PROJECTED_DESCRIPTOR_QUANTIZER_H_
#include <vector>

#include <descriptor-projection/descriptor-projection.h>
#include <vocabulary-tree/mutable-tree.h>
#include <vocabulary-tree/types.h>

namespace descriptor_projection {
struct ProjectedDescriptorQuantizer {
  explicit ProjectedDescriptorQuantizer(int target_dimensionality)
      : initialized_(false), target_dimensionality_(target_dimensionality) {}
  void Save(std::ofstream* out_stream) const;
  bool Load(std::ifstream* in_stream);

  inline loop_closure::Word ProjectAndQuantize(
      const aslam::common::FeatureDescriptorConstRef& descriptor) const {
    CHECK(initialized_);
    descriptor_projection::ProjectedDescriptorType projected_descriptor;
    projected_descriptor.resize(target_dimensionality_, Eigen::NoChange);
    descriptor_projection::ProjectDescriptor(
        descriptor, projection_matrix_, target_dimensionality_,
        projected_descriptor);
    return vocabulary_.Quantize(projected_descriptor);
  }

  template <typename DerivedIn>
  inline loop_closure::Word Quantize(
      const Eigen::MatrixBase<DerivedIn>& projected_descriptor) const {
    CHECK(initialized_);
    CHECK_EQ(projected_descriptor.rows(), target_dimensionality_);
    return vocabulary_.Quantize(projected_descriptor);
  }

  inline void GetNearestNeighbors(
      const aslam::common::FeatureDescriptorConstRef& descriptor,
      int num_neighbors, std::vector<loop_closure::Word>* nearest_neighbors,
      std::vector<float>* distances) const {
    CHECK_NOTNULL(nearest_neighbors);
    CHECK_NOTNULL(distances);
    CHECK(initialized_);
    descriptor_projection::ProjectedDescriptorType projected_descriptor;
    descriptor_projection::ProjectDescriptor(
        descriptor, projection_matrix_, target_dimensionality_,
        projected_descriptor);
    vocabulary_.GetNearestNeighborTopLevel(
        projected_descriptor, num_neighbors, nearest_neighbors, distances);
  }

  enum { kSerializationVersion = 100 };

  typedef loop_closure::MutableVocabularyTree<
      descriptor_projection::ProjectedDescriptorType,
      loop_closure::distance::L2<
          descriptor_projection::ProjectedDescriptorType> >
      Vocabulary;
  Vocabulary vocabulary_;
  Eigen::MatrixXf projection_matrix_;
  bool initialized_;
  int target_dimensionality_;
};
}  // namespace descriptor_projection
#endif  // DESCRIPTOR_PROJECTION_PROJECTED_DESCRIPTOR_QUANTIZER_H_
