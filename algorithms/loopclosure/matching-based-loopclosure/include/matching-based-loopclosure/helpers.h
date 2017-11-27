#ifndef MATCHING_BASED_LOOPCLOSURE_HELPERS_H_
#define MATCHING_BASED_LOOPCLOSURE_HELPERS_H_
#include <vector>

#include <Eigen/Core>
#include <aslam/common/timer.h>
#include <descriptor-projection/descriptor-projection.h>
#include <loopclosure-common/types.h>

namespace loop_closure {
namespace internal {
// Zero copy passing of Eigen-Block Expressions.
// http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
template <typename Derived>
inline Eigen::MatrixBase<Derived>& CastConstEigenMatrixToNonConst(
    const Eigen::MatrixBase<Derived>& value) {
  return const_cast<Eigen::MatrixBase<Derived>&>(value);
}

inline void ProjectDescriptors(
    const DescriptorContainer& descriptors,
    const Eigen::MatrixXf& projection_matrix, int target_dimensionality,
    Eigen::MatrixXf* projected_descriptors) {
  CHECK_NOTNULL(projected_descriptors);
  projected_descriptors->resize(target_dimensionality, descriptors.cols());

  timing::Timer timer_proj("Loop Closure: Project descriptors");
  descriptor_projection::ProjectDescriptorBlock(
      descriptors, projection_matrix, target_dimensionality,
      projected_descriptors);
  timer_proj.Stop();
}

inline void ProjectDescriptors(
    const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
    const Eigen::MatrixXf& projection_matrix, int target_dimensionality,
    Eigen::MatrixXf* projected_descriptors) {
  CHECK_NOTNULL(projected_descriptors);
  projected_descriptors->resize(target_dimensionality, descriptors.size());

  timing::Timer timer_proj("Loop Closure: Project descriptors");
  descriptor_projection::ProjectDescriptorBlock(
      descriptors, projection_matrix, target_dimensionality,
      projected_descriptors);
  timer_proj.Stop();
}

}  // namespace internal

template <typename IdType>
inline size_t getNumberOfMatches(
    const loop_closure::IdToMatches<IdType>& id_to_matches) {
  size_t num_matches = 0u;
  for (const typename loop_closure::IdToMatches<IdType>::value_type&
           id_and_matches : id_to_matches) {
    num_matches += id_and_matches.second.size();
  }
  return num_matches;
}

}  // namespace loop_closure

namespace matching_based_loopclosure {
inline bool doProjectedImagesBelongToSameVertex(
    const loop_closure::ProjectedImagePtrList& projected_image_ptr_list) {
  CHECK(!projected_image_ptr_list.empty());

  pose_graph::VertexId query_vertex_id;
  for (loop_closure::ProjectedImagePtrList::const_iterator it_projected_image =
           projected_image_ptr_list.begin();
       it_projected_image != projected_image_ptr_list.end();
       ++it_projected_image) {
    CHECK(*it_projected_image);
    const loop_closure::ProjectedImage& projected_image = **it_projected_image;
    if (it_projected_image == projected_image_ptr_list.begin()) {
      query_vertex_id = projected_image.keyframe_id.vertex_id;
      CHECK(query_vertex_id.isValid());
      continue;
    }
    if (query_vertex_id != projected_image.keyframe_id.vertex_id) {
      return false;
    }
  }
  return true;
}

}  // namespace matching_based_loopclosure

#endif  // MATCHING_BASED_LOOPCLOSURE_HELPERS_H_
