#ifndef DENSE_RECONSTRUCTION_RESOURCE_UTILS_H_
#define DENSE_RECONSTRUCTION_RESOURCE_UTILS_H_

#include <vi-map/vi-map.h>

DECLARE_bool(overwrite);

namespace dense_reconstruction {

template <typename DataType>
void storeFrameResourceWithOptionalOverwrite(
    const DataType& resource, const unsigned int frame_idx,
    const backend::ResourceType& type, vi_map::Vertex* vertex_ptr,
    vi_map::VIMap* vi_map_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  CHECK_NOTNULL(vi_map_ptr);
  if (vi_map_ptr->hasFrameResource<DataType>(*vertex_ptr, frame_idx, type)) {
    if (FLAGS_overwrite) {
      vi_map_ptr->replaceFrameResource(resource, frame_idx, type, vertex_ptr);
    } else {
      LOG(ERROR) << "Will not overwrite existing resource of type '"
                 << backend::ResourceTypeNames[static_cast<int>(type)]
                 << "' for vertex " << vertex_ptr->id() << " and frame "
                 << frame_idx << "! Use --overwrite.";
    }
  } else {
    vi_map_ptr->storeFrameResource(resource, frame_idx, type, vertex_ptr);
  }
}

// Generate a false color map for a disparity or depth map.
void generateColorMap(const cv::Mat& input_mat, cv::Mat* color_map);

}  // namespace dense_reconstruction

#endif  // DENSE_RECONSTRUCTION_RESOURCE_UTILS_H_
