#include "vi-map-helpers/vi-map-nearest-neighbor-lookup.h"

#include <glog/logging.h>

namespace vi_map_helpers {

template <>
void VIMapNearestNeighborLookup<aslam::Position3D,
                                pose_graph::VertexId>::buildIndex() {
  LOG_IF(WARNING, nn_index_)
      << "The already existing k-d tree index will be overwritten.";
  map_.getAllVertexIds(&data_items_);
  const size_t num_vertices_in_map = data_items_.size();
  if (num_vertices_in_map == 0u) {
    LOG(WARNING) << "Aborting building nearest-neighbor index because the "
                 << "given map contains zero vertices.";
    return;
  }

  data_items_.reserve(num_vertices_in_map);
  constexpr size_t kVertexPositionDimensions = 3u;
  nn_index_data_ =
      Eigen::MatrixXd(kVertexPositionDimensions, num_vertices_in_map);

  int col_idx = 0;
  for (const pose_graph::VertexId& vertex_id : data_items_) {
    CHECK(vertex_id.isValid());

    const pose::Transformation vertex_T_G_I = map_.getVertex_T_G_I(vertex_id);
    CHECK_LT(col_idx, nn_index_data_.cols());
    const Eigen::VectorXd data_item_as_vector =
        queryTypeToVector(vertex_T_G_I.getPosition());
    CHECK_EQ(data_item_as_vector.rows(), nn_index_data_.rows());
    nn_index_data_.col(col_idx) = data_item_as_vector;
    ++col_idx;
  }

  nn_index_.reset(
      Nabo::NNSearchD::createKDTreeLinearHeap(
          nn_index_data_, kVertexPositionDimensions));
  CHECK(nn_index_ != nullptr);
  CHECK_EQ(data_items_.size(), num_vertices_in_map);
  CHECK_EQ(num_vertices_in_map, static_cast<size_t>(nn_index_data_.cols()));
}

}  // namespace vi_map_helpers
