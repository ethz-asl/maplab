#include "sparse-graph/partitioners/base-partitioner.h"

#include <glog/logging.h>

#include "sparse-graph/common/sparse-graph-gflags.h"

namespace spg {

bool BasePartitioner::shouldInsertNode(
    const RepresentativeNodeVector& nodes,
    const RepresentativeNode& cur_node) const noexcept {
  if (nodes.empty()) {
    return true;
  }
  const RepresentativeNode& last_node = nodes.back();
  aslam::Transformation T_last_cur_node = last_node.transformTo(cur_node);
  if (T_last_cur_node.getPosition().norm() >
      FLAGS_sparse_graph_min_distance_to_last_node_m) {
    return true;
  }
  if (std::abs(aslam::AngleAxis(T_last_cur_node.getRotation()).angle()) >
      FLAGS_sparse_graph_min_rotation_to_last_node_rad) {
    return true;
  }
  return false;
}

}  // namespace spg
