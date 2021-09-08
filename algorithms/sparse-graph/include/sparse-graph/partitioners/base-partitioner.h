#ifndef SPARSE_GRAPH_PARTITIONERS_BASE_PARTITIONER_H_
#define SPARSE_GRAPH_PARTITIONERS_BASE_PARTITIONER_H_

#include <glog/logging.h>

#include <vi-map/vi-map.h>

#include "sparse-graph/common/representative-node.h"

namespace spg {

class BasePartitioner {
 public:
  explicit BasePartitioner(const vi_map::VIMap& map) : map_(map) {}

  virtual ~BasePartitioner() = default;

  virtual RepresentativeNodeVector getRepresentativesForSubmap(
      const pose_graph::VertexIdList& vertices, const uint64_t submap_id) = 0;

 protected:
  bool shouldInsertNode(
      const RepresentativeNodeVector& nodes,
      const RepresentativeNode& cur_node) const noexcept;

  const vi_map::VIMap& map_;
};

}  // namespace spg

#endif  // SPARSE_GRAPH_PARTITIONERS_BASE_PARTITIONER_H_
