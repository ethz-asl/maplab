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

  virtual RepresentativeNode getRepresentativesForSubmap(
      const pose_graph::VertexIdList& vertices) = 0;

 protected:
  const vi_map::VIMap& map_;
};

}  // namespace spg

#endif  // SPARSE_GRAPH_PARTITIONERS_BASE_PARTITIONER_H_
