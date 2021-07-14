#ifndef SPARSE_GRAPH_PARTITIONERS_ALL_PARTITIONER_H_
#define SPARSE_GRAPH_PARTITIONERS_ALL_PARTITIONER_H_

#include "sparse-graph/partitioners/base-partitioner.h"

namespace spg {

class AllPartitioner : public BasePartitioner {
 public:
  explicit AllPartitioner(const vi_map::VIMap& map);
  virtual ~AllPartitioner() = default;
  RepresentativeNodeVector getRepresentativesForSubmap(
      const pose_graph::VertexIdList& vertices,
      const uint64_t submap_id) override;
};

}  // namespace spg

#endif  // SPARSE_GRAPH_PARTITIONERS_ALL_PARTITIONER_H_
