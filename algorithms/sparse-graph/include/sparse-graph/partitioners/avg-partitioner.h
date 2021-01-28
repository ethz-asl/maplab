#ifndef SPARSE_GRAPH_PARTITIONERS_AVG_PARTITIONER_H_
#define SPARSE_GRAPH_PARTITIONERS_AVG_PARTITIONER_H_

#include "sparse-graph/partitioners/base-partitioner.h"

namespace spg {

class AvgPartitioner : public BasePartitioner {
 public:
  explicit AvgPartitioner(const vi_map::VIMap& map);
  virtual ~AvgPartitioner() = default;
  RepresentativeNode getRepresentativesForSubmap(
      const pose_graph::VertexIdList& vertices) override;
};

}  // namespace spg

#endif  // SPARSE_GRAPH_PARTITIONERS_AVG_PARTITIONER_H_
