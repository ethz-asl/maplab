#include "sparse-graph/mission-graph.h"

namespace spg {

MissionGraph::MissionGraph() {}

void MissionGraph::addNewVertices(const pose_graph::VertexIdList& vertices) {
  all_vertex_partitions_.emplace_back(vertices);
}

std::size_t MissionGraph::size() const noexcept {
  std::size_t accum_size = 0u;
  for (pose_graph::VertexIdList vertices : all_vertex_partitions_) {
    accum_size += vertices.size();
  }
  return accum_size;
}

}  // namespace spg
