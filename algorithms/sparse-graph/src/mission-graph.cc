#include "sparse-graph/mission-graph.h"

namespace spg {

MissionGraph::MissionGraph() {}

void MissionGraph::addNewVertices(
    const uint64_t submap_id, const pose_graph::VertexIdList& vertices) {
  all_vertex_partitions_[submap_id] = vertices;
}

std::size_t MissionGraph::size() const noexcept {
  std::size_t accum_size = 0u;
  for (const auto& id_and_vertices : all_vertex_partitions_) {
    accum_size += id_and_vertices.second.size();
  }
  return accum_size;
}

}  // namespace spg
