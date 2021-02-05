#include "sparse-graph/mission-graph.h"

namespace spg {

MissionGraph::MissionGraph() {}

void MissionGraph::addNewVertices(
    const uint32_t submap_id, const pose_graph::VertexIdList& vertices) {
  all_vertex_partitions_[submap_id] = vertices;
}

std::size_t MissionGraph::size() const noexcept {
  std::size_t accum_size = 0u;
  for (const auto& id_and_vertices : all_vertex_partitions_) {
    accum_size += id_and_vertices.second.size();
  }
  return accum_size;
}
RepresentativeNodeVector MissionGraph::computeSparseGraph(
    BasePartitioner* partitioner) const {
  CHECK_NOTNULL(partitioner);
  RepresentativeNodeVector subgraph;
  for (const auto& id_and_vertices : all_vertex_partitions_) {
    RepresentativeNodeVector nodes = partitioner->getRepresentativesForSubmap(
        id_and_vertices.second, id_and_vertices.first);
    subgraph.insert(subgraph.end(), nodes.begin(), nodes.end());
  }
  return subgraph;
}

const pose_graph::VertexIdList& MissionGraph::getVerticesForId(
    const uint32_t submap_id) const noexcept {
  CHECK(containsSubmap(submap_id));
  return all_vertex_partitions_.at(submap_id);
}

bool MissionGraph::containsSubmap(const uint32_t submap_id) const noexcept {
  return all_vertex_partitions_.find(submap_id) != all_vertex_partitions_.end();
}

}  // namespace spg
