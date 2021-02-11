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

std::size_t MissionGraph::getNumberOfVerticesForId(
    const uint32_t submap_id) const noexcept {
  CHECK(containsSubmap(submap_id));
  return all_vertex_partitions_.at(submap_id).size();
}

const pose_graph::VertexId MissionGraph::getVertex(
    const uint32_t submap_id, const uint32_t local_id) const noexcept {
  CHECK(containsSubmap(submap_id));
  pose_graph::VertexIdList vertices = all_vertex_partitions_.at(submap_id);
  CHECK(local_id < vertices.size());
  return vertices.at(local_id);
}

uint32_t MissionGraph::getLocalVertexId(
    const uint32_t submap_id, const pose_graph::VertexId& v) const noexcept {
  CHECK(containsSubmap(submap_id));
  const pose_graph::VertexIdList& vertices = getVerticesForId(submap_id);
  auto it = std::find(vertices.begin(), vertices.end(), v);
  if (it == vertices.end()) {
    return vertices.size();
  }
  return std::distance(vertices.begin(), it);
}

bool MissionGraph::containsVertex(
    const uint32_t submap_id, const pose_graph::VertexId& v) const noexcept {
  return getLocalVertexId(submap_id, v) < getNumberOfVerticesForId(submap_id);
}

bool MissionGraph::containsSubmap(const uint32_t submap_id) const noexcept {
  return all_vertex_partitions_.find(submap_id) != all_vertex_partitions_.end();
}

}  // namespace spg
