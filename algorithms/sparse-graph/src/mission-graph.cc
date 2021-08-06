#include "sparse-graph/mission-graph.h"

#include <limits>

#include "sparse-graph/common/utils.h"

namespace spg {

MissionGraph::MissionGraph() {}

void MissionGraph::addNewVertices(
    const uint32_t submap_id, const pose_graph::VertexIdList& vertices) {
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);
  all_vertex_partitions_[submap_id] = vertices;
}

std::size_t MissionGraph::size() const noexcept {
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);

  std::size_t accum_size = 0u;
  for (const auto& id_and_vertices : all_vertex_partitions_) {
    accum_size += id_and_vertices.second.size();
  }
  return accum_size;
}

RepresentativeNodeVector MissionGraph::computeSparseGraph(
    BasePartitioner* partitioner) const {
  CHECK_NOTNULL(partitioner);
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);

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
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);
  if (!containsSubmap(submap_id)) {
    return {};
  }
  return all_vertex_partitions_.at(submap_id);
}

std::size_t MissionGraph::getNumberOfVerticesForId(
    const uint32_t submap_id) const noexcept {
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);
  if (!containsSubmap(submap_id)) {
    return 0u;
  }
  return all_vertex_partitions_.at(submap_id).size();
}

const pose_graph::VertexId MissionGraph::getVertex(
    const uint32_t submap_id, const uint32_t local_id) const noexcept {
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);
  CHECK(containsSubmap(submap_id));
  pose_graph::VertexIdList vertices = all_vertex_partitions_.at(submap_id);
  CHECK(local_id < vertices.size());
  return vertices.at(local_id);
}

uint32_t MissionGraph::getLocalVertexId(
    const uint32_t submap_id, const pose_graph::VertexId& v) const noexcept {
  const std::size_t n_vertices = getNumberOfVerticesForId(submap_id);
  if (n_vertices == 0) {
    return std::numeric_limits<uint32_t>::max();
  }
  const pose_graph::VertexIdList& vertices = getVerticesForId(submap_id);
  auto it = std::find(vertices.begin(), vertices.end(), v);
  if (it == vertices.end()) {
    return std::numeric_limits<uint32_t>::max();
  }
  return std::distance(vertices.begin(), it);
}

bool MissionGraph::containsVertex(
    const uint32_t submap_id, const pose_graph::VertexId& v) const noexcept {
  const std::size_t n_vertices = getNumberOfVerticesForId(submap_id);
  return n_vertices > 0u && getLocalVertexId(submap_id, v) < n_vertices;
}

bool MissionGraph::containsSubmap(const uint32_t submap_id) const noexcept {
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);
  return all_vertex_partitions_.find(submap_id) !=
         all_vertex_partitions_.cend();
}

std::vector<uint32_t> MissionGraph::getAllSubmapIds() const {
  const std::lock_guard<std::recursive_mutex> lock(partition_mutex_);

  std::vector<uint32_t> ids;
  for (const auto& id_and_vertices : all_vertex_partitions_) {
    ids.emplace_back(id_and_vertices.first);
  }
  return ids;
}

vi_map::MissionId MissionGraph::getMissionIdForSubmap(
    const vi_map::VIMap* map, const uint32_t submap_id) const {
  CHECK_NOTNULL(map);
  const pose_graph::VertexIdList vertices = getVerticesForId(submap_id);
  vi_map::MissionIdList mission_ids = Utils::GetMissionIds(map, vertices);
  CHECK_EQ(mission_ids.size(), 1u);
  return mission_ids[0];
}

}  // namespace spg
