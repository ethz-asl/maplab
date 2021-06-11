#ifndef SPARSE_GRAPH_MISSION_GRAPH_H_
#define SPARSE_GRAPH_MISSION_GRAPH_H_

#include <unordered_map>
#include <vector>

#include <vi-map/vi-map.h>

#include "sparse-graph/common/representative-node.h"
#include "sparse-graph/partitioners/base-partitioner.h"

namespace spg {

class MissionGraph {
 public:
  MissionGraph();
  void addNewVertices(
      const uint32_t submap_id, const pose_graph::VertexIdList& vertices);

  RepresentativeNodeVector computeSparseGraph(
      BasePartitioner* partitioner) const;

  const pose_graph::VertexIdList& getVerticesForId(
      const uint32_t submap_id) const noexcept;
  std::size_t getNumberOfVerticesForId(const uint32_t submap_id) const noexcept;
  const pose_graph::VertexId getVertex(
      const uint32_t submap_id, const uint32_t local_id) const noexcept;

  uint32_t getLocalVertexId(
      const uint32_t submap_id, const pose_graph::VertexId& v) const noexcept;
  bool containsVertex(
      const uint32_t submap_id, const pose_graph::VertexId& v) const noexcept;

  bool containsSubmap(const uint32_t submap_id) const noexcept;

  size_t size() const noexcept;
  std::vector<uint32_t> getAllSubmapIds() const;

  vi_map::MissionId getMissionIdForSubmap(
      const vi_map::VIMap* map, const uint32_t submap_id) const;

 private:
  std::unordered_map<uint32_t, pose_graph::VertexIdList> all_vertex_partitions_;
};

}  // namespace spg

#endif  //  SPARSE_GRAPH_MISSION_GRAPH_H_
