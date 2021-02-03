#ifndef SPARSE_GRAPH_MISSION_GRAPH_H_
#define SPARSE_GRAPH_MISSION_GRAPH_H_

#include <vi-map/vi-map.h>

#include <unordered_map>
#include <vector>

namespace spg {

class MissionGraph {
 public:
  MissionGraph();
  void addNewVertices(
      const uint64_t submap_id, const pose_graph::VertexIdList& vertices);

  size_t size() const noexcept;

 private:
  std::unordered_map<uint64_t, pose_graph::VertexIdList> all_vertex_partitions_;
};

}  // namespace spg

#endif  //  SPARSE_GRAPH_MISSION_GRAPH_H_
