#ifndef SPARSE_GRAPH_MISSION_GRAPH_H_
#define SPARSE_GRAPH_MISSION_GRAPH_H_

#include <vi-map/vi-map.h>

#include <vector>

namespace spg {

class MissionGraph {
 public:
  MissionGraph();
  void addNewVertices(const pose_graph::VertexIdList& vertices);

  size_t size() const noexcept;

 private:
  std::vector<pose_graph::VertexIdList> all_vertex_partitions_;
};

}  // namespace spg

#endif  //  SPARSE_GRAPH_MISSION_GRAPH_H_
