#ifndef SPARSE_GRAPH_SPARSE_GRAPH_H_
#define SPARSE_GRAPH_SPARSE_GRAPH_H_

#include <map>
#include <string>
#include <vector>

#include <vi-map/vi-map.h>

#include "sparse-graph/mission-graph.h"

namespace spg {

class SparseGraph {
 public:
  explicit SparseGraph(const vi_map::VIMap& map);
  void addVerticesToMissionGraph(
      const std::string& map_key, const pose_graph::VertexIdList& vertices);

  std::size_t getMissionGraphSize(const std::string& map_key) const noexcept;

  void compute();

 private:
  const vi_map::VIMap& map_;
  std::map<std::string, MissionGraph> mission_graphs_;
};

}  // namespace spg

#endif  //  SPARSE_GRAPH_SPARSE_GRAPH_H_
