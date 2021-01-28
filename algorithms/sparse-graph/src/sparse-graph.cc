#include "sparse-graph/sparse-graph.h"

#include <glog/logging.h>

namespace spg {

SparseGraph::SparseGraph(const vi_map::VIMap& map) : map_(map) {}

void SparseGraph::addVerticesToMissionGraph(
    const std::string& map_key, const pose_graph::VertexIdList& vertices) {
  mission_graphs_[map_key].addNewVertices(vertices);
}

void SparseGraph::compute() {}

std::size_t SparseGraph::getMissionGraphSize(const std::string& map_key) const
    noexcept {
  auto mission_it = mission_graphs_.find(map_key);
  if (mission_it == mission_graphs_.end()) {
    LOG(WARNING) << "Unknown map key " << map_key << " provided.";
    return 0;
  }
  return mission_it->second.size();
}

}  // namespace spg
