#ifndef POSEGRAPH_POSE_GRAPH_INL_H_
#define POSEGRAPH_POSE_GRAPH_INL_H_

#include "posegraph/pose-graph.h"

namespace pose_graph {

template <pose_graph::Edge::EdgeType edge_type>
size_t PoseGraph::removeEdgesOfType() {
  static_assert(
      edge_type != Edge::EdgeType::kViwls &&
          edge_type != Edge::EdgeType::kOdometry,
      "Removing Viwls or Odometry edges is not allowed.");

  size_t number_of_edges_removed = 0u;
  typename EdgeMap::iterator iterator = edges_.begin();
  while (iterator != edges_.end()) {
    if (iterator->second->getType() == edge_type) {
      iterator = removeEdge(iterator->first);
      ++number_of_edges_removed;
    } else {
      ++iterator;
    }
  }
  return number_of_edges_removed;
}

void PoseGraph::clear() {
  vertices_.clear();
  edges_.clear();
}

}  // namespace pose_graph

#endif  // POSEGRAPH_POSE_GRAPH_INL_H_
