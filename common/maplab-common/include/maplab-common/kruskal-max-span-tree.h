#ifndef MAPLAB_COMMON_KRUSKAL_MAX_SPAN_TREE_H_
#define MAPLAB_COMMON_KRUSKAL_MAX_SPAN_TREE_H_

#include <algorithm>
#include <stack>
#include <vector>

#include <glog/logging.h>

namespace common {
// Find root with path compression, returns root of v.
__inline__ int FindRoot(int node, std::vector<int>* edge_list) {
  CHECK_NOTNULL(edge_list);
  std::stack<int> stck;
  stck.push(node);

  while ((*edge_list)[node] != node) {  // Find root.
    node = (*edge_list)[node];
    stck.push(node);
  }
  while (!stck.empty()) {  // Compress children.
    (*edge_list)[stck.top()] = node;
    stck.pop();
  }
  return node;
}

// Union of tree-partitions by size.
__inline__ void UnionClusters(
    int node_a, int node_b, std::vector<int>* parent, std::vector<int>* rank) {
  CHECK_NOTNULL(parent);
  CHECK_NOTNULL(rank);
  if ((*rank)[node_a] < (*rank)[node_b]) {
    (*parent)[node_a] = node_b;
  } else if ((*rank)[node_a] > (*rank)[node_b]) {
    (*parent)[node_b] = node_a;
  } else {
    (*parent)[node_b] = node_a;
    ++((*rank)[node_a]);
  }
}

template <template <typename, typename> class EdgeContainer,
          template <typename> class EdgeAllocator, typename Edge>
__inline__ bool KruskalMaxSpanTree(
    int num_vertices, EdgeContainer<Edge, EdgeAllocator<Edge> >* edges) {
  CHECK_NOTNULL(edges);
  std::vector<int> parents;
  parents.reserve(num_vertices);
  for (int i = 0; i < num_vertices; ++i) {
    parents.push_back(i);
  }

  std::vector<int> vi_rank;
  vi_rank.reserve(num_vertices);
  vi_rank.insert(vi_rank.begin(), num_vertices, 1);

  edges->sort([](const Edge& lhs, const Edge& rhs) {
    return std::get<2>(lhs) > std::get<2>(rhs);
  });

  int in_tree_edges = 0;

  typename EdgeContainer<Edge, EdgeAllocator<Edge> >::iterator edge_it =
      edges->begin();
  while (in_tree_edges < num_vertices - 1 && edge_it != edges->end()) {
    Edge& edge = *edge_it;
    if (FindRoot(std::get<0>(edge), &parents) !=
        FindRoot(std::get<1>(edge), &parents)) {
      UnionClusters(
          parents[std::get<0>(edge)], parents[std::get<1>(edge)], &parents,
          &vi_rank);
      ++in_tree_edges;
      ++edge_it;
    } else {
      edge_it = edges->erase(edge_it);
    }
  }
  edges->erase(edge_it, edges->end());
  if (static_cast<int>(edges->size()) != num_vertices - 1) {
    return false;
  } else {
    LOG(INFO) << "Done. Got " << edges->size() << " edges in MST.";
    return true;
  }
}

}  // namespace common
#endif  // MAPLAB_COMMON_KRUSKAL_MAX_SPAN_TREE_H_
