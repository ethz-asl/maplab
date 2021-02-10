#include "sparse-graph/partitioners/all-partitioner.h"

#include <glog/logging.h>

namespace spg {

AllPartitioner::AllPartitioner(const vi_map::VIMap& map)
    : BasePartitioner(map) {}

RepresentativeNodeVector AllPartitioner::getRepresentativesForSubmap(
    const pose_graph::VertexIdList& vertices, const uint64_t submap_id) {
  const std::size_t n_vertices = vertices.size();
  if (n_vertices == 0) {
    return {};
  }
  RepresentativeNodeVector all_vertices;
  for (std::size_t i = 0u; i < n_vertices; ++i) {
    if (!map_.hasVertex(vertices[i])) {
      LOG(ERROR) << "Vertex " << vertices[i] << " not found.";
      continue;
    }
    const vi_map::Vertex& vertex = map_.getVertex(vertices[i]);
    const int64_t ts_vertex_ns =
        vertex.getVisualNFrame().getMinTimestampNanoseconds();

    // Insert a new representative node if it is not already present.
    RepresentativeNode current_node(
        vertex.get_T_M_I(), ts_vertex_ns, submap_id);
    if (std::find(all_vertices.cbegin(), all_vertices.cend(), current_node) ==
        all_vertices.cend()) {
      all_vertices.emplace_back(current_node);
    }
  }

  return all_vertices;
}

}  // namespace spg
