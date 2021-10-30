#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <vi-map/structure-loopclosure-edge.h>

namespace vi_map {
StructureLoopclosureEdge::StructureLoopclosureEdge()
    : vi_map::Edge(pose_graph::Edge::EdgeType::kStructureLoopClosure),
      switch_variable_(0.0) {}

StructureLoopclosureEdge::StructureLoopclosureEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to, const double switch_variable)
    : vi_map::Edge(
          pose_graph::Edge::EdgeType::kStructureLoopClosure, id, from, to),
      switch_variable_(switch_variable) {}

}  // namespace vi_map
