#include <vi-map/mission.h>

namespace vi_map {
void Mission::setId(const MissionId& id) {
  mission_id_ = id;
}
const MissionId& Mission::id() const {
  return mission_id_;
}

void Mission::setBaseFrameId(const MissionBaseFrameId& base_frame_id) {
  base_frame_id_ = base_frame_id;
}
const MissionBaseFrameId& Mission::getBaseFrameId() const {
  return base_frame_id_;
}

void Mission::setRootVertexId(const pose_graph::VertexId& vertex_id) {
  root_vertex_id_ = vertex_id;
}
const pose_graph::VertexId& Mission::getRootVertexId() const {
  return root_vertex_id_;
}

Mission::BackBone Mission::backboneType() const {
  return backbone_type_;
}
void Mission::setBackboneType(BackBone backbone_type) {
  backbone_type_ = backbone_type;
}

}  // namespace vi_map
