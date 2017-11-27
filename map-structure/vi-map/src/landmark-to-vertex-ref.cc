#include "vi-map/landmark-to-vertex-ref.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

void LandmarkToVertexReference::serialize(
    vi_map::proto::LandmarkToVertexReference* proto) const {
  vertex_id.serialize(proto->mutable_vertex_id());
  landmark_id.serialize(proto->mutable_landmark_id());
}

void LandmarkToVertexReference::deserialize(
    const vi_map::proto::LandmarkToVertexReference& proto) {
  vertex_id.deserialize(proto.vertex_id());
  landmark_id.deserialize(proto.landmark_id());
}

}  // namespace vi_map
