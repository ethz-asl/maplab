#include "vi-map/edge.h"

#include <aslam/common/memory.h>

#include "vi-map/cklam-edge.h"
#include "vi-map/laser-edge.h"
#include "vi-map/loopclosure-edge.h"
#include "vi-map/structure-loopclosure-edge.h"
#include "vi-map/trajectory-edge.h"
#include "vi-map/transformation-edge.h"
#include "vi-map/vi_map.pb.h"
#include "vi-map/viwls-edge.h"

namespace vi_map {

void Edge::setId(const pose_graph::EdgeId& id) {
  CHECK(id.isValid());
  id_ = id;
}

void Edge::setFrom(const pose_graph::VertexId& from) {
  CHECK(from.isValid());
  from_ = from;
}

void Edge::setTo(const pose_graph::VertexId& to) {
  CHECK(to.isValid());
  to_ = to;
}

void Edge::serialize(vi_map::proto::Edge* proto) const {
  CHECK_NOTNULL(proto);
  if (getType() == pose_graph::Edge::EdgeType::kStructureLoopClosure) {
    // do not serialize it!
    CHECK(false) << "StructureLoopClosure edge serialization not supported. "
                 << "The edges must be removed before the commit.";
    return;
  }

  if (getType() == pose_graph::Edge::EdgeType::kViwls) {
    const vi_map::ViwlsEdge& derived_edge =
        static_cast<const vi_map::ViwlsEdge&>(*this);
    derived_edge.serialize(proto->mutable_viwls());
  } else if (getType() == pose_graph::Edge::EdgeType::kOdometry) {
    const vi_map::TransformationEdge& derived_edge =
        static_cast<const vi_map::TransformationEdge&>(*this);
    derived_edge.serialize(proto->mutable_odometry());
  } else if (getType() == pose_graph::Edge::EdgeType::kLoopClosure) {
    const vi_map::LoopClosureEdge& derived_edge =
        static_cast<const vi_map::LoopClosureEdge&>(*this);
    derived_edge.serialize(proto->mutable_loopclosure());
  } else if (getType() == pose_graph::Edge::EdgeType::k6DoFGps) {
    const vi_map::TransformationEdge& derived_edge =
        static_cast<const vi_map::TransformationEdge&>(*this);
    derived_edge.serialize(proto->mutable_transformation());
  } else if (getType() == pose_graph::Edge::EdgeType::kLaser) {
    const vi_map::LaserEdge& derived_edge =
        static_cast<const vi_map::LaserEdge&>(*this);
    derived_edge.serialize(proto->mutable_laser());
  } else if (getType() == pose_graph::Edge::EdgeType::kTrajectory) {
    const vi_map::TrajectoryEdge& derived_edge =
        static_cast<const vi_map::TrajectoryEdge&>(*this);
    derived_edge.serialize(proto->mutable_trajectory());
  } else {
    LOG(FATAL) << "Unknown edge type.";
  }
}

Edge::UniquePtr Edge::deserialize(
    const pose_graph::EdgeId& edge_id, const vi_map::proto::Edge& proto) {
  if (proto.has_viwls()) {
    vi_map::ViwlsEdge* edge(new vi_map::ViwlsEdge());
    edge->deserialize(edge_id, proto.viwls());
    return Edge::UniquePtr(edge);
  } else if (proto.has_odometry()) {
    vi_map::TransformationEdge* edge(
        new vi_map::TransformationEdge(vi_map::Edge::EdgeType::kOdometry));
    edge->deserialize(edge_id, proto.odometry());
    return Edge::UniquePtr(edge);
  } else if (proto.has_loopclosure()) {
    vi_map::LoopClosureEdge* edge(new vi_map::LoopClosureEdge());
    edge->deserialize(edge_id, proto.loopclosure());
    return Edge::UniquePtr(edge);
  } else if (proto.has_transformation()) {
    vi_map::TransformationEdge* edge(
        new vi_map::TransformationEdge(vi_map::Edge::EdgeType::k6DoFGps));
    edge->deserialize(edge_id, proto.transformation());
    return Edge::UniquePtr(edge);
  } else if (proto.has_laser()) {
    vi_map::LaserEdge* edge(new vi_map::LaserEdge());
    edge->deserialize(edge_id, proto.laser());
    return Edge::UniquePtr(edge);
  } else if (proto.has_trajectory()) {
    vi_map::TrajectoryEdge* edge(new vi_map::TrajectoryEdge());
    edge->deserialize(edge_id, proto.trajectory());
    return Edge::UniquePtr(edge);
  } else {
    LOG(FATAL) << "Unknown edge type.";
    return nullptr;
  }
}

Edge::UniquePtr Edge::deserialize(
    const pose_graph::EdgeId& edge_id,
    const vi_map_deprecated::proto::Edge& proto) {
  if (proto.has_viwls()) {
    vi_map::ViwlsEdge* edge(new vi_map::ViwlsEdge());
    edge->deserialize(edge_id, proto.viwls());
    return Edge::UniquePtr(edge);
  } else if (proto.has_odometry()) {
    vi_map::TransformationEdge* edge(
        new vi_map::TransformationEdge(vi_map::Edge::EdgeType::kOdometry));
    edge->deserialize(edge_id, proto.odometry());
    return Edge::UniquePtr(edge);
  } else if (proto.has_loopclosure()) {
    vi_map::LoopClosureEdge* edge(new vi_map::LoopClosureEdge());
    edge->deserialize(edge_id, proto.loopclosure());
    return Edge::UniquePtr(edge);
  } else if (proto.has_transformation()) {
    vi_map::TransformationEdge* edge(
        new vi_map::TransformationEdge(vi_map::Edge::EdgeType::k6DoFGps));
    edge->deserialize(edge_id, proto.transformation());
    return Edge::UniquePtr(edge);
  } else if (proto.has_laser()) {
    vi_map::LaserEdge* edge(new vi_map::LaserEdge());
    edge->deserialize(edge_id, proto.laser());
    return Edge::UniquePtr(edge);
  } else if (proto.has_trajectory()) {
    vi_map::TrajectoryEdge* edge(new vi_map::TrajectoryEdge());
    edge->deserialize(edge_id, proto.trajectory());
    return Edge::UniquePtr(edge);
  } else {
    LOG(FATAL) << "Unknown edge type.";
    return nullptr;
  }
}

void Edge::copyEdgeInto(Edge** new_edge) const {
  CHECK_NOTNULL(new_edge);

  switch (edge_type_) {
    case pose_graph::Edge::EdgeType::kOdometry:
    case pose_graph::Edge::EdgeType::k6DoFGps: {
      copyEdge<TransformationEdge>(new_edge);
      break;
    }
    case pose_graph::Edge::EdgeType::kLoopClosure: {
      copyEdge<LoopClosureEdge>(new_edge);
      break;
    }
    case pose_graph::Edge::EdgeType::kViwls: {
      copyEdge<ViwlsEdge>(new_edge);
      break;
    }
    case pose_graph::Edge::EdgeType::kStructureLoopClosure: {
      copyEdge<StructureLoopclosureEdge>(new_edge);
      break;
    }
    case pose_graph::Edge::EdgeType::kLaser: {
      copyEdge<LaserEdge>(new_edge);
      break;
    }
    case pose_graph::Edge::EdgeType::kTrajectory: {
      copyEdge<TrajectoryEdge>(new_edge);
      break;
    }
    case pose_graph::Edge::EdgeType::kCklamImuLandmark: {
      copyEdge<CklamEdge>(new_edge);
      break;
    }
    case pose_graph::Edge::EdgeType::kUndefined:
    default: {
      LOG(FATAL) << "Edge type " << static_cast<int>(edge_type_)
                 << " is not supported by deep copy.";
    }
  }

  CHECK(new_edge != nullptr);
}

}  // namespace vi_map
