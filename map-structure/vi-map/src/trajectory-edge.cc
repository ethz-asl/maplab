#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <vi-map/trajectory-edge.h>

namespace vi_map {

TrajectoryEdge::TrajectoryEdge()
    : vi_map::Edge(pose_graph::Edge::EdgeType::kTrajectory),
      trajectory_identifier_(0) {}

TrajectoryEdge::TrajectoryEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& trajectory_timestamps_ns,
    const Eigen::Matrix<double, 7, Eigen::Dynamic>& trajectory_G_T_I_pq,
    const uint32_t trajectory_identifier)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kTrajectory, id, from, to),
      trajectory_timestamps_ns_(trajectory_timestamps_ns),
      trajectory_G_T_I_pq_(trajectory_G_T_I_pq),
      trajectory_identifier_(trajectory_identifier) {
  CHECK_EQ(trajectory_timestamps_ns.cols(), trajectory_G_T_I_pq.cols());
}

TrajectoryEdge::TrajectoryEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kTrajectory, id, from, to),
      trajectory_identifier_(0) {}

bool TrajectoryEdge::operator==(const TrajectoryEdge& other) const {
  bool is_same = true;
  is_same &= static_cast<const vi_map::Edge&>(*this) == other;
  is_same &= id_ == other.id_;
  is_same &= from_ == other.from_;
  is_same &= to_ == other.to_;
  is_same &= trajectory_timestamps_ns_ == other.trajectory_timestamps_ns_;
  is_same &= trajectory_G_T_I_pq_ == other.trajectory_G_T_I_pq_;
  is_same &= trajectory_identifier_ == other.trajectory_identifier_;
  return is_same;
}

void TrajectoryEdge::serialize(vi_map::proto::TrajectoryEdge* proto) const {
  CHECK_NOTNULL(proto);
  proto->Clear();

  from_.serialize(proto->mutable_from());
  to_.serialize(proto->mutable_to());

  common::eigen_proto::serialize(
      trajectory_G_T_I_pq_, proto->mutable_trajectory_g_t_i_pq());
  common::eigen_proto::serialize(
      trajectory_timestamps_ns_, proto->mutable_trajectory_timestamps_ns());

  proto->set_trajectory_identifier(trajectory_identifier_);
}

void TrajectoryEdge::deserialize(
    const pose_graph::EdgeId& id, const vi_map::proto::TrajectoryEdge& proto) {
  id_ = id;
  CHECK(proto.has_from());
  from_.deserialize(proto.from());
  CHECK(proto.has_to());
  to_.deserialize(proto.to());

  common::eigen_proto::deserialize(
      proto.trajectory_timestamps_ns(), &trajectory_timestamps_ns_);
  common::eigen_proto::deserialize(
      proto.trajectory_g_t_i_pq(), &trajectory_G_T_I_pq_);

  if (proto.has_trajectory_identifier()) {
    trajectory_identifier_ = proto.trajectory_identifier();
  } else {
    trajectory_identifier_ = 0;
  }
}

const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>&
TrajectoryEdge::getTrajectoryTimestamps() const {
  return trajectory_timestamps_ns_;
}

const Eigen::Matrix<double, 7, Eigen::Dynamic>&
TrajectoryEdge::getTrajectoryData() const {
  return trajectory_G_T_I_pq_;
}

uint32_t TrajectoryEdge::getIdentifier() const {
  return trajectory_identifier_;
}

}  // namespace vi_map
