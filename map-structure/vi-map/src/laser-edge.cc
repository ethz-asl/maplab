#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <vi-map/laser-edge.h>

namespace vi_map {

LaserEdge::LaserEdge() : vi_map::Edge(pose_graph::Edge::EdgeType::kLaser) {}

LaserEdge::LaserEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& laser_timestamps_ns,
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& laser_data_xyzi)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kLaser, id, from, to),
      laser_timestamps_ns_(laser_timestamps_ns),
      laser_data_xyzi_(laser_data_xyzi) {
  CHECK_EQ(laser_timestamps_ns.cols(), laser_data_xyzi.cols());
}

LaserEdge::LaserEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kLaser, id, from, to) {}

bool LaserEdge::operator==(const LaserEdge& other) const {
  bool is_same = true;
  is_same &= static_cast<const vi_map::Edge&>(*this) == other;
  is_same &= id_ == other.id_;
  is_same &= from_ == other.from_;
  is_same &= to_ == other.to_;
  is_same &= laser_timestamps_ns_ == other.laser_timestamps_ns_;
  is_same &= laser_data_xyzi_ == other.laser_data_xyzi_;
  return is_same;
}

void LaserEdge::serialize(vi_map::proto::LaserEdge* proto) const {
  CHECK_NOTNULL(proto);

  proto->Clear();

  from_.serialize(proto->mutable_from());
  to_.serialize(proto->mutable_to());

  common::eigen_proto::serialize(
      laser_data_xyzi_, proto->mutable_laser_data_xyzi());
  common::eigen_proto::serialize(
      laser_timestamps_ns_, proto->mutable_laser_timestamps_ns());
}

void LaserEdge::deserialize(
    const pose_graph::EdgeId& id, const vi_map::proto::LaserEdge& proto) {
  id_ = id;
  CHECK(proto.has_from());
  from_.deserialize(proto.from());
  CHECK(proto.has_to());
  to_.deserialize(proto.to());

  common::eigen_proto::deserialize(
      proto.laser_timestamps_ns(), &laser_timestamps_ns_);
  common::eigen_proto::deserialize(proto.laser_data_xyzi(), &laser_data_xyzi_);
}

const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& LaserEdge::getLaserTimestamps()
    const {
  return laser_timestamps_ns_;
}

const Eigen::Matrix<double, 4, Eigen::Dynamic>& LaserEdge::getLaserData()
    const {
  return laser_data_xyzi_;
}

}  // namespace vi_map
