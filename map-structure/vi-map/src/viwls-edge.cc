#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <vi-map/viwls-edge.h>

namespace vi_map {

ViwlsEdge::ViwlsEdge() : vi_map::Edge(pose_graph::Edge::EdgeType::kViwls) {}

ViwlsEdge::ViwlsEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kViwls, id, from, to),
      imu_timestamps_(imu_timestamps),
      imu_data_(imu_data) {
  CHECK_EQ(imu_timestamps.cols(), imu_data.cols());
}

ViwlsEdge::ViwlsEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kViwls, id, from, to) {}

bool ViwlsEdge::operator==(const ViwlsEdge& other) const {
  bool is_same = true;
  is_same &= static_cast<const vi_map::Edge&>(*this) == other;
  is_same &= id_ == other.id_;
  is_same &= from_ == other.from_;
  is_same &= to_ == other.to_;
  is_same &= imu_timestamps_ == other.imu_timestamps_;
  is_same &= imu_data_ == other.imu_data_;
  return is_same;
}

void ViwlsEdge::serialize(vi_map::proto::ViwlsEdge* proto) const {
  CHECK_NOTNULL(proto);

  proto->Clear();

  from_.serialize(proto->mutable_from());
  to_.serialize(proto->mutable_to());

  common::eigen_proto::serialize(imu_data_, proto->mutable_imu_data());
  common::eigen_proto::serialize(
      imu_timestamps_, proto->mutable_imu_timestamps());
}

void ViwlsEdge::deserialize(
    const pose_graph::EdgeId& id, const vi_map::proto::ViwlsEdge& proto) {
  id_ = id;
  CHECK(proto.has_from());
  from_.deserialize(proto.from());
  CHECK(proto.has_to());
  to_.deserialize(proto.to());

  common::eigen_proto::deserialize(proto.imu_timestamps(), &imu_timestamps_);
  common::eigen_proto::deserialize(proto.imu_data(), &imu_data_);
}

const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& ViwlsEdge::getImuTimestamps()
    const {
  return imu_timestamps_;
}

const Eigen::Matrix<double, 6, Eigen::Dynamic>& ViwlsEdge::getImuData() const {
  return imu_data_;
}

}  // namespace vi_map
