#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include "vi-map/cklam-edge.h"

namespace vi_map {
CklamEdge::CklamEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to, const Eigen::Matrix<double, 12, 1>& b,
    const Eigen::Matrix<double, 12, 12>& A,
    const Eigen::Matrix<double, 7, 1>& keyframe_T_G_B_from,
    const Eigen::Matrix<double, 7, 1>& keyframe_T_G_B_to)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kCklamImuLandmark, id, from, to),
      b_(b),
      A_(A),
      keyframe_T_G_B_from_(keyframe_T_G_B_from),
      keyframe_T_G_B_to_(keyframe_T_G_B_to) {
  CHECK(id.isValid());
  CHECK(from_.isValid());
  CHECK(to_.isValid());
  CHECK_NEAR(keyframe_T_G_B_from.head<4>().norm(), 1.0, 1e-4);
  CHECK_NEAR(keyframe_T_G_B_to.head<4>().norm(), 1.0, 1e-4);
}

void CklamEdge::setb(const Eigen::Matrix<double, 12, 1>& b) {
  b_ = b;
}
const Eigen::Matrix<double, 12, 1>& CklamEdge::getb() const {
  return b_;
}

void CklamEdge::setA(const Eigen::Matrix<double, 12, 12>& A) {
  A_ = A;
}

const Eigen::Matrix<double, 12, 12>& CklamEdge::getA() const {
  return A_;
}

void CklamEdge::setKeyframePoseFrom(
    const Eigen::Matrix<double, 7, 1>& keyframe_T_G_B_from) {
  CHECK_NEAR(keyframe_T_G_B_from.head<4>().norm(), 1.0, 1e-4);
  keyframe_T_G_B_from_ = keyframe_T_G_B_from;
}

void CklamEdge::setKeyframePoseTo(
    const Eigen::Matrix<double, 7, 1>& keyframe_T_G_B_to) {
  CHECK_NEAR(keyframe_T_G_B_to.head<4>().norm(), 1.0, 1e-4);
  keyframe_T_G_B_to_ = keyframe_T_G_B_to;
}

const Eigen::Matrix<double, 7, 1>& CklamEdge::getKeyframePoseFrom() const {
  return keyframe_T_G_B_from_;
}

const Eigen::Matrix<double, 7, 1>& CklamEdge::getKeyframePoseTo() const {
  return keyframe_T_G_B_to_;
}

void CklamEdge::serialize(vi_map::proto::CklamEdge* proto) const {
  CHECK_NOTNULL(proto);

  proto->Clear();
  from_.serialize(proto->mutable_from());
  to_.serialize(proto->mutable_to());
  common::eigen_proto::serialize(b_, proto->mutable_b());
  common::eigen_proto::serialize(A_, proto->mutable_a());
  common::eigen_proto::serialize(
      keyframe_T_G_B_from_, proto->mutable_keyframe_t_g_b_from());
  common::eigen_proto::serialize(
      keyframe_T_G_B_to_, proto->mutable_keyframe_t_g_b_to());
}

void CklamEdge::deserialize(
    const pose_graph::EdgeId& id, const vi_map::proto::CklamEdge& proto) {
  id_ = id;
  from_.deserialize(proto.from());
  to_.deserialize(proto.to());

  common::eigen_proto::deserialize(proto.b(), &b_);
  common::eigen_proto::deserialize(proto.a(), &A_);
  common::eigen_proto::deserialize(
      proto.keyframe_t_g_b_from(), &keyframe_T_G_B_from_);
  common::eigen_proto::deserialize(
      proto.keyframe_t_g_b_to(), &keyframe_T_G_B_to_);
}

}  // namespace vi_map
