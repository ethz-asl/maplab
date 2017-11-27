#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <vi-map/loopclosure-edge.h>

namespace vi_map {
LoopClosureEdge::LoopClosureEdge()
    : vi_map::Edge(pose_graph::Edge::EdgeType::kLoopClosure),
      switch_variable_(1.0),
      switch_variable_variance_(1.0) {}

LoopClosureEdge::LoopClosureEdge(
    const pose_graph::EdgeId& id, const pose_graph::VertexId& from,
    const pose_graph::VertexId& to, const double switch_variable,
    const double switch_variable_variance, const pose::Transformation& T_A_B,
    const Eigen::Matrix<double, 6, 6>& T_A_B_covariance)
    : vi_map::Edge(pose_graph::Edge::EdgeType::kLoopClosure, id, from, to),
      switch_variable_(switch_variable),
      switch_variable_variance_(switch_variable_variance),
      T_A_B_(T_A_B),
      T_A_B_covariance_(T_A_B_covariance) {
  CHECK_NE(from, to)
      << "Loop-closure edges need to be defined between two distinct vertices.";
  CHECK_GT(switch_variable_variance, 0.0);
}

void LoopClosureEdge::serialize(vi_map::proto::LoopclosureEdge* proto) const {
  CHECK_NOTNULL(proto);

  proto->Clear();
  from_.serialize(proto->mutable_from());
  to_.serialize(proto->mutable_to());
  proto->set_switch_variable(switch_variable_);
  proto->set_switch_variable_variance(switch_variable_variance_);

  common::eigen_proto::serialize(
      T_A_B_covariance_, proto->mutable_t_a_b_covariance());
  common::eigen_proto::serialize(T_A_B_, proto->mutable_t_a_b());
}

void LoopClosureEdge::deserialize(
    const pose_graph::EdgeId& id, const vi_map::proto::LoopclosureEdge& proto) {
  id_ = id;
  from_.deserialize(proto.from());
  to_.deserialize(proto.to());
  switch_variable_ = proto.switch_variable();
  switch_variable_variance_ = proto.switch_variable_variance();

  common::eigen_proto::deserialize(proto.t_a_b(), &T_A_B_);
  common::eigen_proto::deserialize(
      proto.t_a_b_covariance(), &T_A_B_covariance_);
}

void LoopClosureEdge::set_T_A_B(const pose::Transformation& T_A_B) {
  T_A_B_ = T_A_B;
}

const pose::Transformation& LoopClosureEdge::getT_A_B() const {
  return T_A_B_;
}

void LoopClosureEdge::set_T_A_B_Covariance(
    const Eigen::Matrix<double, 6, 6>& T_A_B_covariance) {
  T_A_B_covariance_ = T_A_B_covariance;
}

const Eigen::Matrix<double, 6, 6>& LoopClosureEdge::getT_A_BCovariance() const {
  return T_A_B_covariance_;
}

}  // namespace vi_map
