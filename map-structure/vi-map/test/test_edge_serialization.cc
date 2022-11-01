#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/hash-id.h>
#include <aslam/common/memory.h>
#include <aslam/common/unique-id.h>
#include <aslam/frames/visual-frame.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <posegraph/unique-id.h>

#include "vi-map/landmark.h"
#include "vi-map/loopclosure-edge.h"
#include "vi-map/mission.h"
#include "vi-map/pose-graph.h"
#include "vi-map/transformation-edge.h"
#include "vi-map/unique-id.h"
#include "vi-map/vertex.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"
#include "vi-map/viwls-edge.h"

namespace vi_map {

class ViwlsGraph : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  void constructViwlsEdge();
  void constructLoopclosureEdge();
  void constructOdometryEdge();
  void serializeAndDeserializeViwlsEdge();
  void serializeAndDeserializeLoopclosureEdge();
  void serializeAndDeserializeTransformationEdge();

  vi_map::proto::TransformationEdge odometry_edge_proto_;
  vi_map::proto::ViwlsEdge viwls_edge_proto_;
  vi_map::proto::LoopclosureEdge loopclosure_edge_proto_;

  vi_map::ViwlsEdge::UniquePtr viwls_edge_;
  vi_map::TransformationEdge::UniquePtr odo_edge_;
  vi_map::LoopClosureEdge::UniquePtr loop_edge_;

  vi_map::ViwlsEdge::UniquePtr viwls_edge_from_msg_;
  vi_map::TransformationEdge::UniquePtr odo_edge_from_msg_;
  vi_map::LoopClosureEdge::UniquePtr loop_edge_from_msg_;

  // Edge data:
  pose_graph::EdgeId id_;
  aslam::SensorId sensor_id_;
  pose_graph::VertexId from_, to_;
  pose::Transformation T_A_B_;
  Eigen::Matrix<double, 6, 6> T_A_B_covariance_;
  double switch_variable_;
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_;
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> trajectory_timestamps_ns_;
  Eigen::Matrix<double, 7, Eigen::Dynamic> trajectory_G_T_I_pq_;
  uint32_t trajectory_identifier_;
};

void ViwlsGraph::serializeAndDeserializeLoopclosureEdge() {
  loop_edge_->serialize(&loopclosure_edge_proto_);
  loop_edge_from_msg_ = aligned_unique<vi_map::LoopClosureEdge>();
  loop_edge_from_msg_->deserialize(loop_edge_->id(), loopclosure_edge_proto_);
}
void ViwlsGraph::serializeAndDeserializeTransformationEdge() {
  odo_edge_->serialize(&odometry_edge_proto_);
  odo_edge_from_msg_ = aligned_unique<vi_map::TransformationEdge>(
      vi_map::Edge::EdgeType::kOdometry);
  odo_edge_from_msg_->deserialize(odo_edge_->id(), odometry_edge_proto_);
}
void ViwlsGraph::serializeAndDeserializeViwlsEdge() {
  viwls_edge_->serialize(&viwls_edge_proto_);
  viwls_edge_from_msg_ = aligned_unique<vi_map::ViwlsEdge>();
  viwls_edge_from_msg_->deserialize(viwls_edge_->id(), viwls_edge_proto_);
}
void ViwlsGraph::constructOdometryEdge() {
  aslam::generateId(&id_);
  aslam::generateId(&sensor_id_);
  aslam::generateId(&from_);
  aslam::generateId(&to_);
  T_A_B_.setRandom();
  T_A_B_covariance_.setRandom();
  odo_edge_ = aligned_unique<vi_map::TransformationEdge>(
      vi_map::Edge::EdgeType::kOdometry, id_, from_, to_, T_A_B_,
      T_A_B_covariance_, sensor_id_);
}
void ViwlsGraph::constructViwlsEdge() {
  aslam::generateId(&id_);
  aslam::generateId(&from_);
  aslam::generateId(&to_);
  imu_data_.resize(Eigen::NoChange, 20);
  imu_data_.setRandom();
  imu_timestamps_.resize(Eigen::NoChange, 20);
  imu_timestamps_.setRandom();

  viwls_edge_ = aligned_unique<vi_map::ViwlsEdge>(
      id_, from_, to_, imu_timestamps_, imu_data_);
}
void ViwlsGraph::constructLoopclosureEdge() {
  aslam::generateId(&id_);
  aslam::generateId(&from_);
  aslam::generateId(&to_);
  T_A_B_.setRandom();
  T_A_B_covariance_.setRandom();
  switch_variable_ = 1.0;
  const double kSwitchVariableVariance = 1e-3;
  loop_edge_ = aligned_unique<vi_map::LoopClosureEdge>(
      id_, from_, to_, switch_variable_, kSwitchVariableVariance, T_A_B_,
      T_A_B_covariance_);
}
TEST_F(ViwlsGraph, OdometryEdgeSerializationTest) {
  constructOdometryEdge();
  serializeAndDeserializeTransformationEdge();
  ASSERT_TRUE(odo_edge_from_msg_ != nullptr);
  EXPECT_EQ(id_, odo_edge_from_msg_->id());
  EXPECT_EQ(to_, odo_edge_from_msg_->to());
  EXPECT_EQ(from_, odo_edge_from_msg_->from());

  pose::Transformation tmp_quat = odo_edge_from_msg_->get_T_A_B();
  EXPECT_NEAR_EIGEN(
      T_A_B_.getTransformationMatrix(), tmp_quat.getTransformationMatrix(),
      1e-20);
  Eigen::Matrix<double, 6, 6> tmp_cov =
      odo_edge_from_msg_->get_T_A_B_Covariance_p_q();
  EXPECT_NEAR_EIGEN(T_A_B_covariance_, tmp_cov, 1e-20);
}

TEST_F(ViwlsGraph, ViwlsEdgeSerializationTest) {
  constructViwlsEdge();
  serializeAndDeserializeViwlsEdge();
  ASSERT_TRUE(viwls_edge_from_msg_ != nullptr);
  EXPECT_EQ(id_, viwls_edge_from_msg_->id());
  EXPECT_EQ(to_, viwls_edge_from_msg_->to());
  EXPECT_EQ(from_, viwls_edge_from_msg_->from());
  EXPECT_NEAR_EIGEN(imu_data_, viwls_edge_from_msg_->getImuData(), 1e-20);
  EXPECT_NEAR_EIGEN(
      imu_timestamps_, viwls_edge_from_msg_->getImuTimestamps(), 1);
}

TEST_F(ViwlsGraph, LoopclosureEdgeSerializationTest) {
  constructLoopclosureEdge();
  serializeAndDeserializeLoopclosureEdge();
  ASSERT_TRUE(loop_edge_from_msg_ != nullptr);
  EXPECT_EQ(id_, loop_edge_from_msg_->id());
  EXPECT_EQ(to_, loop_edge_from_msg_->to());
  EXPECT_EQ(from_, loop_edge_from_msg_->from());

  pose::Transformation tmp_quat = loop_edge_from_msg_->get_T_A_B();
  EXPECT_NEAR_EIGEN(
      T_A_B_.getTransformationMatrix(), tmp_quat.getTransformationMatrix(),
      1e-20);
  Eigen::Matrix<double, 6, 6> tmp_cov =
      loop_edge_from_msg_->get_T_A_B_Covariance();
  EXPECT_NEAR_EIGEN(T_A_B_covariance_, tmp_cov, 1e-20);
  EXPECT_EQ(loop_edge_from_msg_->getSwitchVariable(), switch_variable_);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
