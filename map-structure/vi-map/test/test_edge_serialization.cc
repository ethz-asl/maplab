#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/common/hash-id.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-frame.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>

#include "vi-map/landmark.h"
#include "vi-map/laser-edge.h"
#include "vi-map/loopclosure-edge.h"
#include "vi-map/mission.h"
#include "vi-map/pose-graph.h"
#include "vi-map/trajectory-edge.h"
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
  void constructLaserEdge();
  void constructTrajectoryEdge();
  void serializeAndDeserializeViwlsEdge();
  void serializeAndDeserializeLoopclosureEdge();
  void serializeAndDeserializeTransformationEdge();
  void serializeAndDeserializeLaserEdge();
  void serializeAndDeserializeTrajectoryEdge();

  vi_map::proto::TransformationEdge odometry_edge_proto_;
  vi_map::proto::ViwlsEdge viwls_edge_proto_;
  vi_map::proto::LoopclosureEdge loopclosure_edge_proto_;
  vi_map::proto::LaserEdge laser_edge_proto_;
  vi_map::proto::TrajectoryEdge trajectory_edge_proto_;

  vi_map::ViwlsEdge::UniquePtr viwls_edge_;
  vi_map::TransformationEdge::UniquePtr odo_edge_;
  vi_map::LoopClosureEdge::UniquePtr loop_edge_;
  vi_map::LaserEdge::UniquePtr laser_edge_;
  vi_map::TrajectoryEdge::UniquePtr trajectory_edge_;

  vi_map::ViwlsEdge::UniquePtr viwls_edge_from_msg_;
  vi_map::TransformationEdge::UniquePtr odo_edge_from_msg_;
  vi_map::LoopClosureEdge::UniquePtr loop_edge_from_msg_;
  vi_map::LaserEdge::UniquePtr laser_edge_from_msg_;
  vi_map::TrajectoryEdge::UniquePtr trajectory_edge_from_msg_;

  // Edge data:
  pose_graph::EdgeId id_;
  pose_graph::VertexId from_, to_;
  pose::Transformation T_A_B_;
  Eigen::Matrix<double, 6, 6> T_A_B_covariance_;
  double switch_variable_;
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> imu_data_;
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> laser_timestamps_ns_;
  Eigen::Matrix<double, 4, Eigen::Dynamic> laser_data_xyzi_;
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
void ViwlsGraph::serializeAndDeserializeLaserEdge() {
  laser_edge_->serialize(&laser_edge_proto_);
  laser_edge_from_msg_ = aligned_unique<vi_map::LaserEdge>();
  laser_edge_from_msg_->deserialize(laser_edge_->id(), laser_edge_proto_);
}
void ViwlsGraph::serializeAndDeserializeTrajectoryEdge() {
  trajectory_edge_->serialize(&trajectory_edge_proto_);
  trajectory_edge_from_msg_ = aligned_unique<vi_map::TrajectoryEdge>();
  trajectory_edge_from_msg_->deserialize(
      trajectory_edge_->id(), trajectory_edge_proto_);
}
void ViwlsGraph::constructOdometryEdge() {
  common::generateId(&id_);
  common::generateId(&from_);
  common::generateId(&to_);
  T_A_B_.setRandom();
  T_A_B_covariance_.setRandom();
  odo_edge_ = aligned_unique<vi_map::TransformationEdge>(
      vi_map::Edge::EdgeType::kOdometry, id_, from_, to_, T_A_B_,
      T_A_B_covariance_);
}
void ViwlsGraph::constructViwlsEdge() {
  common::generateId(&id_);
  common::generateId(&from_);
  common::generateId(&to_);
  imu_data_.resize(Eigen::NoChange, 20);
  imu_data_.setRandom();
  imu_timestamps_.resize(Eigen::NoChange, 20);
  imu_timestamps_.setRandom();

  viwls_edge_ = aligned_unique<vi_map::ViwlsEdge>(
      id_, from_, to_, imu_timestamps_, imu_data_);
}
void ViwlsGraph::constructLoopclosureEdge() {
  common::generateId(&id_);
  common::generateId(&from_);
  common::generateId(&to_);
  T_A_B_.setRandom();
  T_A_B_covariance_.setRandom();
  switch_variable_ = 1.0;
  const double kSwitchVariableVariance = 1e-3;
  loop_edge_ = aligned_unique<vi_map::LoopClosureEdge>(
      id_, from_, to_, switch_variable_, kSwitchVariableVariance, T_A_B_,
      T_A_B_covariance_);
}
void ViwlsGraph::constructLaserEdge() {
  common::generateId(&id_);
  common::generateId(&from_);
  common::generateId(&to_);
  laser_data_xyzi_.resize(Eigen::NoChange, 20);
  laser_data_xyzi_.setRandom();
  laser_timestamps_ns_.resize(Eigen::NoChange, 20);
  laser_timestamps_ns_.setRandom();

  laser_edge_ = aligned_unique<vi_map::LaserEdge>(
      id_, from_, to_, laser_timestamps_ns_, laser_data_xyzi_);
}
void ViwlsGraph::constructTrajectoryEdge() {
  common::generateId(&id_);
  common::generateId(&from_);
  common::generateId(&to_);
  trajectory_G_T_I_pq_.resize(Eigen::NoChange, 20);
  trajectory_G_T_I_pq_.setRandom();
  trajectory_timestamps_ns_.resize(Eigen::NoChange, 20);
  trajectory_timestamps_ns_.setRandom();
  trajectory_identifier_ = 31416;

  trajectory_edge_ = aligned_unique<vi_map::TrajectoryEdge>(
      id_, from_, to_, trajectory_timestamps_ns_, trajectory_G_T_I_pq_,
      trajectory_identifier_);
}
TEST_F(ViwlsGraph, OdometryEdgeSerializationTest) {
  constructOdometryEdge();
  serializeAndDeserializeTransformationEdge();
  ASSERT_TRUE(odo_edge_from_msg_ != nullptr);
  EXPECT_EQ(id_, odo_edge_from_msg_->id());
  EXPECT_EQ(to_, odo_edge_from_msg_->to());
  EXPECT_EQ(from_, odo_edge_from_msg_->from());

  pose::Transformation tmp_quat = odo_edge_from_msg_->getT_A_B();
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

  pose::Transformation tmp_quat = loop_edge_from_msg_->getT_A_B();
  EXPECT_NEAR_EIGEN(
      T_A_B_.getTransformationMatrix(), tmp_quat.getTransformationMatrix(),
      1e-20);
  Eigen::Matrix<double, 6, 6> tmp_cov =
      loop_edge_from_msg_->getT_A_BCovariance();
  EXPECT_NEAR_EIGEN(T_A_B_covariance_, tmp_cov, 1e-20);
  EXPECT_EQ(loop_edge_from_msg_->getSwitchVariable(), switch_variable_);
}

TEST_F(ViwlsGraph, LaserEdgeSerializationTest) {
  constructLaserEdge();
  serializeAndDeserializeLaserEdge();
  ASSERT_TRUE(laser_edge_from_msg_ != nullptr);
  EXPECT_EQ(id_, laser_edge_from_msg_->id());
  EXPECT_EQ(to_, laser_edge_from_msg_->to());
  EXPECT_EQ(from_, laser_edge_from_msg_->from());
  EXPECT_NEAR_EIGEN(
      laser_data_xyzi_, laser_edge_from_msg_->getLaserData(), 1e-20);
  EXPECT_NEAR_EIGEN(
      laser_timestamps_ns_, laser_edge_from_msg_->getLaserTimestamps(), 1);
}

TEST_F(ViwlsGraph, TrajectoryEdgeSerializationTest) {
  constructTrajectoryEdge();
  serializeAndDeserializeTrajectoryEdge();
  ASSERT_TRUE(trajectory_edge_from_msg_ != nullptr);
  EXPECT_EQ(id_, trajectory_edge_from_msg_->id());
  EXPECT_EQ(to_, trajectory_edge_from_msg_->to());
  EXPECT_EQ(from_, trajectory_edge_from_msg_->from());
  EXPECT_NEAR_EIGEN(
      trajectory_G_T_I_pq_, trajectory_edge_from_msg_->getTrajectoryData(),
      1e-20);
  EXPECT_NEAR_EIGEN(
      trajectory_timestamps_ns_,
      trajectory_edge_from_msg_->getTrajectoryTimestamps(), 1);
  EXPECT_EQ(trajectory_edge_from_msg_->getIdentifier(), trajectory_identifier_);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
