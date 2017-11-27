#include <memory>
#include <string>

#include <Eigen/Core>
#include <aslam/common/hash-id.h>
#include <aslam/common/memory.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>

#include "vi-map/mission-baseframe.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

class ViwlsGraph : public ::testing::Test {
 protected:
  void constructBaseframe();
  void serializeAndDeserialize();

  vi_map::MissionBaseFrame::UniquePtr initial_baseframe_;
  vi_map::MissionBaseFrame::UniquePtr baseframe_from_msg_;

  pose::Transformation T_G_M_;
  Eigen::Matrix<double, 6, 6> T_G_M_covariance_;
  vi_map::MissionBaseFrameId mission_baseframe_id_;
};

void ViwlsGraph::serializeAndDeserialize() {
  vi_map::proto::MissionBaseframe baseframe_proto, proto_from_msg;
  initial_baseframe_->serialize(&baseframe_proto);
  std::string serialized_baseframe = baseframe_proto.SerializeAsString();

  proto_from_msg.ParseFromString(serialized_baseframe);

  baseframe_from_msg_ = aligned_unique<vi_map::MissionBaseFrame>();
  baseframe_from_msg_->deserialize(initial_baseframe_->id(), proto_from_msg);
}

void ViwlsGraph::constructBaseframe() {
  T_G_M_.setRandom();
  T_G_M_covariance_.setRandom();
  mission_baseframe_id_.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1");

  initial_baseframe_ = aligned_unique<vi_map::MissionBaseFrame>(
      mission_baseframe_id_, T_G_M_, T_G_M_covariance_);
}

TEST_F(ViwlsGraph, ViwlsBaseframeSerializeAndDeserializeTest) {
  constructBaseframe();
  serializeAndDeserialize();

  EXPECT_EQ(initial_baseframe_->id(), baseframe_from_msg_->id());
  EXPECT_NEAR_ASLAM_TRANSFORMATION(
      initial_baseframe_->get_T_G_M(), baseframe_from_msg_->get_T_G_M(), 1e-15);
  EXPECT_NEAR_EIGEN(
      initial_baseframe_->get_T_G_M_Covariance(),
      baseframe_from_msg_->get_T_G_M_Covariance(), 1e-15);
}

MAPLAB_UNITTEST_ENTRYPOINT
