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

#include "vi-map/unique-id.h"
#include "vi-map/vi-mission.h"
#include "vi-map/vi_map.pb.h"

class ViwlsGraph : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    mission_id_.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa0");
    mission_base_frame_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1");

    n_camera_ = aslam::NCamera::createSurroundViewTestNCamera();
    back_bone_type_ = vi_map::Mission::BackBone::kOdometry;
  }

  void constructMission();
  void serializeAndDeserialize();

  vi_map::VIMission::UniquePtr initial_mission_;
  vi_map::VIMission::UniquePtr mission_from_msg_;

  vi_map::MissionId mission_id_;
  vi_map::MissionBaseFrameId mission_base_frame_id;
  vi_map::ImuSigmas imu_sigmas_;
  vi_map::Mission::BackBone back_bone_type_;
  aslam::NCamera::Ptr n_camera_;
};

void ViwlsGraph::serializeAndDeserialize() {
  vi_map::proto::Mission mission_proto, proto_from_msg;
  initial_mission_->serialize(&mission_proto);
  std::string serialized_mission = mission_proto.SerializeAsString();

  proto_from_msg.ParseFromString(serialized_mission);

  mission_from_msg_ = aligned_unique<vi_map::VIMission>();
  mission_from_msg_->deserialize(initial_mission_->id(), proto_from_msg);
}

void ViwlsGraph::constructMission() {
  initial_mission_ = aligned_unique<vi_map::VIMission>(
      mission_id_, mission_base_frame_id, back_bone_type_);
}

TEST_F(ViwlsGraph, ViwlsMissionSerializeAndDeserializeTest) {
  constructMission();
  serializeAndDeserialize();

  EXPECT_EQ(initial_mission_->id(), mission_from_msg_->id());
  EXPECT_EQ(
      initial_mission_->getBaseFrameId(), mission_from_msg_->getBaseFrameId());
  EXPECT_EQ(
      initial_mission_->getRootVertexId(),
      mission_from_msg_->getRootVertexId());
  EXPECT_EQ(
      initial_mission_->backboneType(), mission_from_msg_->backboneType());
  EXPECT_EQ(*initial_mission_, *mission_from_msg_);
}

TEST_F(ViwlsGraph, ViwlsMissionSerializeAndDeserializeNotEqualTest) {
  constructMission();
  serializeAndDeserialize();

  // Corrupt few members.
  mission_from_msg_->setOrdering(std::numeric_limits<int>::max());

  EXPECT_NE(*initial_mission_, *mission_from_msg_);
}

MAPLAB_UNITTEST_ENTRYPOINT
