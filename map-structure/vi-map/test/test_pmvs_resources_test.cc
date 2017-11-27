#include <maplab-common/test/testing-entrypoint.h>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
#include <aslam/common/unique-id.h>
#include <gtest/gtest.h>
#include <map-resources/resource-common.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>
#include <vi-map/vi-mission.h>

namespace vi_map {

static const std::string kMapFolder = "test_map";

class PmvsResourcesTest : public ::testing::Test {
 public:
  PmvsResourcesTest() {}

  backend::ResourceType kType = backend::ResourceType::kPmvsReconstructionPath;

  static const std::string kPmvsPath0;
  static const std::string kPmvsPath1;
  static const std::string kPmvsPath5;

  static const MissionId kMissionId0;
  static const MissionId kMissionId1;
  static const MissionId kMissionId2;
  static const MissionId kMissionId3;

  MissionIdList involved_missions_0;
  MissionIdList involved_missions_1;
  MissionIdList involved_missions_2;
  MissionIdList involved_missions_3;
  MissionIdList involved_missions_4;
  MissionIdList involved_missions_5;

  VIMap vi_map_;

  virtual void SetUp() {
    vi_map_.setMapFolder(kMapFolder);

    involved_missions_0.push_back(kMissionId0);
    involved_missions_0.push_back(kMissionId1);
    involved_missions_0.push_back(kMissionId2);

    involved_missions_1.push_back(kMissionId2);
    involved_missions_1.push_back(kMissionId3);

    involved_missions_2.push_back(kMissionId2);

    involved_missions_3.push_back(kMissionId0);
    involved_missions_3.push_back(kMissionId1);
    involved_missions_3.push_back(kMissionId2);
    involved_missions_3.push_back(kMissionId3);

    involved_missions_4.push_back(kMissionId1);
    involved_missions_4.push_back(kMissionId2);
    involved_missions_4.push_back(kMissionId3);

    involved_missions_5.push_back(kMissionId0);

    // Dummy data.
    pose::Transformation T_G_M;
    aslam::TransformationVector T_C_B;
    Eigen::Matrix<double, 6, 6> T_G_M_covariance =
        Eigen::Matrix<double, 6, 6>::Identity();
    std::vector<aslam::Camera::Ptr> cameras;
    aslam::NCameraId n_camera_id = common::createRandomId<aslam::NCameraId>();
    std::string label = "nacmera";
    aslam::NCamera::Ptr n_camera_ptr(
        new aslam::NCamera(n_camera_id, T_C_B, cameras, label));

    vi_map_.addNewMissionWithBaseframe(
        kMissionId0, T_G_M, T_G_M_covariance, n_camera_ptr,
        Mission::BackBone::kViwls);
    vi_map_.addNewMissionWithBaseframe(
        kMissionId1, T_G_M, T_G_M_covariance, n_camera_ptr,
        Mission::BackBone::kViwls);
    vi_map_.addNewMissionWithBaseframe(
        kMissionId2, T_G_M, T_G_M_covariance, n_camera_ptr,
        Mission::BackBone::kViwls);
    vi_map_.addNewMissionWithBaseframe(
        kMissionId3, T_G_M, T_G_M_covariance, n_camera_ptr,
        Mission::BackBone::kViwls);

    vi_map_.storePmvsReconstructionPath(kPmvsPath0, involved_missions_0);
    vi_map_.storePmvsReconstructionPath(kPmvsPath5, involved_missions_5);
  }
};

const std::string PmvsResourcesTest::kPmvsPath0 = "test_0";
const std::string PmvsResourcesTest::kPmvsPath1 = "test_1";
const std::string PmvsResourcesTest::kPmvsPath5 = "test_5";

const MissionId PmvsResourcesTest::kMissionId0 =
    common::createRandomId<vi_map::MissionId>();
const MissionId PmvsResourcesTest::kMissionId1 =
    common::createRandomId<vi_map::MissionId>();
const MissionId PmvsResourcesTest::kMissionId2 =
    common::createRandomId<vi_map::MissionId>();
const MissionId PmvsResourcesTest::kMissionId3 =
    common::createRandomId<vi_map::MissionId>();

TEST_F(PmvsResourcesTest, TestViMissionResourceIdListSerialization) {
  const vi_map::VIMission& mission_0 = vi_map_.getMission(kMissionId0);

  backend::ResourceIdSet resource_ids_before;
  mission_0.getAllResourceIds(
      backend::ResourceType::kPmvsReconstructionPath, &resource_ids_before);

  EXPECT_EQ(2u, resource_ids_before.size());

  vi_map::proto::Mission proto_mission_0;
  mission_0.serialize(&proto_mission_0);

  vi_map::VIMission mission_0_deserialized;
  mission_0_deserialized.deserialize(kMissionId0, proto_mission_0);

  backend::ResourceIdSet resource_ids_after;
  mission_0_deserialized.getAllResourceIds(
      backend::ResourceType::kPmvsReconstructionPath, &resource_ids_after);

  EXPECT_EQ(resource_ids_after, resource_ids_before);
}

TEST_F(PmvsResourcesTest, TestHasPmvsReconstructionPath) {
  EXPECT_TRUE(vi_map_.hasPmvsReconstructionPath(involved_missions_0));
  EXPECT_FALSE(vi_map_.hasPmvsReconstructionPath(involved_missions_1));
  EXPECT_FALSE(vi_map_.hasPmvsReconstructionPath(involved_missions_2));
  EXPECT_FALSE(vi_map_.hasPmvsReconstructionPath(involved_missions_3));
  EXPECT_FALSE(vi_map_.hasPmvsReconstructionPath(involved_missions_4));
}

TEST_F(PmvsResourcesTest, TestGetPmvsReconstructionPath) {
  std::string path_0;
  std::string path_1;
  EXPECT_TRUE(vi_map_.getPmvsReconstructionPath(involved_missions_0, &path_0));
  EXPECT_EQ(path_0.compare(kPmvsPath0), 0);

  EXPECT_FALSE(vi_map_.getPmvsReconstructionPath(involved_missions_1, &path_1));
  EXPECT_TRUE(path_1.empty());
  EXPECT_FALSE(vi_map_.getPmvsReconstructionPath(involved_missions_2, &path_1));
  EXPECT_TRUE(path_1.empty());
  EXPECT_FALSE(vi_map_.getPmvsReconstructionPath(involved_missions_3, &path_1));
  EXPECT_TRUE(path_1.empty());
  EXPECT_FALSE(vi_map_.getPmvsReconstructionPath(involved_missions_4, &path_1));
  EXPECT_TRUE(path_1.empty());
}

TEST_F(PmvsResourcesTest, TestStorePmvsReconstructionPath) {
  vi_map_.storePmvsReconstructionPath(kPmvsPath1, involved_missions_1);

  std::string path_0;
  std::string path_1;
  std::string path_2;
  EXPECT_TRUE(vi_map_.getPmvsReconstructionPath(involved_missions_0, &path_0));
  EXPECT_EQ(path_0.compare(kPmvsPath0), 0);
  EXPECT_TRUE(vi_map_.getPmvsReconstructionPath(involved_missions_1, &path_1));
  EXPECT_EQ(path_1.compare(kPmvsPath1), 0);

  EXPECT_FALSE(vi_map_.getPmvsReconstructionPath(involved_missions_2, &path_2));
  EXPECT_TRUE(path_2.empty());
  EXPECT_FALSE(vi_map_.getPmvsReconstructionPath(involved_missions_3, &path_2));
  EXPECT_TRUE(path_2.empty());
  EXPECT_FALSE(vi_map_.getPmvsReconstructionPath(involved_missions_4, &path_2));
  EXPECT_TRUE(path_2.empty());
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
