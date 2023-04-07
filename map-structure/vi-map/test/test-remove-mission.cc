#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/check-map-consistency.h"
#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/vi-map.h"

namespace vi_map {

class RemoveMissionTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test::generateMap<vi_map::TransformationEdge>(&map_);
    ASSERT_TRUE(checkMapConsistency(map_));

    vi_map::MissionIdList mission_ids;
    map_.getAllMissionIds(&mission_ids);
    ASSERT_EQ(1u, mission_ids.size());
    mission_id_ = mission_ids.front();

    sensor_manager_ = &map_.getSensorManager();
    CHECK_NOTNULL(sensor_manager_);

    ASSERT_LT(0u, map_.numMissions());
    ASSERT_LT(0u, map_.numVertices());
    ASSERT_LT(0u, map_.numLandmarks());
    ASSERT_LT(0u, sensor_manager_->getNumSensors());
    ASSERT_LT(
        0u, sensor_manager_->getNumSensorsOfType(vi_map::SensorType::kNCamera));
  }

  virtual void TearDown() {
    EXPECT_TRUE(checkMapConsistency(map_));
  }

  MissionId generateSecondMission() {
    vi_map::VIMap second_map;
    test::generateMap<vi_map::TransformationEdge>(&second_map);

    const bool success = map_.mergeAllMissionsFromMap(second_map);
    EXPECT_TRUE(success);
    EXPECT_TRUE(checkMapConsistency(map_));

    vi_map::MissionIdList mission_ids;
    second_map.getAllMissionIds(&mission_ids);
    EXPECT_EQ(1u, mission_ids.size());
    return mission_ids.front();
  }

  static constexpr bool kRemoveBaseframe = true;

  VIMap map_;
  SensorManager* sensor_manager_;
  MissionId mission_id_;
};

TEST_F(RemoveMissionTest, RemoveFromSingleMissionMap) {
  map_.removeMission(mission_id_, kRemoveBaseframe);
  EXPECT_EQ(0u, map_.numMissions());
  EXPECT_EQ(0u, map_.numVertices());
  EXPECT_EQ(0u, map_.numLandmarks());
  // TODO(mfehr): We are currently not removing sensors when removing missions!
  // If this is fixed, we should reenable these checks!
  // EXPECT_EQ(0u, sensor_manager_->getNumSensors());
  // EXPECT_EQ(
  //     0u,
  //     sensor_manager_->getNumSensorsOfType(vi_map::SensorType::kNCamera));
}

TEST_F(RemoveMissionTest, RemoveFromMultiMissionMap) {
  generateSecondMission();
  ASSERT_TRUE(checkMapConsistency(map_));
  ASSERT_EQ(2u, map_.numMissions());
  map_.removeMission(mission_id_, kRemoveBaseframe);
  ASSERT_TRUE(checkMapConsistency(map_));
  ASSERT_EQ(1u, map_.numMissions());
}

TEST_F(RemoveMissionTest, DuplicateAndRemoveMissionWithInterconnectedSensors) {
  const size_t num_sensors_before_modification =
      sensor_manager_->getNumSensors();
  const size_t num_landmarks_before_modification = map_.numLandmarksInIndex();

  // Duplicate first mission.
  const MissionId duplicated_mission_id = map_.duplicateMission(mission_id_);

  // Check map consistency.
  ASSERT_EQ(2u, map_.numMissions());
  ASSERT_TRUE(checkMapConsistency(map_));
  EXPECT_EQ(
      num_sensors_before_modification * 2, sensor_manager_->getNumSensors());
  EXPECT_EQ(num_landmarks_before_modification * 2, map_.numLandmarksInIndex());

  // Remove duplicated mission.
  map_.removeMission(duplicated_mission_id, kRemoveBaseframe);

  // Check map consistency.
  ASSERT_TRUE(checkMapConsistency(map_));
  ASSERT_EQ(1u, map_.numMissions());
  // TODO(mfehr): We currently do not remove sensors when removing missions,
  // hence the cloned sensors remain!
  EXPECT_EQ(
      num_sensors_before_modification * 2, sensor_manager_->getNumSensors());
  EXPECT_EQ(num_landmarks_before_modification, map_.numLandmarksInIndex());
}

TEST_F(RemoveMissionTest, AddAndRemoveMissionWithoutInterconnectedSensors) {
  const size_t num_sensors_before_modification =
      sensor_manager_->getNumSensors();
  const size_t num_landmarks_before_modification = map_.numLandmarksInIndex();

  // Generate second mission.
  const MissionId second_mission_id = generateSecondMission();

  // Check map consistency.
  ASSERT_EQ(2u, map_.numMissions());
  ASSERT_TRUE(checkMapConsistency(map_));

  // Remove second mission.
  map_.removeMission(second_mission_id, kRemoveBaseframe);

  // Check map consistency.
  ASSERT_TRUE(checkMapConsistency(map_));
  ASSERT_EQ(1u, map_.numMissions());
  EXPECT_EQ(num_landmarks_before_modification, map_.numLandmarksInIndex());

  // TODO(mfehr): sensors are currently not removed with the mission, reenable
  // if this is fixed.
  // EXPECT_EQ(num_sensors_before_modification,
  // sensor_manager_->getNumSensors());
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
