#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/check-map-consistency.h"
#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/vi-map.h"

namespace vi_map {

class RemoveMissionTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test::generateMap<vi_map::TransformationEdge>(&map_);
    test::generateOptionalSensorDataAndAddToMap(&map_);
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
    ASSERT_LT(0u, sensor_manager_->getNumNCameraSensors());
  }

  virtual void TearDown() {
    EXPECT_TRUE(checkMapConsistency(map_));
  }

  MissionId generateSecondMission() {
    vi_map::VIMap second_map;
    test::generateMap<vi_map::TransformationEdge>(&second_map);
    test::generateOptionalSensorDataAndAddToMap(&second_map);

    map_.mergeAllMissionsFromMap(second_map);
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
  EXPECT_EQ(0u, sensor_manager_->getNumSensors());
  EXPECT_EQ(0u, sensor_manager_->getNumNCameraSensors());
}

TEST_F(RemoveMissionTest, RemoveFromMultiMissionMap) {
  generateSecondMission();
  ASSERT_EQ(2u, map_.numMissions());
  map_.removeMission(mission_id_, kRemoveBaseframe);
  ASSERT_EQ(1u, map_.numMissions());
}

TEST_F(RemoveMissionTest, RemoveFromMultiMissionMapWithInterconnectedSensors) {
  const MissionId second_mission_id = generateSecondMission();
  ASSERT_EQ(2u, map_.numMissions());

  // Add some interconnections.
  const size_t num_sensors_before_modification =
      sensor_manager_->getNumSensors();
  SensorIdSet sensors_associated_with_first_mission;
  sensor_manager_->getAllSensorIdsAssociatedWithMission(
      mission_id_, &sensors_associated_with_first_mission);
  for (const SensorId& sensor_id : sensors_associated_with_first_mission) {
    sensor_manager_->associateExistingSensorWithMission(
        sensor_id, second_mission_id);
  }
  EXPECT_EQ(num_sensors_before_modification, sensor_manager_->getNumSensors());

  map_.removeMission(mission_id_, kRemoveBaseframe);
  ASSERT_EQ(1u, map_.numMissions());
  EXPECT_EQ(num_sensors_before_modification, sensor_manager_->getNumSensors());
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
