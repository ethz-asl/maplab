#include <string>

#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/vi-map.h"

namespace vi_map {

class MergeMapTest : public ::testing::Test {
 protected:
  MergeMapTest() {}
  virtual void SetUp() {
    test::generateMap(&map_);
    ASSERT_TRUE(checkMapConsistency(map_));
  }

  vi_map::VIMap map_, empty_map_;
};

TEST_F(MergeMapTest, MergeIntoEmptyMap) {
  empty_map_.mergeAllMissionsFromMap(map_);
  EXPECT_TRUE(test::compareVIMap(map_, empty_map_));

  // Merge into empty map with no new ids should be the same as deep copy.
  vi_map::VIMap second_map;
  second_map.deepCopy(map_);
  EXPECT_TRUE(test::compareVIMap(empty_map_, second_map));
}

TEST_F(MergeMapTest, MergeIntoSameMap) {
  const std::string kErrorMessage =
      "NCamera with id .* is already associated with mission .*.";
  EXPECT_DEATH(map_.mergeAllMissionsFromMap(map_), kErrorMessage);
}

TEST_F(MergeMapTest, MergeTwoMissions) {
  map_.duplicateMission(map_.getIdOfFirstMission());
  EXPECT_EQ(2u, map_.numMissions());
  empty_map_.mergeAllMissionsFromMap(map_);
  EXPECT_TRUE(test::compareVIMap(map_, empty_map_));
}

TEST_F(MergeMapTest, MergeIntoNonEmpty) {
  vi_map::VIMap second_map;
  test::generateMap(&second_map);
  const size_t num_vertices_before = second_map.numVertices();
  const size_t num_edges_before = second_map.numEdges();
  const size_t num_landmarks_before = second_map.numLandmarks();

  second_map.mergeAllMissionsFromMap(map_);
  EXPECT_EQ(2u, second_map.numMissions());
  EXPECT_EQ(num_vertices_before + map_.numVertices(), second_map.numVertices());
  EXPECT_EQ(num_edges_before + map_.numEdges(), second_map.numEdges());
  EXPECT_EQ(
      num_landmarks_before + map_.numLandmarks(), second_map.numLandmarks());
}

TEST_F(MergeMapTest, MergeMapWithTwoLinkedMissions) {
  vi_map::MissionIdList all_mission_ids;
  map_.getAllMissionIds(&all_mission_ids);
  ASSERT_EQ(1u, all_mission_ids.size());
  map_.duplicateMission(all_mission_ids[0]);

  map_.getAllMissionIds(&all_mission_ids);
  ASSERT_EQ(2u, all_mission_ids.size());

  // Create an edge between mission 0 and mission 1.
  {
    const pose_graph::VertexId& root_vertex_mission_0 =
        map_.getMission(all_mission_ids[0]).getRootVertexId();
    const pose_graph::VertexId& root_vertex_mission_1 =
        map_.getMission(all_mission_ids[1]).getRootVertexId();
    pose_graph::EdgeId new_edge_id;
    common::generateId(&new_edge_id);
    vi_map::LoopClosureEdge::UniquePtr loop_closure_edge =
        aligned_unique<vi_map::LoopClosureEdge>();
    loop_closure_edge->setId(new_edge_id);
    loop_closure_edge->setFrom(root_vertex_mission_0);
    loop_closure_edge->setTo(root_vertex_mission_1);
    map_.addEdge(std::move(loop_closure_edge));
  }

  const size_t num_vertices_before = map_.numVertices();
  const size_t num_edges_before = map_.numEdges();
  const size_t num_landmarks_before = map_.numLandmarks();

  // Merge into empty map.
  empty_map_.mergeAllMissionsFromMap(map_);
  EXPECT_EQ(num_vertices_before, empty_map_.numVertices());
  EXPECT_EQ(num_edges_before, empty_map_.numEdges());
  EXPECT_EQ(num_landmarks_before, empty_map_.numLandmarks());
}

//  OptionalSensorData related unit tests.
TEST_F(MergeMapTest, MergeIntoEmptyMapGPS) {
  test::generateOptionalSensorDataAndAddToMap(&map_);
  empty_map_.mergeAllMissionsFromMap(map_);
  MissionIdList mission_ids;
  map_.getAllMissionIds(&mission_ids);
  MissionIdList merged_map_mission_ids;
  empty_map_.getAllMissionIds(&merged_map_mission_ids);
  EXPECT_EQ(mission_ids, merged_map_mission_ids);

  const SensorManager& sensor_manager = map_.getSensorManager();
  const SensorManager& sensor_manager_empty_map = empty_map_.getSensorManager();
  for (const MissionId& mission_id : mission_ids) {
    CHECK(mission_id.isValid());
    ASSERT_EQ(
        sensor_manager.getNumSensorsOfTypeAssociatedWithMission<GpsUtm>(
            mission_id),
        1u);
    ASSERT_EQ(
        sensor_manager.getNumSensorsOfTypeAssociatedWithMission<GpsWgs>(
            mission_id),
        1u);
    ASSERT_EQ(
        sensor_manager_empty_map
            .getNumSensorsOfTypeAssociatedWithMission<GpsUtm>(mission_id),
        1u);
    ASSERT_EQ(
        sensor_manager_empty_map
            .getNumSensorsOfTypeAssociatedWithMission<GpsWgs>(mission_id),
        1u);
    const GpsUtm& map_gps_utm_sensor =
        sensor_manager.getSensorForMission<GpsUtm>(mission_id);
    const GpsWgs& map_gps_wgs_sensor =
        sensor_manager.getSensorForMission<GpsWgs>(mission_id);
    const GpsUtm& empty_map_gps_utm_sensor =
        sensor_manager_empty_map.getSensorForMission<GpsUtm>(mission_id);
    const GpsWgs& empty_map_gps_wgs_sensor =
        sensor_manager_empty_map.getSensorForMission<GpsWgs>(mission_id);
    EXPECT_EQ(map_gps_utm_sensor.getId(), empty_map_gps_utm_sensor.getId());
    EXPECT_EQ(map_gps_wgs_sensor.getId(), empty_map_gps_wgs_sensor.getId());

    EXPECT_EQ(
        map_.getOptionalSensorData(mission_id),
        empty_map_.getOptionalSensorData(mission_id));
  }
}

TEST_F(MergeMapTest, MergeIntoNonEmptyOptionalSensorData) {
  test::generateOptionalSensorDataAndAddToMap(&map_);
  vi_map::GpsWgsMeasurementBuffer map_gps_wgs_measurements_before;
  vi_map::GpsUtmMeasurementBuffer map_gps_utm_measurements_before;

  MissionIdList map_mission_ids;
  map_.getAllMissionIds(&map_mission_ids);

  const SensorManager& sensor_manager = map_.getSensorManager();
  for (const MissionId& mission_id : map_mission_ids) {
    CHECK(mission_id.isValid());
    ASSERT_EQ(
        sensor_manager.getNumSensorsOfTypeAssociatedWithMission<GpsUtm>(
            mission_id),
        1u);
    ASSERT_EQ(
        sensor_manager.getNumSensorsOfTypeAssociatedWithMission<GpsWgs>(
            mission_id),
        1u);
    const GpsUtm& map_gps_utm_sensor =
        sensor_manager.getSensorForMission<GpsUtm>(mission_id);
    const GpsWgs& map_gps_wgs_sensor =
        sensor_manager.getSensorForMission<GpsWgs>(mission_id);

    const vi_map::GpsWgsMeasurementBuffer mission_gps_wgs_measurements_before =
        map_.getOptionalSensorData(mission_id)
            .getMeasurements<GpsWgsMeasurement>(map_gps_wgs_sensor.getId());
    map_gps_wgs_measurements_before.insert(mission_gps_wgs_measurements_before);

    const vi_map::GpsUtmMeasurementBuffer mission_gps_utm_measurements_before =
        map_.getOptionalSensorData(mission_id)
            .getMeasurements<GpsUtmMeasurement>(map_gps_utm_sensor.getId());
    map_gps_utm_measurements_before.insert(mission_gps_utm_measurements_before);
  }

  vi_map::VIMap second_map;
  test::generateMap(&second_map);
  test::generateOptionalSensorDataAndAddToMap(&second_map);

  vi_map::GpsWgsMeasurementBuffer second_map_gps_wgs_measurements_before;
  vi_map::GpsUtmMeasurementBuffer second_map_gps_utm_measurements_before;

  MissionIdList second_map_mission_ids_before;
  second_map.getAllMissionIds(&second_map_mission_ids_before);

  const SensorManager& sensor_manager_second_map =
      second_map.getSensorManager();
  for (const MissionId& mission_id : second_map_mission_ids_before) {
    CHECK(mission_id.isValid());
    ASSERT_EQ(
        sensor_manager_second_map
            .getNumSensorsOfTypeAssociatedWithMission<GpsUtm>(mission_id),
        1u);
    ASSERT_EQ(
        sensor_manager_second_map
            .getNumSensorsOfTypeAssociatedWithMission<GpsWgs>(mission_id),
        1u);
    const GpsUtm& second_map_gps_utm_sensor =
        sensor_manager_second_map.getSensorForMission<GpsUtm>(mission_id);
    const GpsWgs& second_map_gps_wgs_sensor =
        sensor_manager_second_map.getSensorForMission<GpsWgs>(mission_id);

    const vi_map::GpsUtmMeasurementBuffer mission_gps_utm_measurements_before =
        second_map.getOptionalSensorData(mission_id)
            .getMeasurements<GpsUtmMeasurement>(
                second_map_gps_utm_sensor.getId());
    second_map_gps_utm_measurements_before.insert(
        mission_gps_utm_measurements_before);

    const vi_map::GpsWgsMeasurementBuffer mission_gps_wgs_measurements_before =
        second_map.getOptionalSensorData(mission_id)
            .getMeasurements<GpsWgsMeasurement>(
                second_map_gps_wgs_sensor.getId());
    second_map_gps_wgs_measurements_before.insert(
        mission_gps_wgs_measurements_before);
  }

  second_map.mergeAllMissionsFromMap(map_);

  vi_map::GpsWgsMeasurementBuffer second_map_gps_wgs_measurements_after;
  vi_map::GpsUtmMeasurementBuffer second_map_gps_utm_measurements_after;

  MissionIdList second_map_mission_ids_after;
  second_map.getAllMissionIds(&second_map_mission_ids_after);
  const size_t second_map_num_missions_after =
      second_map_mission_ids_after.size();

  for (const MissionId& mission_id : second_map_mission_ids_after) {
    CHECK(mission_id.isValid());
    ASSERT_EQ(
        sensor_manager_second_map
            .getNumSensorsOfTypeAssociatedWithMission<GpsUtm>(mission_id),
        1u);
    ASSERT_EQ(
        sensor_manager_second_map
            .getNumSensorsOfTypeAssociatedWithMission<GpsWgs>(mission_id),
        1u);
    const GpsUtm& map_gps_utm_sensor =
        sensor_manager_second_map.getSensorForMission<GpsUtm>(mission_id);
    const GpsWgs& map_gps_wgs_sensor =
        sensor_manager_second_map.getSensorForMission<GpsWgs>(mission_id);

    const vi_map::GpsUtmMeasurementBuffer mission_gps_utm_measurements_after =
        second_map.getOptionalSensorData(mission_id)
            .getMeasurements<GpsUtmMeasurement>(map_gps_utm_sensor.getId());
    second_map_gps_utm_measurements_after.insert(
        mission_gps_utm_measurements_after);

    const vi_map::GpsWgsMeasurementBuffer mission_gps_wgs_measurements_after =
        second_map.getOptionalSensorData(mission_id)
            .getMeasurements<GpsWgsMeasurement>(map_gps_wgs_sensor.getId());
    second_map_gps_wgs_measurements_after.insert(
        mission_gps_wgs_measurements_after);
  }

  EXPECT_EQ(
      second_map_gps_wgs_measurements_before.size() +
          map_gps_wgs_measurements_before.size(),
      second_map_gps_wgs_measurements_after.size());
  EXPECT_EQ(
      second_map_gps_utm_measurements_before.size() +
          map_gps_utm_measurements_before.size(),
      second_map_gps_utm_measurements_after.size());

  vi_map::GpsWgsMeasurementBuffer gps_wgs_measurements_before;
  gps_wgs_measurements_before.insert(map_gps_wgs_measurements_before);
  gps_wgs_measurements_before.insert(second_map_gps_wgs_measurements_before);
  EXPECT_EQ(gps_wgs_measurements_before, second_map_gps_wgs_measurements_after);

  vi_map::GpsUtmMeasurementBuffer gps_utm_measurements_before;
  gps_utm_measurements_before.insert(map_gps_utm_measurements_before);
  gps_utm_measurements_before.insert(second_map_gps_utm_measurements_before);
  EXPECT_EQ(gps_utm_measurements_before, second_map_gps_utm_measurements_after);

  EXPECT_EQ(
      map_mission_ids.size() + second_map_mission_ids_before.size(),
      second_map_mission_ids_after.size());
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
