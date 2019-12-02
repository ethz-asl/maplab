#include <string>

#include <aslam/cameras/camera.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/vi-map.h"

namespace vi_map {

class MergeMapTest : public ::testing::Test {
 protected:
  MergeMapTest() {}
  virtual void SetUp() {
    test::generateMap<vi_map::TransformationEdge>(&map_);
    ASSERT_TRUE(checkMapConsistency(map_));

    test::generateSensorResourceIdsAndAddToAllMissions(&map_);
  }

  vi_map::VIMap map_, empty_map_;
};

TEST_F(MergeMapTest, MergeIntoEmptyMap) {
  ASSERT_TRUE(empty_map_.mergeAllMissionsFromMap(map_));
  EXPECT_TRUE(test::compareVIMap(map_, empty_map_));

  // Merge into empty map with no new ids should be the same as deep copy.
  vi_map::VIMap second_map;
  second_map.deepCopy(map_);
  EXPECT_TRUE(test::compareVIMap(empty_map_, second_map));
}

TEST_F(MergeMapTest, MergeIntoSameMap) {
  EXPECT_FALSE(map_.mergeAllMissionsFromMap(map_));
}

TEST_F(MergeMapTest, MergeTwoMissions) {
  map_.duplicateMission(map_.getIdOfFirstMission());
  EXPECT_EQ(2u, map_.numMissions());
  ASSERT_TRUE(empty_map_.mergeAllMissionsFromMap(map_));
  EXPECT_TRUE(test::compareVIMap(map_, empty_map_));
}

TEST_F(MergeMapTest, MergeIntoNonEmpty) {
  vi_map::VIMap second_map;
  test::generateMap<vi_map::TransformationEdge>(&second_map);
  const size_t num_vertices_before = second_map.numVertices();
  const size_t num_edges_before = second_map.numEdges();
  const size_t num_landmarks_before = second_map.numLandmarks();

  ASSERT_TRUE(second_map.mergeAllMissionsFromMap(map_));
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
    aslam::generateId(&new_edge_id);
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
  ASSERT_TRUE(empty_map_.mergeAllMissionsFromMap(map_));
  EXPECT_EQ(num_vertices_before, empty_map_.numVertices());
  EXPECT_EQ(num_edges_before, empty_map_.numEdges());
  EXPECT_EQ(num_landmarks_before, empty_map_.numLandmarks());
}

// Copies a map, then deletes the map's only mission and merges the mission back
// from the copy.
TEST_F(MergeMapTest, CopyDeleteMerge) {
  vi_map::VIMap map_copy;
  map_copy.deepCopy(map_);

  ASSERT_GT(map_copy.getSensorManager().getNumSensors(), 0u);
  ASSERT_EQ(
      map_copy.getSensorManager().getNumSensors(),
      map_.getSensorManager().getNumSensors());
  ASSERT_EQ(
      map_copy.getSensorManager().getNumSensorsOfType(SensorType::kNCamera),
      map_.getSensorManager().getNumSensorsOfType(SensorType::kNCamera));

  vi_map::MissionIdList all_mission_ids;
  map_.getAllMissionIds(&all_mission_ids);
  ASSERT_EQ(1u, all_mission_ids.size());
  const vi_map::MissionId& mission_id = all_mission_ids[0];

  constexpr bool kRemoveBaseframe = true;
  map_.removeMission(mission_id, kRemoveBaseframe);

  ASSERT_EQ(0u, map_.numMissions());
  EXPECT_EQ(0u, map_.numVertices());
  EXPECT_EQ(0u, map_.numLandmarks());

  // TODO(mfehr): We currently do not clean up the sensor manager when removing
  // missions, fix an reenable.
  // EXPECT_EQ(0u, map_.getSensorManager().getNumSensors());
  // EXPECT_EQ(0u,
  // map_.getSensorManager().getNumSensorsOfType(vi_map::SensorType::kNCamera));

  ASSERT_EQ(1u, map_copy.numMissions());
  ASSERT_TRUE(map_.mergeAllMissionsFromMap(map_copy));
  EXPECT_EQ(map_copy.numMissions(), map_.numMissions());
  EXPECT_EQ(map_copy.numVertices(), map_.numVertices());
  EXPECT_EQ(map_copy.numLandmarks(), map_.numLandmarks());
  EXPECT_EQ(
      map_copy.getSensorManager().getNumSensors(),
      map_.getSensorManager().getNumSensors());
  EXPECT_EQ(
      map_copy.getSensorManager().getNumSensorsOfType(
          vi_map::SensorType::kNCamera),
      map_.getSensorManager().getNumSensorsOfType(
          vi_map::SensorType::kNCamera));
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
