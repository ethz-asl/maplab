#include <string>

#include <Eigen/Core>

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/vi-map.h"

class VIMapMissionSelectionTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    vi_map::test::generateMap(kTestDataVertexCount, &map_);
    initial_mission_ = map_.getIdOfFirstMission();
    map_.getAllEdgeIds(&initial_edges_);
    map_.getAllVertexIds(&initial_vertices_);
    map_.getAllLandmarkIds(&initial_landmarks_);
  }

  size_t numVertices() const {
    return kTestDataVertexCount;
  }

  vi_map::VIMap map_;
  pose_graph::VertexIdList initial_vertices_;
  pose_graph::EdgeIdList initial_edges_;
  vi_map::LandmarkIdSet initial_landmarks_;
  vi_map::MissionId initial_mission_;

 private:
  // Modify if test dataset with different vertex count used.
  static constexpr size_t kTestDataVertexCount = 405u;
};

TEST_F(VIMapMissionSelectionTest, TestMissionVertexCount) {
  EXPECT_EQ(numVertices(), map_.numVertices());
}

TEST_F(VIMapMissionSelectionTest, TestMissionEdgeCount) {
  EXPECT_EQ(numVertices() - 1u, map_.numEdges());
}

TEST_F(VIMapMissionSelectionTest, DuplicatedMissionVertexEdgeCount) {
  map_.duplicateMission(initial_mission_);
  EXPECT_EQ(2u * numVertices(), map_.numVertices());
  EXPECT_EQ(2u * (numVertices() - 1u), map_.numEdges());
}

TEST_F(VIMapMissionSelectionTest, SelectedMissionVertexEdgeCount) {
  map_.duplicateMission(initial_mission_);

  vi_map::MissionIdSet initial_mission_set;
  initial_mission_set.emplace(initial_mission_);
  map_.selectMissions(initial_mission_set);
  EXPECT_EQ(numVertices(), map_.numVertices());
  EXPECT_EQ((numVertices() - 1u), map_.numEdges());
}

TEST_F(VIMapMissionSelectionTest, DeselectMission) {
  map_.duplicateMission(initial_mission_);

  vi_map::MissionIdSet initial_mission_set;
  initial_mission_set.emplace(initial_mission_);
  map_.selectMissions(initial_mission_set);
  EXPECT_EQ(numVertices(), map_.numVertices());
  EXPECT_EQ((numVertices() - 1u), map_.numEdges());

  map_.deselectMission(initial_mission_);
  EXPECT_EQ(2u * numVertices(), map_.numVertices());
  EXPECT_EQ(2u * (numVertices() - 1u), map_.numEdges());
}

TEST_F(VIMapMissionSelectionTest, DuplicatedSelectedMissionLandmarkCount) {
  map_.duplicateMission(initial_mission_);
  EXPECT_EQ(2u * initial_landmarks_.size(), map_.numLandmarks());

  vi_map::MissionIdSet initial_mission_set;
  initial_mission_set.emplace(initial_mission_);
  map_.selectMissions(initial_mission_set);
  EXPECT_EQ(initial_landmarks_.size(), map_.numLandmarks());
}

TEST_F(VIMapMissionSelectionTest, DuplicatedSelectedMissionLandmarkIds) {
  map_.duplicateMission(initial_mission_);
  EXPECT_EQ(2u * initial_landmarks_.size(), map_.numLandmarks());

  vi_map::MissionIdSet initial_mission_set;
  initial_mission_set.emplace(initial_mission_);
  map_.selectMissions(initial_mission_set);

  vi_map::LandmarkIdSet landmark_ids;
  map_.getAllLandmarkIds(&landmark_ids);
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    EXPECT_GT(initial_landmarks_.count(landmark_id), 0u);
    EXPECT_TRUE(map_.hasLandmark(landmark_id));
  }
}

TEST_F(VIMapMissionSelectionTest, SelectedMissionGetVertexEdgeIds) {
  EXPECT_TRUE(map_.hasMission(initial_mission_));
  map_.duplicateMission(initial_mission_);

  // Select the initial mission only.
  vi_map::MissionIdSet initial_mission_set;
  initial_mission_set.emplace(initial_mission_);

  // Find the id of a duplicated mission.
  vi_map::MissionIdList mission_ids;
  map_.getAllMissionIds(&mission_ids);
  vi_map::MissionId duplicated_mission;
  vi_map::MissionIdSet duplicated_mission_set;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    EXPECT_TRUE(map_.hasMission(mission_id));
    if (mission_id != initial_mission_) {
      duplicated_mission = mission_id;
      duplicated_mission_set.emplace(mission_id);
    }
  }
  CHECK_EQ(1u, duplicated_mission_set.size());
  CHECK(duplicated_mission.isValid());

  // Get all vertices of duplicated mission.
  pose_graph::VertexIdList duplicated_vertices;
  map_.getAllVertexIdsInMission(duplicated_mission, &duplicated_vertices);

  map_.selectMissions(initial_mission_set);

  for (const pose_graph::VertexId& vertex_id : initial_vertices_) {
    EXPECT_TRUE(map_.hasVertex(vertex_id));
  }
  for (const pose_graph::EdgeId& edge_id : initial_edges_) {
    EXPECT_TRUE(map_.hasEdge(edge_id));
  }
  for (const pose_graph::VertexId& vertex_id : duplicated_vertices) {
    EXPECT_FALSE(map_.hasVertex(vertex_id));
  }

  map_.deselectMission(initial_mission_);
  map_.selectMissions(duplicated_mission_set);

  for (const pose_graph::VertexId& vertex_id : initial_vertices_) {
    EXPECT_FALSE(map_.hasVertex(vertex_id));
  }
  for (const pose_graph::EdgeId& edge_id : initial_edges_) {
    EXPECT_FALSE(map_.hasEdge(edge_id));
  }
  for (const pose_graph::VertexId& vertex_id : duplicated_vertices) {
    EXPECT_TRUE(map_.hasVertex(vertex_id));
  }
}

TEST_F(VIMapMissionSelectionTest, SelectiveVIMapHasMission) {
  EXPECT_TRUE(map_.hasMission(initial_mission_));
  map_.duplicateMission(initial_mission_);

  vi_map::MissionIdList mission_ids;
  map_.getAllMissionIds(&mission_ids);

  vi_map::MissionId duplicated_mission;
  vi_map::MissionIdSet duplicated_mission_set;

  for (const vi_map::MissionId& mission_id : mission_ids) {
    EXPECT_TRUE(map_.hasMission(mission_id));
    if (mission_id != initial_mission_) {
      duplicated_mission = mission_id;
      duplicated_mission_set.emplace(mission_id);
    }
  }
  CHECK_EQ(1u, duplicated_mission_set.size());
  CHECK(duplicated_mission.isValid());

  // Select the initial mission only.
  vi_map::MissionIdSet initial_mission_set;
  initial_mission_set.emplace(initial_mission_);
  map_.selectMissions(initial_mission_set);

  // Check if initial mission is visible and duplicated one hidden.
  EXPECT_TRUE(map_.hasMission(initial_mission_));
  EXPECT_FALSE(map_.hasMission(duplicated_mission));

  // Deselect the initial mission.
  map_.deselectMission(initial_mission_);

  // Now both missions should be accessible.
  EXPECT_TRUE(map_.hasMission(initial_mission_));
  EXPECT_TRUE(map_.hasMission(duplicated_mission));

  // Select the duplicated mission.
  map_.selectMissions(duplicated_mission_set);
  EXPECT_FALSE(map_.hasMission(initial_mission_));
  EXPECT_TRUE(map_.hasMission(duplicated_mission));

  EXPECT_EQ(duplicated_mission, map_.getIdOfFirstMission());
}

TEST_F(VIMapMissionSelectionTest, GetAllVertexIdsInMissionDeathTest) {
  EXPECT_TRUE(map_.hasMission(initial_mission_));
  map_.duplicateMission(initial_mission_);

  vi_map::MissionIdList mission_ids;
  map_.getAllMissionIds(&mission_ids);
  vi_map::MissionId duplicated_mission;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    EXPECT_TRUE(map_.hasMission(mission_id));
    if (mission_id != initial_mission_) {
      duplicated_mission = mission_id;
    }
  }
  CHECK(duplicated_mission.isValid());

  pose_graph::VertexIdList vertices;
  map_.getAllVertexIdsInMission(duplicated_mission, &vertices);
  EXPECT_EQ(numVertices(), vertices.size());

  vi_map::MissionIdSet initial_mission_set;
  initial_mission_set.insert(initial_mission_);
  map_.selectMissions(initial_mission_set);

  const std::string kErrorMessage = "hasMission";
  EXPECT_DEATH(
      map_.getAllVertexIdsInMission(duplicated_mission, &vertices),
      kErrorMessage);
}

MAPLAB_UNITTEST_ENTRYPOINT
