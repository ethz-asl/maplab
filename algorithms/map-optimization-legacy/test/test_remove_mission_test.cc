#include <gtest/gtest.h>

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/check-map-consistency.h>

#include "map-optimization-legacy/test/6dof-vi-map-gen.h"

namespace map_optimization_legacy {

class RemoveMissionTest : public testing::Test {
 public:
  RemoveMissionTest() : generator_(), map_(generator_.vi_map_) {
    // Create a map with 2 missions.
    generator_.generateVIMap();
    first_mission_id_ = map_.getIdOfFirstMission();
    map_.duplicateMission(first_mission_id_);
  }

  void addCrossMissionObservations() {
    vi_map::MissionId other_mission_id;
    vi_map::MissionId first_mission_id = map_.getIdOfFirstMission();
    const vi_map::VIMission& first_mission = map_.getMission(first_mission_id);
    pose_graph::VertexId first_vertex_id = first_mission.getRootVertexId();

    // Get other mission ID.
    vi_map::MissionIdList mission_ids;
    map_.getAllMissionIds(&mission_ids);
    for (const vi_map::MissionId& mission_id : mission_ids) {
      if (mission_id != first_mission_id) {
        other_mission_id = mission_id;
        break;
      }
    }
    CHECK(other_mission_id.isValid());

    const vi_map::VIMission& other_mission = map_.getMission(other_mission_id);
    pose_graph::VertexId other_vertex_id = other_mission.getRootVertexId();

    // Merge a couple of landmarks.
    const unsigned int merge_every_nth_landmark = 5;
    unsigned int landmark_count = 0;
    do {
      vi_map::Vertex& first_vertex = map_.getVertex(first_vertex_id);
      vi_map::Vertex& other_vertex = map_.getVertex(other_vertex_id);

      vi_map::LandmarkIdList first_vertex_list, other_vertex_list;
      first_vertex.getStoredLandmarkIdList(&first_vertex_list);
      other_vertex.getStoredLandmarkIdList(&other_vertex_list);

      ASSERT_EQ(first_vertex_list.size(), other_vertex_list.size());

      for (size_t i = 0; i < first_vertex_list.size(); ++i) {
        if (landmark_count % merge_every_nth_landmark == 0) {
          map_.mergeLandmarks(first_vertex_list[i], other_vertex_list[i]);
        }
        ++landmark_count;
      }

      // Also swap observed landmark IDs of some landmarks so we have landmark
      // observations shared across missions too.
      vi_map::LandmarkIdList first_vertex_observed_landmark_list;
      vi_map::LandmarkIdList other_vertex_observed_landmark_list;
      first_vertex.getFrameObservedLandmarkIds(
          0, &first_vertex_observed_landmark_list);
      other_vertex.getFrameObservedLandmarkIds(
          0, &other_vertex_observed_landmark_list);

      const unsigned int swap_every_nth_landmark = 9;
      landmark_count = 0;
      for (size_t i = 0; i < first_vertex_observed_landmark_list.size(); ++i) {
        if (landmark_count % swap_every_nth_landmark == 0) {
          other_vertex.setObservedLandmarkId(0, i, vi_map::LandmarkId());
          map_.getLandmark(other_vertex_observed_landmark_list[i])
              .removeAllObservationsOfVertexAndFrame(other_vertex_id, 0);
          map_.associateKeypointWithExistingLandmark(
              other_vertex_id, 0, i, first_vertex_observed_landmark_list[i]);
        }
        ++landmark_count;
      }
    } while (map_.getNextVertex(
                 first_vertex_id, pose_graph::Edge::EdgeType::kViwls,
                 &first_vertex_id) &&
             map_.getNextVertex(
                 other_vertex_id, pose_graph::Edge::EdgeType::kViwls,
                 &other_vertex_id));
  }

  void addNonChronologicalLandmarkMerges() {
    vi_map::MissionIdList mission_ids;
    map_.getAllMissionIds(&mission_ids);

    for (const vi_map::MissionId& mission_id : mission_ids) {
      pose_graph::VertexIdList mission_vertex_ids;
      map_.getAllVertexIdsInMission(mission_id, &mission_vertex_ids);

      CHECK_GT(mission_vertex_ids.size(), 3u);
      const pose_graph::VertexId& first_vertex_id = mission_vertex_ids[0];
      const pose_graph::VertexId& second_vertex_id = mission_vertex_ids[2];

      CHECK_GT(map_.getVertex(first_vertex_id).getLandmarks().size(), 0u);
      CHECK_GT(map_.getVertex(second_vertex_id).getLandmarks().size(), 0u);

      const vi_map::LandmarkId first_vertex_landmark =
          map_.getVertex(first_vertex_id).getLandmarks().begin()->id();
      const vi_map::LandmarkId second_vertex_landmark =
          map_.getVertex(second_vertex_id).getLandmarks().begin()->id();

      map_.mergeLandmarks(first_vertex_landmark, second_vertex_landmark);
    }
  }

 protected:
  SixDofVIMapGenerator generator_;
  vi_map::VIMap& map_;
  vi_map::MissionId first_mission_id_;
};

TEST_F(RemoveMissionTest, MapConsistencyTest) {
  addCrossMissionObservations();
  ASSERT_TRUE(checkMapConsistency(map_));
  EXPECT_EQ(map_.numMissions(), 2u);
  map_.removeMission(first_mission_id_, true);
  EXPECT_TRUE(checkMapConsistency(map_));
  EXPECT_EQ(map_.numMissions(), 1u);
}

TEST_F(RemoveMissionTest, MapConsistencyTestWithMergedLandmarkObservations) {
  addCrossMissionObservations();
  addNonChronologicalLandmarkMerges();
  ASSERT_TRUE(checkMapConsistency(map_));
  EXPECT_EQ(map_.numMissions(), 2u);
  map_.removeMission(first_mission_id_, true);
  EXPECT_TRUE(checkMapConsistency(map_));
  EXPECT_EQ(map_.numMissions(), 1u);
}

}  // namespace map_optimization_legacy

MAPLAB_UNITTEST_ENTRYPOINT
