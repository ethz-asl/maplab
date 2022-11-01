#include <Eigen/Core>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/test/vi-map-generator.h"

namespace vi_map {

class VIMapTest : public ::testing::Test {
 protected:
  VIMapTest() : map_(), generator_(map_, 42) {}

  virtual void SetUp() {
    pose::Transformation T_G_V;
    pose::Transformation T_G_M;

    missions_[0] = generator_.createMission(T_G_M);
    missions_[1] = generator_.createMission(T_G_M);
    // Add an empty mission with no vertices
    missions_[2] = generator_.createMission(T_G_M);

    vertices_[0] = generator_.createVertex(missions_[0], T_G_V, 11);
    vertices_[1] = generator_.createVertex(missions_[0], T_G_V, 12);
    vertices_[2] = generator_.createVertex(missions_[0], T_G_V, 13);
    vertices_[3] = generator_.createVertex(missions_[1], T_G_V, 3);
    vertices_[4] = generator_.createVertex(missions_[1], T_G_V, 5);
    vertices_[5] = generator_.createVertex(missions_[1], T_G_V, 7);

    generator_.generateMap();
  }

  vi_map::MissionId missions_[3];
  pose_graph::VertexId vertices_[6];

  VIMap map_;
  VIMapGenerator generator_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(VIMapTest, TestGetAllVertexIdsInMission) {
  pose_graph::VertexIdList vertex_ids;
  map_.getAllVertexIdsInMission(missions_[0], &vertex_ids);

  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    EXPECT_EQ(map_.getVertex(vertex_id).getMissionId(), missions_[0]);
  }
}

TEST_F(VIMapTest, TestGetAllVertexIdsInMissionAlongGraph) {
  pose_graph::VertexIdList vertex_ids;
  map_.getAllVertexIdsInMissionAlongGraph(missions_[0], &vertex_ids);

  EXPECT_FALSE(vertex_ids.empty());

  for (size_t i = 0; i < vertex_ids.size(); ++i) {
    EXPECT_EQ(map_.getVertex(vertex_ids[i]).getMissionId(), missions_[0]);
    EXPECT_EQ(vertex_ids[i], vertices_[i]);
  }
}

TEST_F(VIMapTest, TestGetAllVertexIdsAlongGraphsSortedByTimestamp) {
  pose_graph::VertexIdList vertex_ids;
  map_.getAllVertexIdsAlongGraphsSortedByTimestamp(&vertex_ids);
  // first make sure we actually have collected all vertices
  EXPECT_EQ(vertex_ids.size(), sizeof(vertices_) / sizeof(vertices_[0]));

  // Verify that the vertices of the second mission are correctly placed
  EXPECT_EQ(vertex_ids[0], vertices_[3]);
  EXPECT_EQ(vertex_ids[1], vertices_[4]);
  EXPECT_EQ(vertex_ids[2], vertices_[5]);

  EXPECT_EQ(vertex_ids[3], vertices_[0]);
  EXPECT_EQ(vertex_ids[4], vertices_[1]);
  EXPECT_EQ(vertex_ids[5], vertices_[2]);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
