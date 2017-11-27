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

    vertices_[0] = generator_.createVertex(missions_[0], T_G_V);
    vertices_[1] = generator_.createVertex(missions_[0], T_G_V);
    vertices_[2] = generator_.createVertex(missions_[0], T_G_V);
    vertices_[3] = generator_.createVertex(missions_[1], T_G_V);
    vertices_[4] = generator_.createVertex(missions_[1], T_G_V);
    vertices_[5] = generator_.createVertex(missions_[1], T_G_V);

    generator_.generateMap();
  }

  vi_map::MissionId missions_[2];
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

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
