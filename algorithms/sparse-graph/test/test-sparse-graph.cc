#include "sparse-graph/sparse-graph.h"

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

namespace spg {

class SparseGraphTest : public ::testing::Test {
 protected:
  SparseGraphTest() : ::testing::Test() {}

  virtual void SetUp() {}

  pose_graph::VertexIdList createRandomIds(const std::size_t n_ids) {
    pose_graph::VertexIdList all_vertices;

    for (std::size_t i = 0; i < n_ids; ++i) {
      pose_graph::VertexId id = aslam::createRandomId<pose_graph::VertexId>();
      all_vertices.emplace_back(std::move(id));
    }

    return all_vertices;
  }
};

TEST_F(SparseGraphTest, TestEmptyMissionMap) {
  vi_map::VIMap map;
  pose_graph::VertexIdList all_vertices;
  const std::string map_key = "foo";

  spg::SparseGraph graph(map);
  graph.addVerticesToMissionGraph(map_key, all_vertices);

  EXPECT_EQ(0u, graph.getMissionGraphSize(map_key));
}

TEST_F(SparseGraphTest, TestMissionMapSingleStorage) {
  vi_map::VIMap map;
  const std::size_t n_vertices = 5u;
  pose_graph::VertexIdList all_vertices = createRandomIds(n_vertices);
  const std::string map_key = "foo";

  spg::SparseGraph graph(map);
  graph.addVerticesToMissionGraph(map_key, all_vertices);

  EXPECT_EQ(n_vertices, graph.getMissionGraphSize(map_key));
}

TEST_F(SparseGraphTest, TestMissionMapMultipleStorage) {
  vi_map::VIMap map;
  const std::size_t n_vertices = 5u;
  pose_graph::VertexIdList all_vertices_1 = createRandomIds(n_vertices);
  pose_graph::VertexIdList all_vertices_2 = createRandomIds(n_vertices);
  const std::string map_key = "foo";

  spg::SparseGraph graph(map);
  graph.addVerticesToMissionGraph(map_key, all_vertices_1);
  graph.addVerticesToMissionGraph(map_key, all_vertices_2);

  EXPECT_EQ(2 * n_vertices, graph.getMissionGraphSize(map_key));
}

}  // namespace spg

MAPLAB_UNITTEST_ENTRYPOINT
