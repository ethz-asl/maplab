#include <Eigen/Core>
#include <aslam/common/hash-id.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <posegraph/example/edge.h>
#include <posegraph/example/pose-graph.h>
#include <posegraph/example/vertex.h>

using namespace pose_graph::example;  // NOLINT

class PosegraphDeathTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    CHECK(vertex1_.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1"));
    CHECK(vertex2_.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2"));
    pose_graph_.addVertex(vertex1_);
    pose_graph_.addVertex(vertex2_);

    CHECK(edge_.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa4"));
    pose_graph_.addEdge(vertex1_, vertex2_, edge_);
  }

  PoseGraph pose_graph_;
  pose_graph::EdgeId edge_;
  pose_graph::VertexId vertex1_;
  pose_graph::VertexId vertex2_;
};

TEST_F(PosegraphDeathTest, DuplicateVertex) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(pose_graph_.addVertex(vertex1_), ".*Vertex.*already exists.*");
}

TEST_F(PosegraphDeathTest, DuplicateEdge) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(
      pose_graph_.addEdge(vertex1_, vertex2_, edge_),
      ".*Edge.*already exists.*");
}

TEST_F(PosegraphDeathTest, EdgeToNonexistentVertex) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  pose_graph::VertexId some_vertex;
  CHECK(some_vertex.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa007"));

  pose_graph::EdgeId edge_to_be_added;
  CHECK(edge_to_be_added.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa008"));
  EXPECT_DEATH(
      pose_graph_.addEdge(vertex1_, some_vertex, edge_to_be_added),
      "Check failed: it != map.end()");
}

TEST_F(PosegraphDeathTest, GetNonexistentVertex) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  pose_graph::VertexId some_vertex;
  CHECK(some_vertex.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa007"));
  EXPECT_DEATH(
      pose_graph_.getVertex(some_vertex), ".*Vertex.*not in posegraph.*");
  EXPECT_DEATH(
      pose_graph_.getVertexPtr(some_vertex), "Check failed: it != map.end()");
}

TEST_F(PosegraphDeathTest, GetNonexistentEdge) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  pose_graph::EdgeId some_edge;
  CHECK(some_edge.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa007"));
  EXPECT_DEATH(pose_graph_.getEdge(some_edge), ".*Edge.*not in posegraph.*");
  EXPECT_DEATH(
      pose_graph_.getEdgePtr(some_edge), "Check failed: it != map.end()");
}

TEST_F(PosegraphDeathTest, CallEdgeExistsWithNonexistentVertex) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  pose_graph::VertexId some_vertex;
  CHECK(some_vertex.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaa007"));
  EXPECT_DEATH(
      pose_graph_.edgeExists(vertex1_, some_vertex),
      ".*Vertex.*not in posegraph.*");
}

MAPLAB_UNITTEST_ENTRYPOINT
