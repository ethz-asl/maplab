#include <memory>

#include <Eigen/Core>
#include <aslam/common/hash-id.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <posegraph/edge.h>
#include <posegraph/example/edge.h>
#include <posegraph/example/pose-graph.h>
#include <posegraph/example/vertex.h>
#include <posegraph/vertex.h>

namespace pose_graph {
namespace example {

TEST(AslamPosegraph, DataStructureSizes) {
  PoseGraph pose_graph;
  VertexIdList vertex_ids;
  EdgeIdList edge_ids;
  pose_graph.getAllVertexIds(&vertex_ids);
  pose_graph.getAllEdgeIds(&edge_ids);
  EXPECT_TRUE(vertex_ids.empty());
  EXPECT_TRUE(edge_ids.empty());

  pose_graph::VertexId vertex1;
  CHECK(vertex1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa0"));
  pose_graph.addVertex(vertex1);
  pose_graph.getAllVertexIds(&vertex_ids);
  pose_graph.getAllEdgeIds(&edge_ids);
  EXPECT_TRUE(edge_ids.empty());
  EXPECT_EQ(1u, vertex_ids.size());

  pose_graph::VertexId vertex2;
  CHECK(vertex2.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1"));
  pose_graph.addVertex(vertex2);
  pose_graph.getAllVertexIds(&vertex_ids);
  pose_graph.getAllEdgeIds(&edge_ids);
  EXPECT_TRUE(edge_ids.empty());
  EXPECT_EQ(2u, vertex_ids.size());

  pose_graph::EdgeId edge1;
  CHECK(edge1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2"));
  pose_graph.addEdge(vertex1, vertex2, edge1);
  pose_graph.getAllVertexIds(&vertex_ids);
  pose_graph.getAllEdgeIds(&edge_ids);
  EXPECT_EQ(1u, edge_ids.size());
  EXPECT_EQ(2u, vertex_ids.size());
}

TEST(AslamPosegraph, StaticOperators) {
  PoseGraph pose_graph;

  pose_graph::VertexId vertex1;
  pose_graph::VertexId vertex2;
  pose_graph::VertexId vertex3;
  CHECK(vertex1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1"));
  CHECK(vertex2.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2"));
  CHECK(vertex3.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa3"));
  pose_graph.addVertex(vertex1);
  pose_graph.addVertex(vertex2);
  pose_graph.addVertex(vertex3);

  pose_graph::EdgeId edge1;
  pose_graph::EdgeId edge2;
  pose_graph::EdgeId edge3;
  CHECK(edge1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa4"));
  CHECK(edge2.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa5"));
  CHECK(edge3.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa6"));
  pose_graph.addEdge(vertex1, vertex2, edge1);
  pose_graph.addEdge(vertex2, vertex3, edge2);
  pose_graph.addEdge(vertex3, vertex1, edge3);

  VertexIdList vertex_ids;
  EdgeIdList edge_ids;
  pose_graph.getAllVertexIds(&vertex_ids);
  pose_graph.getAllEdgeIds(&edge_ids);
  EXPECT_EQ(3u, edge_ids.size());
  EXPECT_EQ(3u, vertex_ids.size());

  EXPECT_TRUE(pose_graph.vertexExists(vertex1));
  EXPECT_TRUE(pose_graph.vertexExists(vertex2));
  EXPECT_TRUE(pose_graph.vertexExists(vertex3));

  EXPECT_TRUE(pose_graph.edgeExists(edge1));
  EXPECT_TRUE(pose_graph.edgeExists(edge2));
  EXPECT_TRUE(pose_graph.edgeExists(edge3));

  pose_graph::VertexId dummy_vertex_id;
  CHECK(dummy_vertex_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaada33e0"));
  Vertex dummy_vertex(dummy_vertex_id);
  EXPECT_FALSE(pose_graph.vertexExists(dummy_vertex.id()));

  pose_graph::EdgeId dummy_edge_id;
  CHECK(dummy_edge_id.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaada33e1"));
  Edge dummy_edge(dummy_edge_id);
  EXPECT_FALSE(pose_graph.edgeExists(dummy_edge.id()));

  std::pair<pose_graph::VertexId, pose_graph::VertexId> edge1_incident_vertices;
  std::pair<pose_graph::VertexId, pose_graph::VertexId> edge2_incident_vertices;
  std::pair<pose_graph::VertexId, pose_graph::VertexId> edge3_incident_vertices;

  pose_graph.getEdge(edge1).incidentVertices(&edge1_incident_vertices);
  pose_graph.getEdge(edge2).incidentVertices(&edge2_incident_vertices);
  pose_graph.getEdge(edge3).incidentVertices(&edge3_incident_vertices);

  EXPECT_EQ(vertex1, edge1_incident_vertices.first);
  EXPECT_EQ(vertex2, edge1_incident_vertices.second);

  EXPECT_EQ(vertex2, edge2_incident_vertices.first);
  EXPECT_EQ(vertex3, edge2_incident_vertices.second);

  EXPECT_EQ(vertex3, edge3_incident_vertices.first);
  EXPECT_EQ(vertex1, edge3_incident_vertices.second);
}

TEST(AslamPosegraph, IncidentEdges) {
  PoseGraph pose_graph;
  pose_graph::VertexId vertex1;
  pose_graph::VertexId vertex2;
  pose_graph::VertexId vertex3;
  pose_graph::VertexId vertex4;
  CHECK(vertex1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1"));
  CHECK(vertex2.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2"));
  CHECK(vertex3.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa3"));
  CHECK(vertex4.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa4"));
  pose_graph.addVertex(vertex1);
  pose_graph.addVertex(vertex2);
  pose_graph.addVertex(vertex3);
  pose_graph.addVertex(vertex4);

  pose_graph::EdgeId edge1;
  pose_graph::EdgeId edge2;
  pose_graph::EdgeId edge3;
  CHECK(edge1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa5"));
  CHECK(edge2.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa6"));
  CHECK(edge3.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa7"));
  pose_graph.addEdge(vertex1, vertex2, edge1);
  pose_graph.addEdge(vertex1, vertex3, edge2);
  pose_graph.addEdge(vertex4, vertex1, edge3);

  VertexIdList vertex_ids;
  EdgeIdList edge_ids;
  pose_graph.getAllVertexIds(&vertex_ids);
  pose_graph.getAllEdgeIds(&edge_ids);
  EXPECT_EQ(3u, edge_ids.size());
  EXPECT_EQ(4u, vertex_ids.size());

  std::unordered_set<pose_graph::EdgeId> vertex1_incident_edges;
  pose_graph.getVertex(vertex1).incidentEdges(&vertex1_incident_edges);

  EXPECT_EQ(3u, vertex1_incident_edges.size());
  EXPECT_NE(vertex1_incident_edges.find(edge1), vertex1_incident_edges.end());
  EXPECT_NE(vertex1_incident_edges.find(edge2), vertex1_incident_edges.end());
  EXPECT_NE(vertex1_incident_edges.find(edge3), vertex1_incident_edges.end());
}

TEST(AslamPosegraph, ElementPointers) {
  PoseGraph pose_graph;

  pose_graph::VertexId vertex1;
  pose_graph::VertexId vertex2;
  CHECK(vertex1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1"));
  CHECK(vertex2.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2"));
  pose_graph.addVertex(vertex1);
  pose_graph.addVertex(vertex2);

  pose_graph::EdgeId edge1;
  CHECK(edge1.fromHexString("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa3"));
  pose_graph.addEdge(vertex1, vertex2, edge1);

  const pose_graph::Vertex* vertex1_ptr = pose_graph.getVertexPtr(vertex1);
  const pose_graph::Vertex* vertex2_ptr = pose_graph.getVertexPtr(vertex2);
  const pose_graph::Edge* edge1_ptr = pose_graph.getEdgePtr(edge1);

  // assert, because we will segfault anyway if the pointers are NULL
  ASSERT_TRUE(vertex1_ptr != nullptr);
  ASSERT_TRUE(vertex2_ptr != nullptr);
  ASSERT_TRUE(edge1_ptr != nullptr);

  // compare the addresses (we expect that edges/vertices are not copied
  // when added to posegraph)
  EXPECT_EQ(vertex1_ptr, &pose_graph.getVertex(vertex1));
  EXPECT_EQ(vertex2_ptr, &pose_graph.getVertex(vertex2));
  EXPECT_EQ(edge1_ptr, &pose_graph.getEdge(edge1));
}

}  // namespace example
}  // namespace pose_graph

MAPLAB_UNITTEST_ENTRYPOINT
