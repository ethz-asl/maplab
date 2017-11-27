#include <memory>
#include <random>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/unique-id.h>
#include <posegraph/edge.h>
#include <posegraph/vertex.h>

#include "vi-map/loopclosure-edge.h"
#include "vi-map/mission.h"
#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/vi-map.h"
#include "vi-map/vi_map.pb.h"

namespace vi_map {

class VIMapEdgeRemovalTest : public ::testing::Test {
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

  size_t numLoopClosureEdges() const;
  void addRandomLoopClosureEdges(const size_t number_of_edges);

  size_t removeLoopClosureEdges() {
    return map_.removeLoopClosureEdges();
  }

  vi_map::VIMap map_;
  pose_graph::VertexIdList initial_vertices_;
  pose_graph::EdgeIdList initial_edges_;
  vi_map::LandmarkIdSet initial_landmarks_;
  vi_map::MissionId initial_mission_;

 private:
  // Modify if test dataset with different vertex count used.
  static constexpr size_t kTestDataVertexCount = 405u;
  static constexpr double kSwitchVariable = 1.0;
  static constexpr double kSwitchVariableVariance = 1e-3;
};

size_t VIMapEdgeRemovalTest::numLoopClosureEdges() const {
  size_t number_of_loop_edges = 0u;
  pose_graph::EdgeIdList edges;
  map_.getAllEdgeIds(&edges);

  for (const pose_graph::EdgeId& edge : edges) {
    number_of_loop_edges += static_cast<int>(
        map_.getEdgeType(edge) == pose_graph::Edge::EdgeType::kLoopClosure);
  }
  return number_of_loop_edges;
}

void VIMapEdgeRemovalTest::addRandomLoopClosureEdges(
    const size_t number_of_edges_to_add) {
  Eigen::Matrix<double, 6, 6> T_A_B_covariance;
  pose::Transformation T_A_B;

  constexpr size_t kSeed = 85u;
  std::srand(kSeed);

  std::mt19937 integer_generator(kSeed);
  constexpr size_t kLowerBound = 0u;
  constexpr size_t kUpperBound = kTestDataVertexCount - 1u;
  std::uniform_int_distribution<> vertex_index_uniform_distribution(
      kLowerBound, kUpperBound);

  constexpr size_t kVerticesOffset = 10u;
  for (size_t edges_added = 0u; edges_added < number_of_edges_to_add;
       ++edges_added) {
    const size_t vertex_idx_from =
        vertex_index_uniform_distribution(integer_generator);
    const size_t vertex_idx_to =
        (vertex_idx_from + kVerticesOffset) % kTestDataVertexCount;
    ASSERT_NE(vertex_idx_from, vertex_idx_to);

    T_A_B.setRandom();
    T_A_B_covariance.setRandom();

    pose_graph::EdgeId edge_id;
    common::generateId(&edge_id);
    ASSERT_TRUE(edge_id.isValid());

    const pose_graph::VertexId& from = initial_vertices_[vertex_idx_from];
    const pose_graph::VertexId& to = initial_vertices_[vertex_idx_to];

    map_.addEdge(
        vi_map::Edge::UniquePtr(
            new LoopClosureEdge(
                edge_id, from, to, kSwitchVariable, kSwitchVariableVariance,
                T_A_B, T_A_B_covariance)));
  }
}

TEST_F(VIMapEdgeRemovalTest, RemoveLoopClosureEdgesTest) {
  EXPECT_EQ(numLoopClosureEdges(), 0u);

  constexpr size_t kNumNewLoopEdges = 100u;
  addRandomLoopClosureEdges(kNumNewLoopEdges);

  EXPECT_EQ(numLoopClosureEdges(), kNumNewLoopEdges);
  EXPECT_EQ(removeLoopClosureEdges(), kNumNewLoopEdges);
  EXPECT_EQ(numLoopClosureEdges(), 0u);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
