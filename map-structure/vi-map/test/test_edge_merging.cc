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
#include <posegraph/unique-id.h>
#include <posegraph/vertex.h>
#include <sensors/sensor-factory.h>

#include "vi-map/mission.h"
#include "vi-map/test/vi-map-test-helpers.h"
#include "vi-map/transformation-edge.h"
#include "vi-map/vi-map.h"
#include "vi-map/viwls-edge.h"

namespace vi_map {

class VIMapEdgeMergingTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  virtual void SetUp() {
    vi_map::test::generateMap<ViwlsEdge>(kTestDataVertexCount, &map_);
    CHECK_EQ(map_.numMissions(), 1u);
    CHECK_EQ(map_.numVertices(), kTestDataVertexCount);
    mission_id_ = map_.getIdOfFirstMission();
  }

  TransformationEdge getOutgoingTransformationEdge(
      const pose_graph::VertexId& vertex_id) const;

  ViwlsEdge getOutgoingViwlsEdge(const pose_graph::VertexId& vertex_id) const;

  void mergeRandomVertex(
      const pose_graph::VertexId& vertex_id_from,
      const bool has_wheel_odometry_edge);

  void addWheelOdometryEdges();

  vi_map::VIMap map_;
  vi_map::MissionId mission_id_;

 private:
  // Modify if test dataset with different vertex count used.
  static constexpr size_t kTestDataVertexCount = 405u;
};

void VIMapEdgeMergingTest::addWheelOdometryEdges() {
  CHECK(mission_id_.isValid());

  Sensor::UniquePtr wheel_odometry_sensor =
      createTestSensor(SensorType::kRelative6DoFPose);
  CHECK(wheel_odometry_sensor);
  const SensorId wheel_odometry_sensor_id = wheel_odometry_sensor->getId();
  map_.getSensorManager().addSensor(
      std::move(wheel_odometry_sensor), mission_id_);
  CHECK(wheel_odometry_sensor_id.isValid());

  const pose_graph::Edge::EdgeType traversal_edge_type =
      map_.getGraphTraversalEdgeType(mission_id_);

  pose_graph::VertexId vertex_id_k =
      map_.getMission(mission_id_).getRootVertexId();
  CHECK(vertex_id_k.isValid());

  aslam::Transformation T_G_Ik = map_.getVertex_T_G_I(vertex_id_k);

  pose_graph::VertexId vertex_id_kp1;
  while (map_.getNextVertex(vertex_id_k, traversal_edge_type, &vertex_id_kp1)) {
    CHECK(vertex_id_kp1.isValid());

    const aslam::Transformation T_G_Ikp1 = map_.getVertex_T_G_I(vertex_id_kp1);

    const aslam::Transformation T_Ik_Ikp1 = T_G_Ik.inverse() * T_G_Ikp1;
    TransformationEdge::UniquePtr transformation_edge =
        aligned_unique<TransformationEdge>(
            Edge::EdgeType::kOdometry,
            common::createRandomId<pose_graph::EdgeId>(), vertex_id_k,
            vertex_id_kp1, T_Ik_Ikp1,
            aslam::TransformationCovariance::Identity(),
            wheel_odometry_sensor_id);

    map_.addEdge(std::move(transformation_edge));

    T_G_Ik = T_G_Ikp1;
    vertex_id_k = vertex_id_kp1;
  }
}

TransformationEdge VIMapEdgeMergingTest::getOutgoingTransformationEdge(
    const pose_graph::VertexId& vertex_id) const {
  CHECK(vertex_id.isValid());
  pose_graph::EdgeIdSet outgoing_edge_ids;
  map_.getVertex(vertex_id).getOutgoingEdges(&outgoing_edge_ids);

  CHECK(!outgoing_edge_ids.empty());

  pose_graph::EdgeId outgoing_transformation_edge_id;
  outgoing_transformation_edge_id.setInvalid();
  for (const pose_graph::EdgeId& edge_id : outgoing_edge_ids) {
    CHECK(edge_id.isValid());
    if (map_.getEdgeType(edge_id) == Edge::EdgeType::kOdometry) {
      outgoing_transformation_edge_id = edge_id;
    }
  }
  CHECK(outgoing_transformation_edge_id.isValid());

  return map_.getEdgeAs<TransformationEdge>(outgoing_transformation_edge_id);
}

ViwlsEdge VIMapEdgeMergingTest::getOutgoingViwlsEdge(
    const pose_graph::VertexId& vertex_id) const {
  CHECK(vertex_id.isValid());
  pose_graph::EdgeIdSet outgoing_edge_ids;
  map_.getVertex(vertex_id).getOutgoingEdges(&outgoing_edge_ids);

  CHECK(!outgoing_edge_ids.empty());

  pose_graph::EdgeId outgoing_viwls_edge_id;
  outgoing_viwls_edge_id.setInvalid();
  for (const pose_graph::EdgeId& edge_id : outgoing_edge_ids) {
    CHECK(edge_id.isValid());
    if (map_.getEdgeType(edge_id) == Edge::EdgeType::kViwls) {
      outgoing_viwls_edge_id = edge_id;
    }
  }
  CHECK(outgoing_viwls_edge_id.isValid());

  return map_.getEdgeAs<ViwlsEdge>(outgoing_viwls_edge_id);
}

void VIMapEdgeMergingTest::mergeRandomVertex(
    const pose_graph::VertexId& vertex_id_from,
    const bool has_wheel_odometry_edge) {
  const pose_graph::Edge::EdgeType traversal_edge_type =
      map_.getGraphTraversalEdgeType(mission_id_);
  pose_graph::VertexId next_vertex_id;
  const bool from_has_next_vertex =
      map_.getNextVertex(vertex_id_from, traversal_edge_type, &next_vertex_id);
  if (!from_has_next_vertex) {
    return;
  }

  const ViwlsEdge viwls_edge_1 = getOutgoingViwlsEdge(vertex_id_from);
  const pose_graph::VertexId& in_between_vertex_id = viwls_edge_1.to();
  ASSERT_TRUE(in_between_vertex_id.isValid());

  pose_graph::EdgeId wheel_odometry_edge_1_id;
  aslam::Transformation wheel_odometry_edge_1_T_A_B;
  if (has_wheel_odometry_edge) {
    const TransformationEdge wheel_odometry_edge_1 =
        getOutgoingTransformationEdge(vertex_id_from);
    wheel_odometry_edge_1_id = wheel_odometry_edge_1.id();
    wheel_odometry_edge_1_T_A_B = wheel_odometry_edge_1.getT_A_B();
    ASSERT_EQ(wheel_odometry_edge_1.to(), in_between_vertex_id);
  }

  const bool in_between_has_next_vertex = map_.getNextVertex(
      in_between_vertex_id, traversal_edge_type, &next_vertex_id);

  if (in_between_has_next_vertex) {
    const ViwlsEdge viwls_edge_2 = getOutgoingViwlsEdge(in_between_vertex_id);
    ASSERT_TRUE(viwls_edge_2.to().isValid());

    pose_graph::EdgeId wheel_odometry_edge_2_id;
    aslam::Transformation wheel_odometry_edge_2_T_A_B;
    if (has_wheel_odometry_edge) {
      const TransformationEdge wheel_odometry_edge_2 =
          getOutgoingTransformationEdge(in_between_vertex_id);
      wheel_odometry_edge_2_id = wheel_odometry_edge_2.id();
      wheel_odometry_edge_2_T_A_B = wheel_odometry_edge_2.getT_A_B();
      ASSERT_EQ(wheel_odometry_edge_2.to(), viwls_edge_2.to());
    }

    map_.mergeNeighboringVertices(vertex_id_from, in_between_vertex_id);
    ASSERT_TRUE(map_.hasVertex(vertex_id_from));
    ASSERT_FALSE(map_.hasVertex(in_between_vertex_id));
    ASSERT_FALSE(map_.hasEdge(viwls_edge_1.id()));
    ASSERT_FALSE(map_.hasEdge(viwls_edge_2.id()));

    const ViwlsEdge merged_vilws_edge = getOutgoingViwlsEdge(vertex_id_from);
    EXPECT_EQ(merged_vilws_edge.to(), viwls_edge_2.to());

    if (has_wheel_odometry_edge) {
      const aslam::Transformation T_A_B_merged =
          wheel_odometry_edge_1_T_A_B * wheel_odometry_edge_2_T_A_B;

      const TransformationEdge merged_wheel_odometry_edge =
          getOutgoingTransformationEdge(vertex_id_from);
      EXPECT_EQ(merged_wheel_odometry_edge.getT_A_B(), T_A_B_merged);
      EXPECT_EQ(merged_wheel_odometry_edge.from(), vertex_id_from);
      EXPECT_EQ(merged_wheel_odometry_edge.to(), viwls_edge_2.to());

      ASSERT_FALSE(map_.hasEdge(wheel_odometry_edge_1_id));
      ASSERT_FALSE(map_.hasEdge(wheel_odometry_edge_2_id));
    }
  } else {
    map_.mergeNeighboringVertices(vertex_id_from, in_between_vertex_id);
    ASSERT_TRUE(map_.hasVertex(vertex_id_from));
    ASSERT_FALSE(map_.hasVertex(in_between_vertex_id));
    ASSERT_FALSE(map_.hasEdge(viwls_edge_1.id()));
    if (has_wheel_odometry_edge) {
      ASSERT_FALSE(map_.hasEdge(wheel_odometry_edge_1_id));
    }
  }
}

TEST_F(VIMapEdgeMergingTest, EdgeMergingTestViwls) {
  CHECK_GT(map_.numVertices(), 0u);

  constexpr size_t kSeed = 42u;
  std::srand(kSeed);
  std::mt19937 integer_generator(kSeed);
  constexpr size_t kLowerBound = 0u;

  constexpr size_t kMaxIterations = 1000u;
  size_t iteration = 0u;
  do {
    pose_graph::VertexIdList vertex_ids;
    map_.getAllVertexIds(&vertex_ids);

    if (vertex_ids.size() < 2u) {
      break;
    }
    const size_t upper_bound = vertex_ids.size() - 1u;
    std::uniform_int_distribution<> vertex_index_uniform_distribution(
        kLowerBound, upper_bound);

    const size_t vertex_idx_from =
        vertex_index_uniform_distribution(integer_generator);

    const pose_graph::VertexId& vertex_id_from = vertex_ids[vertex_idx_from];
    constexpr bool kHasWheelOdometryEdges = false;
    mergeRandomVertex(vertex_id_from, kHasWheelOdometryEdges);

    ++iteration;
  } while (iteration < kMaxIterations);
}

TEST_F(VIMapEdgeMergingTest, EdgeMergingTestViwlsOdometry) {
  addWheelOdometryEdges();

  CHECK_GT(map_.numVertices(), 0u);

  constexpr size_t kSeed = 42u;
  std::srand(kSeed);
  std::mt19937 integer_generator(kSeed);
  constexpr size_t kLowerBound = 0u;

  constexpr size_t kMaxIterations = 1000u;
  size_t iteration = 0u;
  do {
    pose_graph::VertexIdList vertex_ids;
    map_.getAllVertexIds(&vertex_ids);

    if (vertex_ids.size() < 2u) {
      break;
    }
    const size_t upper_bound = vertex_ids.size() - 1u;
    std::uniform_int_distribution<> vertex_index_uniform_distribution(
        kLowerBound, upper_bound);

    const size_t vertex_idx_from =
        vertex_index_uniform_distribution(integer_generator);

    const pose_graph::VertexId& vertex_id_from = vertex_ids[vertex_idx_from];
    constexpr bool kHasWheelOdometryEdges = true;
    mergeRandomVertex(vertex_id_from, kHasWheelOdometryEdges);

    ++iteration;
  } while (iteration < kMaxIterations);
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
