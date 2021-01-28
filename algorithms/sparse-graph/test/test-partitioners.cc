#include "sparse-graph/partitioners/avg-partitioner.h"
#include "sparse-graph/partitioners/base-partitioner.h"
#include "sparse-graph/sparse-graph.h"

#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <vi-map/test/vi-map-generator.h>

#include <utility>

namespace spg {

class PartitionerTest : public ::testing::Test {
 protected:
  PartitionerTest() : ::testing::Test() {}

  virtual void SetUp() {}

  pose_graph::VertexIdList createRandomIds(const std::size_t n_ids) {
    pose_graph::VertexIdList all_vertices;

    for (std::size_t i = 0; i < n_ids; ++i) {
      pose_graph::VertexId id = aslam::createRandomId<pose_graph::VertexId>();
      all_vertices.emplace_back(std::move(id));
    }

    return all_vertices;
  }

  RepresentativeNode createEmptyTest(
      vi_map::VIMap* map, BasePartitioner* partitioner) {
    CHECK_NOTNULL(partitioner);
    CHECK_NOTNULL(map);

    vi_map::VIMapGenerator generator(*map, 42u);
    const vi_map::MissionId mission_id = generator.createMission();
    CHECK(mission_id.isValid());
    generator.generateMap();

    return partitioner->getRepresentativesForSubmap({});
  }

  std::pair<RepresentativeNode, aslam::Transformation> createSingleTest(
      vi_map::VIMap* map, BasePartitioner* partitioner) {
    CHECK_NOTNULL(partitioner);
    CHECK_NOTNULL(map);
    vi_map::VIMapGenerator generator(*map, 42u);
    const vi_map::MissionId mission_id = generator.createMission();
    CHECK(mission_id.isValid());
    aslam::Transformation T_M_I0;
    T_M_I0.getPosition() << 1, 2, 3;
    T_M_I0.getRotation() =
        kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0));
    const pose_graph::VertexId vertex_id_0 =
        generator.createVertex(mission_id, T_M_I0);
    CHECK(vertex_id_0.isValid());
    generator.generateMap();
    return std::make_pair(
        partitioner->getRepresentativesForSubmap({vertex_id_0}),
        std::move(T_M_I0));
  }
};

TEST_F(PartitionerTest, TestEmptySetAveragePartitioner) {
  vi_map::VIMap map;
  AvgPartitioner avg_partitioner(map);
  RepresentativeNode node = createEmptyTest(&map, &avg_partitioner);
  aslam::Transformation T_expected;
  T_expected.setIdentity();

  aslam::Transformation T_node = node.getPose();
  EXPECT_NEAR_EIGEN(T_node.getPosition(), T_expected.getPosition(), 1e-5);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_node.getRotation(), T_expected.getRotation(), 1e-5);
}

TEST_F(PartitionerTest, TestSingleSetAveragePartitioner) {
  vi_map::VIMap map;
  AvgPartitioner avg_partitioner(map);
  auto node_and_vertex = createSingleTest(&map, &avg_partitioner);
  aslam::Transformation T_node = node_and_vertex.first.getPose();
  aslam::Transformation T_expected = node_and_vertex.second;

  EXPECT_NEAR_EIGEN(T_node.getPosition(), T_expected.getPosition(), 1e-5);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_node.getRotation(), T_expected.getRotation(), 1e-5);
}

}  // namespace spg

MAPLAB_UNITTEST_ENTRYPOINT
