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

  RepresentativeNodeVector createEmptyTest(
      vi_map::VIMap* map, BasePartitioner* partitioner) {
    CHECK_NOTNULL(partitioner);
    CHECK_NOTNULL(map);

    vi_map::VIMapGenerator generator(*map, 42u);
    const vi_map::MissionId mission_id = generator.createMission();
    CHECK(mission_id.isValid());
    generator.generateMap();

    const uint64_t submap_id = 0u;
    return partitioner->getRepresentativesForSubmap({}, submap_id);
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

    const uint64_t submap_id = 0u;
    RepresentativeNodeVector nodes =
        partitioner->getRepresentativesForSubmap({vertex_id_0}, submap_id);
    CHECK_EQ(nodes.size(), 1);

    return std::make_pair(std::move(nodes[0]), std::move(T_M_I0));
  }

  std::pair<RepresentativeNodeVector, std::vector<aslam::Transformation>>
  createMultipleTest(vi_map::VIMap* map, BasePartitioner* partitioner) {
    CHECK_NOTNULL(partitioner);
    CHECK_NOTNULL(map);
    vi_map::VIMapGenerator generator(*map, 42u);
    const vi_map::MissionId mission_id = generator.createMission();
    CHECK(mission_id.isValid());
    aslam::Transformation T_M_I0;
    aslam::Transformation T_M_I1;
    aslam::Transformation T_M_I2;
    T_M_I0.getPosition() << 1, 2, 3;
    T_M_I1.getPosition() << 4, 5, 6;
    T_M_I2.getPosition() << 7, 8, 9;
    T_M_I0.getRotation() =
        kindr::minimal::RotationQuaternion(Eigen::Quaterniond(1, 0, 0, 0));
    T_M_I1.getRotation() = kindr::minimal::RotationQuaternion(
        Eigen::Quaterniond(0.9970644, 0.0454372, 0.0416356, 0.0454372));
    T_M_I2.getRotation() = kindr::minimal::RotationQuaternion(
        Eigen::Quaterniond(0.9879654, 0.0940609, 0.0789265, 0.0940609));
    const pose_graph::VertexId vertex_id_0 =
        generator.createVertex(mission_id, T_M_I0);
    const pose_graph::VertexId vertex_id_1 =
        generator.createVertex(mission_id, T_M_I1);
    const pose_graph::VertexId vertex_id_2 =
        generator.createVertex(mission_id, T_M_I2);
    CHECK(vertex_id_0.isValid());
    CHECK(vertex_id_1.isValid());
    CHECK(vertex_id_2.isValid());

    generator.generateMap();
    const uint64_t submap_id = 0u;
    return std::make_pair(
        partitioner->getRepresentativesForSubmap(
            {vertex_id_0, vertex_id_1, vertex_id_2}, submap_id),
        std::vector<aslam::Transformation>({T_M_I0, T_M_I1, T_M_I2}));
  }
};

TEST_F(PartitionerTest, TestEmptySetAveragePartitioner) {
  vi_map::VIMap map;
  AvgPartitioner avg_partitioner(map);
  RepresentativeNodeVector node = createEmptyTest(&map, &avg_partitioner);
  aslam::Transformation T_expected;
  EXPECT_TRUE(node.empty());
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

TEST_F(PartitionerTest, TestMultipleSetAveragePartitioner) {
  vi_map::VIMap map;
  AvgPartitioner avg_partitioner(map);
  auto node_and_vertex = createMultipleTest(&map, &avg_partitioner);
  RepresentativeNodeVector nodes = node_and_vertex.first;
  EXPECT_FALSE(nodes.empty());

  aslam::Transformation T_node = nodes[0].getPose();
  aslam::Transformation T_expected = node_and_vertex.second[1];  // center

  EXPECT_NEAR_EIGEN(T_node.getPosition(), T_expected.getPosition(), 1e-5);
  EXPECT_NEAR_KINDR_QUATERNION(
      T_node.getRotation(), T_expected.getRotation(), 1e-2);
}

}  // namespace spg

MAPLAB_UNITTEST_ENTRYPOINT
