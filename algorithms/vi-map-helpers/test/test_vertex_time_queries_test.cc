#include <aslam/common/pose-types.h>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-generator.h>

#include "vi-map-helpers/vi-map-vertex-time-queries.h"

namespace vi_map_helpers {

pose_graph::VertexId getGroundTruthVertexIdClosestInTime(
    const int64_t query_timestamp_nanoseconds,
    const pose_graph::VertexIdList& vertex_ids, const vi_map::VIMap::Ptr& map) {
  CHECK(map);
  pose_graph::VertexId closest_vertex_id;
  closest_vertex_id.setInvalid();
  CHECK(!vertex_ids.empty());
  int64_t closest_timestamp_difference = std::numeric_limits<int64_t>::max();
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    CHECK(vertex_id.isValid());
    const int64_t timestamp_nanoseconds =
        map->getVertex(vertex_id).getMinTimestampNanoseconds();

    const int64_t timestamp_difference_nanoseconds =
        std::abs(timestamp_nanoseconds - query_timestamp_nanoseconds);
    if (timestamp_difference_nanoseconds < closest_timestamp_difference) {
      closest_timestamp_difference = timestamp_difference_nanoseconds;
      closest_vertex_id = vertex_id;
    }
  }
  CHECK(closest_vertex_id.isValid());
  return closest_vertex_id;
}

pose_graph::VertexId getGroundTruthVertexIdClosestInTime(
    const int64_t query_timestamp_nanoseconds, const vi_map::VIMap::Ptr& map) {
  CHECK(map);
  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIds(&vertex_ids);
  return getGroundTruthVertexIdClosestInTime(query_timestamp_nanoseconds,
                                             vertex_ids, map);
}

pose_graph::VertexId getGroundTruthVertexIdClosestInTime(
    const int64_t query_timestamp_nanoseconds,
    const vi_map::MissionId& mission_id, const vi_map::VIMap::Ptr& map) {
  CHECK(map);
  CHECK(mission_id.isValid());
  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIdsInMission(mission_id, &vertex_ids);
  return getGroundTruthVertexIdClosestInTime(query_timestamp_nanoseconds,
                                             vertex_ids, map);
}

TEST(VertexTimeQueriesTest, TestTimeQueriesSingleAndMultiMission) {
  vi_map::VIMap::Ptr map(new vi_map::VIMap);
  const int kSeed = 42;
  vi_map::VIMapGenerator generator(*map, kSeed);

  std::unordered_map<pose_graph::VertexId, int64_t>
      vertex_id_to_timestamp_nanoseconds;

  std::default_random_engine random_number_engine(kSeed);
  std::uniform_int_distribution<int64_t> timestamp_distribution(0, 1e12);

  const vi_map::MissionId mission_id_1 = generator.createMission();
  CHECK(mission_id_1.isValid());

  const vi_map::MissionId mission_id_2 = generator.createMission();
  CHECK(mission_id_2.isValid());

  constexpr size_t kNumVerticesPerMission = 200u;

  for (size_t vertex_idx = 0u; vertex_idx < kNumVerticesPerMission;
       ++vertex_idx) {
    const aslam::Transformation T_M_I(aslam::Quaternion(),
                                      aslam::Position3D::Random());
    generator.createVertex(mission_id_1, T_M_I,
                           timestamp_distribution(random_number_engine));
  }

  for (size_t vertex_idx = 0u; vertex_idx < kNumVerticesPerMission;
       ++vertex_idx) {
    const aslam::Transformation T_M_I(aslam::Quaternion(),
                                      aslam::Position3D::Random());
    generator.createVertex(mission_id_2, T_M_I,
                           timestamp_distribution(random_number_engine));
  }

  generator.generateMap();

  VIMapVertexTimeQueries vertex_time_index(map);

  const size_t kNumQueries = 50u;

  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const int64_t query_timestamp_nanoseconds =
        timestamp_distribution(random_number_engine);

    pose_graph::VertexId nn_vertex_id;
    vertex_time_index.getClosestVertexInTime(query_timestamp_nanoseconds,
                                             &nn_vertex_id);
    ASSERT_TRUE(nn_vertex_id.isValid());
    EXPECT_EQ(nn_vertex_id, getGroundTruthVertexIdClosestInTime(
                                query_timestamp_nanoseconds, map));
  }

  VIMapMissionsVertexTimeQueries mission_vertex_time_index(map);

  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const int64_t query_timestamp_nanoseconds =
        timestamp_distribution(random_number_engine);

    pose_graph::VertexId nn_vertex_id;
    mission_vertex_time_index.getClosestVertexInTimeForMission(
        query_timestamp_nanoseconds, mission_id_1, &nn_vertex_id);
    ASSERT_TRUE(nn_vertex_id.isValid());
    EXPECT_EQ(nn_vertex_id,
              getGroundTruthVertexIdClosestInTime(query_timestamp_nanoseconds,
                                                  mission_id_1, map));
  }

  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const int64_t query_timestamp_nanoseconds =
        timestamp_distribution(random_number_engine);

    pose_graph::VertexId nn_vertex_id;
    mission_vertex_time_index.getClosestVertexInTimeForMission(
        query_timestamp_nanoseconds, mission_id_2, &nn_vertex_id);
    ASSERT_TRUE(nn_vertex_id.isValid());
    EXPECT_EQ(nn_vertex_id,
              getGroundTruthVertexIdClosestInTime(query_timestamp_nanoseconds,
                                                  mission_id_2, map));
  }
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
