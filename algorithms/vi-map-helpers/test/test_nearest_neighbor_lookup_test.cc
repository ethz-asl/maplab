#include <random>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-generator.h>

#include "vi-map-helpers/gps-vi-map-nearest-neighbor-lookup.h"
#include "vi-map-helpers/vi-map-nearest-neighbor-lookup.h"

namespace vi_map_helpers {

constexpr size_t kSeed = 42u;

typedef Aligned<std::vector, Eigen::VectorXd> MeasurementsList;
typedef AlignedUnorderedSet<Eigen::VectorXd> MeasurementsSet;

Eigen::VectorXd getGroundTruthClosestDataItem(
    const Aligned<std::vector, Eigen::VectorXd>& measurements,
    const Eigen::VectorXd& query) {
  Eigen::VectorXd closest_measurement;

  double min_distance = std::numeric_limits<double>::max();
  for (size_t idx = 0u; idx < measurements.size(); ++idx) {
    const double distance = (query - measurements[idx]).squaredNorm();
    if (distance < min_distance) {
      closest_measurement = measurements[idx];
      min_distance = distance;
    }
  }
  return closest_measurement;
}

void getGroundTruthAllDataItemsWithinRadius(
    const MeasurementsList& measurements, const Eigen::VectorXd& query,
    const double search_radius,
    AlignedUnorderedSet<Eigen::VectorXd>* measurements_within_radius) {
  CHECK_NOTNULL(measurements_within_radius)->clear();

  for (size_t idx = 0u; idx < measurements.size(); ++idx) {
    const Eigen::VectorXd measurement = measurements[idx];
    const double distance = (query - measurement).norm();
    if (distance <= search_radius) {
      measurements_within_radius->emplace(measurement);
    }
  }
}

TEST(VIMapNearestNeighborLookupTest, EmptyMap) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  generator.generateMap();

  VIMapNearestNeighborLookupVertexId nn_query_database(map);

  const Eigen::Vector3d p_query = Eigen::Vector3d::Zero();

  pose_graph::VertexId nn_vertex_id;
  nn_query_database.getClosestDataItem(p_query, &nn_vertex_id);
  EXPECT_FALSE(nn_vertex_id.isValid());
}

TEST(VIMapNearestNeighborLookupTest, SingleVertexMap) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  const vi_map::MissionId mission_id = generator.createMission();
  CHECK(mission_id.isValid());
  aslam::Transformation T_M_I0;
  T_M_I0.getPosition() << 1, 2, 3;
  const pose_graph::VertexId vertex_id_0 =
      generator.createVertex(mission_id, T_M_I0);
  CHECK(vertex_id_0.isValid());

  generator.generateMap();

  VIMapNearestNeighborLookupVertexId nn_query_database(map);

  srand(kSeed);

  const size_t kNumQueries = 10u;
  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    Eigen::Vector3d p_query = 1e3 * Eigen::Vector3d::Random();
    pose_graph::VertexId nn_vertex_id;
    nn_query_database.getClosestDataItem(p_query, &nn_vertex_id);
    EXPECT_TRUE(nn_vertex_id.isValid());
    EXPECT_EQ(nn_vertex_id, vertex_id_0);
  }
}

typedef AlignedUnorderedMap<pose_graph::VertexId, Eigen::VectorXd>
    VertexIdToMeasurementMap;

TEST(VIMapNearestNeighborLookupTest, MultipleVertexMap) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  const vi_map::MissionId mission_id = generator.createMission();
  CHECK(mission_id.isValid());

  srand(kSeed);

  const size_t kNumVertices = 10u;

  MeasurementsList p_G_Is;
  aslam::TransformationVector T_G_Is(kNumVertices);
  VertexIdToMeasurementMap vertex_id_to_data_item_map;

  for (size_t idx = 0u; idx < kNumVertices; ++idx) {
    T_G_Is[idx].getPosition() = 1e3 * Eigen::Vector3d::Random();

    const pose_graph::VertexId vertex_id =
        generator.createVertex(mission_id, T_G_Is[idx]);
    CHECK(vertex_id.isValid());

    const Eigen::VectorXd measurement =
        queryTypeToVector(T_G_Is[idx].getPosition());
    p_G_Is.emplace_back(measurement);
    vertex_id_to_data_item_map.emplace(vertex_id, measurement);
  }

  generator.generateMap();

  VIMapNearestNeighborLookupVertexId nn_query_database(map);

  const size_t kNumQueries = 10u;
  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    Eigen::Vector3d p_G_I_query = 1e3 * Eigen::Vector3d::Random();
    pose_graph::VertexId closest_vertex_id;
    nn_query_database.getClosestDataItem(p_G_I_query, &closest_vertex_id);
    EXPECT_TRUE(closest_vertex_id.isValid());

    const Eigen::VectorXd ground_truth_closest_p_G_I =
        getGroundTruthClosestDataItem(p_G_Is, queryTypeToVector(p_G_I_query));

    VertexIdToMeasurementMap::const_iterator p_G_I_iterator =
        vertex_id_to_data_item_map.find(closest_vertex_id);
    CHECK(p_G_I_iterator != vertex_id_to_data_item_map.end());

    EXPECT_EQ(p_G_I_iterator->second, ground_truth_closest_p_G_I);
  }
}

TEST(VIMapNearestNeighborLookupTest, MultipleVertexRadiusLookup) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  const vi_map::MissionId mission_id = generator.createMission();
  CHECK(mission_id.isValid());

  srand(kSeed);

  const size_t kNumVertices = 7u;

  MeasurementsList p_G_Is;
  aslam::TransformationVector T_G_Is(kNumVertices);
  pose_graph::VertexIdList T_G_I_vertex_ids(kNumVertices);

  VertexIdToMeasurementMap vertex_id_to_data_item_map;

  for (size_t idx = 0u; idx < kNumVertices; ++idx) {
    T_G_Is[idx].getPosition() = 1e3 * Eigen::Vector3d::Random();

    const pose_graph::VertexId vertex_id =
        generator.createVertex(mission_id, T_G_Is[idx]);
    CHECK(vertex_id.isValid());

    T_G_I_vertex_ids[idx] = vertex_id;

    const Eigen::VectorXd p_G_I = queryTypeToVector(T_G_Is[idx].getPosition());
    p_G_Is.emplace_back(p_G_I);
    vertex_id_to_data_item_map.emplace(vertex_id, p_G_I);
  }

  generator.generateMap();

  VIMapNearestNeighborLookupVertexId nn_query_database(map);

  std::mt19937 random_number_generator(kSeed);
  std::uniform_real_distribution<> uniform_distribution(0.0, 1e3);

  const size_t kNumQueries = 10u;
  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const Eigen::Vector3d p_G_I_query = 1e3 * Eigen::Vector3d::Random();

    const double search_radius = uniform_distribution(random_number_generator);

    pose_graph::VertexIdSet vertex_ids_within_search_radius;
    nn_query_database.getAllDataItemsWithinRadius(
        p_G_I_query, search_radius, &vertex_ids_within_search_radius);

    MeasurementsSet p_G_Is_within_search_radius;
    for (const pose_graph::VertexId& vertex_id :
         vertex_ids_within_search_radius) {
      CHECK(vertex_id.isValid());
      VertexIdToMeasurementMap::const_iterator p_G_I_iterator =
          vertex_id_to_data_item_map.find(vertex_id);
      CHECK(p_G_I_iterator != vertex_id_to_data_item_map.end());
      p_G_Is_within_search_radius.emplace(p_G_I_iterator->second);
    }

    MeasurementsSet ground_truth_p_G_Is_within_search_radius;
    getGroundTruthAllDataItemsWithinRadius(
        p_G_Is, queryTypeToVector(p_G_I_query), search_radius,
        &ground_truth_p_G_Is_within_search_radius);

    EXPECT_EQ(
        p_G_Is_within_search_radius, ground_truth_p_G_Is_within_search_radius);
  }
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
