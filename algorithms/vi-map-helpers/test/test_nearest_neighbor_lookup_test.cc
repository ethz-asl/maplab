#include <random>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-generator.h>

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

    EXPECT_EQ(p_G_Is_within_search_radius,
              ground_truth_p_G_Is_within_search_radius);
  }
}

TEST(VIMapNearestNeighborLookupTest, MultipleGPSWGSMeasurementsNNLookup) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  const vi_map::MissionId mission_id = generator.createMission();
  CHECK(mission_id.isValid());

  generator.generateMap();

  srand(kSeed);

  const size_t kNumMeasurements = 10u;

  MeasurementsList wgs_measurements_lat_lon;

  std::mt19937 random_number_generator(kSeed);
  std::uniform_real_distribution<> uniform_distribution_latitude(
      vi_map::GpsWgsMeasurement::kMinLatitudeDeg,
      vi_map::GpsWgsMeasurement::kMaxLatitudeDeg);
  std::uniform_real_distribution<> uniform_distribution_longitude(
      vi_map::GpsWgsMeasurement::kMinLongitudeDeg,
      vi_map::GpsWgsMeasurement::kMaxLongitudeDeg);
  std::uniform_real_distribution<> uniform_distribution_altitude(
      vi_map::GpsWgsMeasurement::kMinAltitudeMeters,
      vi_map::GpsWgsMeasurement::kMaxAltitudeMeters);

  vi_map::GpsWgs::UniquePtr gps_wgs_sensor =
      vi_map::createTestSensor<vi_map::GpsWgs>();
  CHECK(gps_wgs_sensor);
  const vi_map::SensorId sensor_id = gps_wgs_sensor->getId();
  CHECK(sensor_id.isValid());
  map.getSensorManager().addSensor(std::move(gps_wgs_sensor), mission_id);

  for (size_t idx = 0u; idx < kNumMeasurements; ++idx) {
    const vi_map::GpsWgsMeasurement wgs_measurement(
        sensor_id, static_cast<int64_t>(idx) + 1,
        uniform_distribution_latitude(random_number_generator),
        uniform_distribution_longitude(random_number_generator),
        uniform_distribution_altitude(random_number_generator));

    map.addOptionalSensorMeasurement(wgs_measurement, mission_id);
    wgs_measurements_lat_lon.emplace_back(queryTypeToVector(wgs_measurement));
  }

  VIMapNearestNeighborLookupGpsWgs nn_query_database(map);

  const size_t kNumQueries = 10u;
  const int64_t kInvalidTimestampNanoseconds = 0;
  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const vi_map::GpsWgsMeasurement query_wgs_measurement(
        sensor_id, kInvalidTimestampNanoseconds,
        uniform_distribution_latitude(random_number_generator),
        uniform_distribution_longitude(random_number_generator),
        uniform_distribution_altitude(random_number_generator));

    vi_map::GpsWgsMeasurement closest_wgs_measurement;
    nn_query_database.getClosestDataItem(query_wgs_measurement,
                                         &closest_wgs_measurement);

    const Eigen::VectorXd ground_truth_closest_wgs_measurement =
        getGroundTruthClosestDataItem(
            wgs_measurements_lat_lon, queryTypeToVector(query_wgs_measurement));
    EXPECT_EQ(
        queryTypeToVector(closest_wgs_measurement),
        ground_truth_closest_wgs_measurement);
  }
}

TEST(VIMapNearestNeighborLookupTest, MultipleWGSMeasurementRadiusLookup) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  const vi_map::MissionId mission_id = generator.createMission();
  CHECK(mission_id.isValid());

  generator.generateMap();

  srand(kSeed);

  const size_t kNumMeasurements = 10u;

  MeasurementsList wgs_measurements_lat_lon;

  std::mt19937 random_number_generator(kSeed);
  std::uniform_real_distribution<> uniform_distribution_latitude(
      vi_map::GpsWgsMeasurement::kMinLatitudeDeg,
      vi_map::GpsWgsMeasurement::kMaxLatitudeDeg);
  std::uniform_real_distribution<> uniform_distribution_longitude(
      vi_map::GpsWgsMeasurement::kMinLongitudeDeg,
      vi_map::GpsWgsMeasurement::kMaxLongitudeDeg);
  std::uniform_real_distribution<> uniform_distribution_altitude(
      vi_map::GpsWgsMeasurement::kMinAltitudeMeters,
      vi_map::GpsWgsMeasurement::kMaxAltitudeMeters);

  vi_map::GpsWgs::UniquePtr gps_wgs_sensor =
      vi_map::createTestSensor<vi_map::GpsWgs>();
  CHECK(gps_wgs_sensor);
  const vi_map::SensorId sensor_id = gps_wgs_sensor->getId();
  CHECK(sensor_id.isValid());
  map.getSensorManager().addSensor(std::move(gps_wgs_sensor), mission_id);

  for (size_t idx = 0u; idx < kNumMeasurements; ++idx) {
    const vi_map::GpsWgsMeasurement wgs_measurement(
        sensor_id, static_cast<int64_t>(idx) + 1,
        uniform_distribution_latitude(random_number_generator),
        uniform_distribution_longitude(random_number_generator),
        uniform_distribution_altitude(random_number_generator));

    map.addOptionalSensorMeasurement(wgs_measurement, mission_id);
    wgs_measurements_lat_lon.emplace_back(queryTypeToVector(wgs_measurement));
  }

  VIMapNearestNeighborLookupGpsWgs nn_query_database(map);

  std::uniform_real_distribution<> uniform_distribution_search_radius(
      0.0, 180.0);

  const size_t kNumQueries = 10u;
  const int64_t kInvalidTimestampNanoseconds = 0;
  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const vi_map::GpsWgsMeasurement query_wgs_measurement(
        sensor_id, kInvalidTimestampNanoseconds,
        uniform_distribution_latitude(random_number_generator),
        uniform_distribution_longitude(random_number_generator),
        uniform_distribution_altitude(random_number_generator));

    const double search_radius =
        uniform_distribution_search_radius(random_number_generator);

    vi_map::GpsWgsMeasurementSet wgs_measurments_lat_lon_within_search_radius;
    nn_query_database.getAllDataItemsWithinRadius(
        query_wgs_measurement, search_radius,
        &wgs_measurments_lat_lon_within_search_radius);

    MeasurementsSet wgs_measurements_lat_lon_within_search_radius_as_vectors;
    for (const vi_map::GpsWgsMeasurement& wgs_measurement_within_search_radius :
         wgs_measurments_lat_lon_within_search_radius) {
      wgs_measurements_lat_lon_within_search_radius_as_vectors.emplace(
          queryTypeToVector(wgs_measurement_within_search_radius));
    }

    MeasurementsSet
        ground_truth_wgs_measurments_lat_lon_within_search_radius_as_vectors;
    getGroundTruthAllDataItemsWithinRadius(
        wgs_measurements_lat_lon, queryTypeToVector(query_wgs_measurement),
        search_radius,
        &ground_truth_wgs_measurments_lat_lon_within_search_radius_as_vectors);

    EXPECT_EQ(
        wgs_measurements_lat_lon_within_search_radius_as_vectors,
        ground_truth_wgs_measurments_lat_lon_within_search_radius_as_vectors);
  }
}

TEST(VIMapNearestNeighborLookupTest, MultipleGPSUTMMeasuremnentsNNLookup) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  const vi_map::MissionId mission_id = generator.createMission();
  CHECK(mission_id.isValid());

  generator.generateMap();

  srand(kSeed);

  const size_t kNumMeasurements = 10u;

  std::mt19937 random_number_generator(kSeed);

  MeasurementsList utm_measurments_p_UTM_Bs;

  vi_map::GpsUtm::UniquePtr gps_utm_sensor =
      vi_map::createTestSensor<vi_map::GpsUtm>();
  CHECK(gps_utm_sensor);
  const vi_map::SensorId sensor_id = gps_utm_sensor->getId();
  CHECK(sensor_id.isValid());
  map.getSensorManager().addSensor(std::move(gps_utm_sensor), mission_id);

  for (size_t idx = 0u; idx < kNumMeasurements; ++idx) {
    const Eigen::Vector3d axis_angle_UTM_B = Eigen::Vector3d::Random();
    const aslam::Position3D p_UTM_B = 1e6 * aslam::Position3D::Random();

    const aslam::Transformation T_UTM_B(
        aslam::Quaternion(aslam::AngleAxis(axis_angle_UTM_B)), p_UTM_B);

    const vi_map::GpsUtmMeasurement utm_measurement(
        sensor_id, static_cast<int64_t>(idx) + 1, T_UTM_B,
        vi_map::UtmZone(32u, 'T'));

    map.addOptionalSensorMeasurement(utm_measurement, mission_id);

    utm_measurments_p_UTM_Bs.emplace_back(queryTypeToVector(utm_measurement));
  }

  VIMapNearestNeighborLookupGpsUtm nn_query_database(map);

  const size_t kNumQueries = 10u;
  const int64_t kInvalidTimestampNanoseconds = 0;
  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const Eigen::Vector3d axis_angle_UTM_B = Eigen::Vector3d::Random();
    const aslam::Position3D p_UTM_B = 1e6 * aslam::Position3D::Random();

    const aslam::Transformation T_UTM_B(
        aslam::Quaternion(aslam::AngleAxis(axis_angle_UTM_B)), p_UTM_B);

    const vi_map::GpsUtmMeasurement query_utm_measurement(
        sensor_id, kInvalidTimestampNanoseconds, T_UTM_B,
        vi_map::UtmZone(32u, 'T'));

    vi_map::GpsUtmMeasurement closest_utm_measurement;
    nn_query_database.getClosestDataItem(query_utm_measurement,
                                         &closest_utm_measurement);

    const Eigen::VectorXd ground_truth_closest_utm_measurement_p_UTM_B =
        getGroundTruthClosestDataItem(
            utm_measurments_p_UTM_Bs, queryTypeToVector(query_utm_measurement));
    EXPECT_EQ(
        queryTypeToVector(closest_utm_measurement),
        ground_truth_closest_utm_measurement_p_UTM_B);
  }
}

TEST(VIMapNearestNeighborLookupTest, MultipleUTMMeasurementsRadiusLookup) {
  vi_map::VIMap map;
  vi_map::VIMapGenerator generator(map, kSeed);
  const vi_map::MissionId mission_id = generator.createMission();
  CHECK(mission_id.isValid());

  generator.generateMap();

  srand(kSeed);

  const size_t kNumMeasurements = 10u;

  MeasurementsList utm_measurments_p_UTM_Bs;

  vi_map::GpsUtm::UniquePtr gps_utm_sensor =
      vi_map::createTestSensor<vi_map::GpsUtm>();
  CHECK(gps_utm_sensor);
  const vi_map::SensorId sensor_id = gps_utm_sensor->getId();
  CHECK(sensor_id.isValid());
  map.getSensorManager().addSensor(std::move(gps_utm_sensor), mission_id);
  for (size_t idx = 0u; idx < kNumMeasurements; ++idx) {
    const Eigen::Vector3d axis_angle_UTM_B = Eigen::Vector3d::Random();
    const aslam::Position3D p_UTM_B = 1e6 * aslam::Position3D::Random();

    const aslam::Transformation T_UTM_B(
        aslam::Quaternion(aslam::AngleAxis(axis_angle_UTM_B)), p_UTM_B);

    const vi_map::GpsUtmMeasurement utm_measurement(
        sensor_id, static_cast<int64_t>(idx) + 1, T_UTM_B,
        vi_map::UtmZone(32u, 'T'));

    map.addOptionalSensorMeasurement(utm_measurement, mission_id);

    utm_measurments_p_UTM_Bs.emplace_back(queryTypeToVector(utm_measurement));
  }

  std::mt19937 random_number_generator(kSeed);
  std::uniform_real_distribution<> uniform_distribution(0.0, 1e6);

  VIMapNearestNeighborLookupGpsUtm nn_query_database(map);

  const size_t kNumQueries = 10u;
  const int64_t kInvalidTimestampNanoseconds = 0;
  for (size_t query_idx = 0u; query_idx < kNumQueries; ++query_idx) {
    const double search_radius = uniform_distribution(random_number_generator);

    const Eigen::Vector3d axis_angle_UTM_B = Eigen::Vector3d::Random();
    const aslam::Position3D p_UTM_B = 1e6 * aslam::Position3D::Random();

    const aslam::Transformation T_UTM_B(
        aslam::Quaternion(aslam::AngleAxis(axis_angle_UTM_B)), p_UTM_B);

    const vi_map::GpsUtmMeasurement query_utm_measurement(
        sensor_id, kInvalidTimestampNanoseconds, T_UTM_B,
        vi_map::UtmZone(32u, 'T'));

    vi_map::GpsUtmMeasurementSet utm_measurments_p_UTM_Bs_within_search_radius;
    nn_query_database.getAllDataItemsWithinRadius(
        query_utm_measurement, search_radius,
        &utm_measurments_p_UTM_Bs_within_search_radius);

    MeasurementsSet utm_measurments_p_UTM_Bs_within_search_radius_as_vectors;
    for (const vi_map::GpsUtmMeasurement& utm_measurement_within_search_radius :
         utm_measurments_p_UTM_Bs_within_search_radius) {
      utm_measurments_p_UTM_Bs_within_search_radius_as_vectors.emplace(
          queryTypeToVector(utm_measurement_within_search_radius));
    }

    MeasurementsSet
        ground_truth_utm_measurments_p_UTM_Bs_within_search_radius_as_vectors;
    getGroundTruthAllDataItemsWithinRadius(
        utm_measurments_p_UTM_Bs, queryTypeToVector(query_utm_measurement),
        search_radius,
        &ground_truth_utm_measurments_p_UTM_Bs_within_search_radius_as_vectors);

    EXPECT_EQ(
        utm_measurments_p_UTM_Bs_within_search_radius_as_vectors,
        ground_truth_utm_measurments_p_UTM_Bs_within_search_radius_as_vectors);
  }
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
