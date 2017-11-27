#include <Eigen/Core>

#include <map-manager/map-manager.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/vi-map.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "landmark-triangulation/landmark-triangulation.h"

DECLARE_double(vi_map_landmark_quality_min_observation_angle_deg);
DECLARE_uint64(vi_map_landmark_quality_min_observers);
DECLARE_double(vi_map_landmark_quality_max_distance_from_closest_observer);
DECLARE_double(vi_map_landmark_quality_min_distance_from_closest_observer);

namespace landmark_triangulation {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/vi_app_test");
    CHECK_NOTNULL(test_app_.getMapMutable());
  }

  virtual void corruptLandmarks();
  visual_inertial_mapping::VIMappingTestApp test_app_;
};

void ViMappingTest::corruptLandmarks() {
  constexpr double kLandmarkPositionStdDev = 5.0;
  constexpr int kEveryNthToCorrupt = 1;
  test_app_.corruptLandmarkPositions(
      kLandmarkPositionStdDev, kEveryNthToCorrupt);
}

TEST_F(ViMappingTest, TestLandmarkTriangulation) {
  corruptLandmarks();

  EXPECT_TRUE(retriangulateLandmarks(test_app_.getMapMutable()));
  constexpr double kPrecision = 0.1;
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecision, kMinPassingLandmarkFraction);
}

void checkLandmarkQualityInView(
    const vi_map::VIMap& map, int expected_num_unknown_quality,
    int expected_num_good_quality, int expected_num_bad_quality) {
  pose_graph::VertexIdList all_vertices;
  map.getAllVertexIds(&all_vertices);

  int num_unknown_quality = 0;
  int num_good_quality = 0;
  int num_bad_quality = 0;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    for (const vi_map::Landmark& landmark : vertex.getLandmarks()) {
      switch (landmark.getQuality()) {
        case vi_map::Landmark::Quality::kUnknown: {
          ++num_unknown_quality;
          break;
        }
        case vi_map::Landmark::Quality::kBad: {
          ++num_bad_quality;
          break;
        }
        case vi_map::Landmark::Quality::kGood: {
          ++num_good_quality;
          break;
        }
        default: {
          // Fall through intended.
        }
      }
    }
  }

  EXPECT_EQ(num_unknown_quality, expected_num_unknown_quality);
  EXPECT_EQ(num_good_quality, expected_num_good_quality);
  EXPECT_EQ(num_bad_quality, expected_num_bad_quality);
}

TEST_F(ViMappingTest, TestLandmarkQualityMetrics) {
  corruptLandmarks();

  vi_map::VIMap* map = test_app_.getMapMutable();
  checkLandmarkQualityInView(*map, 8359, 0, 0);

  FLAGS_vi_map_landmark_quality_min_observation_angle_deg = 5;
  FLAGS_vi_map_landmark_quality_min_observers = 4;
  FLAGS_vi_map_landmark_quality_max_distance_from_closest_observer = 40;
  FLAGS_vi_map_landmark_quality_min_distance_from_closest_observer = 0.05;

  EXPECT_TRUE(retriangulateLandmarks(map));
  checkLandmarkQualityInView(*map, 0, 6138, 2221);
}

}  // namespace landmark_triangulation

MAPLAB_UNITTEST_ENTRYPOINT
