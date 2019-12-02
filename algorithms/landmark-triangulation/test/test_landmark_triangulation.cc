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

  vi_map::MissionIdList mission_ids;
  test_app_.getMapMutable()->getAllMissionIds(&mission_ids);
  retriangulateLandmarks(mission_ids, test_app_.getMapMutable());
  constexpr double kPrecision = 0.1;
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecision, kMinPassingLandmarkFraction);
}

TEST_F(ViMappingTest, TestLandmarkTriangulationEntireMap) {
  corruptLandmarks();

  retriangulateLandmarks(test_app_.getMapMutable());
  constexpr double kPrecision = 0.1;
  constexpr double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecision, kMinPassingLandmarkFraction);
}

}  // namespace landmark_triangulation

MAPLAB_UNITTEST_ENTRYPOINT
