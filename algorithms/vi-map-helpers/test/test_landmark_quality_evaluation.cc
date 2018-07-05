#include <glog/logging.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/vi-map.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "vi-map-helpers/test/vi-map-landmark-quality-check.h"
#include "vi-map-helpers/vi-map-landmark-quality-evaluation.h"

DECLARE_double(vi_map_landmark_quality_min_observation_angle_deg);
DECLARE_uint64(vi_map_landmark_quality_min_observers);
DECLARE_double(vi_map_landmark_quality_max_distance_from_closest_observer);
DECLARE_double(vi_map_landmark_quality_min_distance_from_closest_observer);

namespace vi_map_helpers {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/vi_app_test");
    CHECK_NOTNULL(test_app_.getMapMutable());
  }

  visual_inertial_mapping::VIMappingTestApp test_app_;
};

TEST_F(ViMappingTest, TestLandmarkQualityEvaluation) {
  vi_map::VIMap* map = CHECK_NOTNULL(test_app_.getMapMutable());

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  resetLandmarkQualityToUnknown(mission_ids, map);

  checkLandmarkQualityInView(*map, 8359, 0, 0);

  FLAGS_vi_map_landmark_quality_min_observation_angle_deg = 5;
  FLAGS_vi_map_landmark_quality_min_observers = 4;
  FLAGS_vi_map_landmark_quality_max_distance_from_closest_observer = 40;
  FLAGS_vi_map_landmark_quality_min_distance_from_closest_observer = 0.05;

  evaluateLandmarkQuality(map);
  checkLandmarkQualityInView(*map, 0, 6107, 2252);
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
