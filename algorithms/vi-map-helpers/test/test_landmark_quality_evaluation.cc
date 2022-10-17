#include <glog/logging.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-map/vi-map.h>

#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "vi-map-helpers/vi-map-landmark-quality-evaluation.h"

namespace vi_map_helpers {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/common_test_map");
    CHECK_NOTNULL(test_app_.getMapMutable());
  }

  virtual void corruptLandmarks();
  virtual void addCorruptDuplicateLandmarkObservations();
  virtual void countLandmarkQualityInView(
      const vi_map::VIMap& map, int* num_unknown_quality, int* num_good_quality,
      int* num_bad_quality);
  visual_inertial_mapping::VIMappingTestApp test_app_;
};

void ViMappingTest::corruptLandmarks() {
  constexpr double kLandmarkPositionStdDev = 20.0;
  constexpr int kEveryNthToCorrupt = 5;
  test_app_.corruptLandmarkPositions(
      kLandmarkPositionStdDev, kEveryNthToCorrupt);
}

void ViMappingTest::addCorruptDuplicateLandmarkObservations() {
  constexpr int kEveryNthToCorrupt = 20;
  test_app_.addCorruptDuplicateLandmarkObservations(kEveryNthToCorrupt);
}

void ViMappingTest::countLandmarkQualityInView(
    const vi_map::VIMap& map, int* num_unknown, int* num_good, int* num_bad) {
  CHECK_NOTNULL(num_unknown);
  CHECK_NOTNULL(num_good);
  CHECK_NOTNULL(num_bad);

  pose_graph::VertexIdList all_vertices;
  map.getAllVertexIds(&all_vertices);

  *num_unknown = 0;
  *num_good = 0;
  *num_bad = 0;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    for (const vi_map::Landmark& landmark : vertex.getLandmarks()) {
      switch (landmark.getQuality()) {
        case vi_map::Landmark::Quality::kUnknown:
          ++(*num_unknown);
          break;
        case vi_map::Landmark::Quality::kBad:
          ++(*num_bad);
          break;
        case vi_map::Landmark::Quality::kGood:
          ++(*num_good);
          break;
        default:
          break;
      }
    }
  }
}

TEST_F(ViMappingTest, TestCorruptObservationCleanup) {
  addCorruptDuplicateLandmarkObservations();
  vi_map::VIMap* map = test_app_.getMapMutable();
  EXPECT_FALSE(test_app_.isMapConsistent());
  removeInvalidLandmarkObservations(map);
  evaluateLandmarkQuality(map);
  EXPECT_TRUE(test_app_.isMapConsistent());
}

TEST_F(ViMappingTest, TestLandmarkQualityEvaluation) {
  vi_map::VIMap* map = CHECK_NOTNULL(test_app_.getMapMutable());

  int num_unknown0, num_good0, num_bad0;
  countLandmarkQualityInView(*map, &num_unknown0, &num_good0, &num_bad0);
  // We begin by assuming the map has been processed
  CHECK_EQ(num_unknown0, 0);

  // Reset and re-evaluate landmark quality
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  resetLandmarkQualityToUnknown(mission_ids, map);

  int num_unknown1, num_good1, num_bad1;
  countLandmarkQualityInView(*map, &num_unknown1, &num_good1, &num_bad1);
  // After the reset all previous landmarks should be unknown
  CHECK_EQ(num_unknown1, num_good0 + num_bad0);
  CHECK_EQ(num_good1, 0);
  CHECK_EQ(num_bad1, 0);

  evaluateLandmarkQuality(map);

  countLandmarkQualityInView(*map, &num_unknown1, &num_good1, &num_bad1);
  // After re-evaluating the number should be similar to what we started with
  CHECK_EQ(num_unknown0, 0);
  CHECK_LE(std::abs(num_good1 - num_good0), 0.001 * num_good0);

  // Corrupt 20% of the landmarks
  corruptLandmarks();
  evaluateLandmarkQuality(map);

  countLandmarkQualityInView(*map, &num_unknown1, &num_good1, &num_bad1);
  CHECK_EQ(num_unknown0, 0);
  const double ratio_good = static_cast<double>(num_good1) / num_good0;
  CHECK_GE(ratio_good, 0.7999);
  CHECK_LE(ratio_good, 0.90);
}

}  // namespace vi_map_helpers

MAPLAB_UNITTEST_ENTRYPOINT
