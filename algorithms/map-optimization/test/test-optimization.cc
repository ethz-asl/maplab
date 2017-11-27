#include <ceres/ceres.h>
#include <map-manager/map-manager.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "map-optimization/vi-map-optimizer.h"

namespace visual_inertial_mapping {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/vi_app_test");
  }

  virtual void corruptVertices();
  virtual void corruptLandmarks();
  bool optimize(bool vision_only);

  VIMappingTestApp test_app_;
};

void ViMappingTest::corruptVertices() {
  const double kKeyframePositionStdDevM = 0.25;
  const double kOrientationStdDevQuat = 0.05;
  const int kEveryNthToCorrupt = 3;
  test_app_.corruptKeyframePoses(
      kKeyframePositionStdDevM, kOrientationStdDevQuat, kEveryNthToCorrupt);
}

void ViMappingTest::corruptLandmarks() {
  const double kLandmarkPositionStdDevM = 0.25;
  const int kEveryNthToCorrupt = 3;
  test_app_.corruptLandmarkPositions(
      kLandmarkPositionStdDevM, kEveryNthToCorrupt);
}

bool ViMappingTest::optimize(bool vision_only) {
  map_optimization::ViProblemOptions options =
      map_optimization::ViProblemOptions::initFromGFlags();
  options.fix_landmark_positions = vision_only;

  visualization::ViwlsGraphRvizPlotter* plotter = nullptr;
  constexpr bool kSignalHandlerEnabled = false;
  map_optimization::VIMapOptimizer optimizer(plotter, kSignalHandlerEnabled);

  vi_map::VIMapManager map_manager;
  vi_map::VIMap* map = CHECK_NOTNULL(test_app_.getMapMutable());

  vi_map::MissionIdSet mission_ids;
  map->getAllMissionIds(&mission_ids);

  map_optimization::OutlierRejectionSolverOptions rejection_options =
      map_optimization::OutlierRejectionSolverOptions::initFromFlags();
  return optimizer.optimizeVisualInertial(
      options, mission_ids, &rejection_options, map);
}

TEST_F(ViMappingTest, TestIsDatasetConsistent) {
  EXPECT_TRUE(test_app_.isMapConsistent());
}

TEST_F(ViMappingTest, TestCorruptedVisualInertialOptimization) {
  corruptVertices();
  corruptLandmarks();

  const bool kVisionOnly = false;
  EXPECT_TRUE(optimize(kVisionOnly));

  const double kPrecisionM = 0.01;
  test_app_.testIfKeyframesMatchReference(kPrecisionM);
  const double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionM, kMinPassingLandmarkFraction);
}

TEST_F(ViMappingTest, TestCorruptedVisualOptimization) {
  corruptLandmarks();

  const bool kVisionOnly = true;
  EXPECT_TRUE(optimize(kVisionOnly));

  const double kPrecisionM = 0.1;
  const double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfKeyframesMatchReference(kPrecisionM);
  test_app_.testIfLandmarksMatchReference(
      kPrecisionM, kMinPassingLandmarkFraction);
}

}  // namespace visual_inertial_mapping

MAPLAB_UNITTEST_ENTRYPOINT
