#include <ceres/ceres.h>
#include <map-manager/map-manager.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "map-optimization-legacy-plugin/vi-map-optimizer-legacy.h"

namespace visual_inertial_mapping {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/vi_app_test");
  }

  virtual void corruptVertices();
  virtual void corruptLandmarks();
  bool optimize(bool vision_only, ceres::Solver::Summary* summary);

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

bool ViMappingTest::optimize(
    bool vision_only, ceres::Solver::Summary* summary) {
  CHECK_NOTNULL(summary);
  map_optimization_legacy::BaOptimizationOptions options;
  options.include_inertial = !vision_only;
  options.fix_vertex_poses = false;
  options.fix_landmark_positions = false;
  options.fix_landmark_positions_of_fixed_vertices = false;
  options.include_wheel_odometry = false;
  options.include_gps = false;
  options.position_only_gps = false;
  options.num_iterations = 100;
  options.min_number_of_visible_landmarks_at_vertex = 1u;

  map_optimization_legacy_plugin::VIMapOptimizer optimizer;
  vi_map::VIMapManager map_manager;
  vi_map::VIMap* map = CHECK_NOTNULL(test_app_.getMapMutable());
  return optimizer.optimize(options, map, summary) == common::kSuccess;
}

TEST_F(ViMappingTest, TestIsDatasetConsistent) {
  EXPECT_TRUE(test_app_.isMapConsistent());
}

TEST_F(ViMappingTest, TestCorruptedVisualInertialOptimization) {
  corruptVertices();
  corruptLandmarks();

  const bool kVisionOnly = false;
  ceres::Solver::Summary summary;
  EXPECT_TRUE(optimize(kVisionOnly, &summary));
  EXPECT_GT(summary.num_successful_steps, summary.num_unsuccessful_steps);

  const double kPrecisionM = 0.01;
  test_app_.testIfKeyframesMatchReference(kPrecisionM);
  const double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfLandmarksMatchReference(
      kPrecisionM, kMinPassingLandmarkFraction);
}

TEST_F(ViMappingTest, TestCorruptedVisualOptimization) {
  corruptLandmarks();

  const bool kVisionOnly = true;
  ceres::Solver::Summary summary;
  EXPECT_TRUE(optimize(kVisionOnly, &summary));
  EXPECT_GT(summary.num_successful_steps, summary.num_unsuccessful_steps);

  const double kPrecisionM = 0.1;
  const double kMinPassingLandmarkFraction = 0.99;
  test_app_.testIfKeyframesMatchReference(kPrecisionM);
  test_app_.testIfLandmarksMatchReference(
      kPrecisionM, kMinPassingLandmarkFraction);
}

}  // namespace visual_inertial_mapping

MAPLAB_UNITTEST_ENTRYPOINT
