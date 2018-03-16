#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-test-helpers.h>
#include <vi-map/vi-map.h>

#include "map-optimization/optimization-problem.h"
#include "map-optimization/optimization-terms-addition.h"

DECLARE_uint64(vi_map_landmark_quality_min_observers);
DECLARE_double(vi_map_landmark_quality_min_observation_angle_deg);

namespace map_optimization {

class OptimizationTermAdditionTest : public ::testing::Test {
 public:
  void SetUp() {
    FLAGS_vi_map_landmark_quality_min_observers = 1u;
    constexpr size_t kNumAdditionalVertices = 10u;
    vi_map::test::generateMap<vi_map::ViwlsEdge>(kNumAdditionalVertices, &map);
    map.getAllMissionIds(&mission_ids_);
    num_vertices_ = map.numVertices();
    CHECK_GE(num_vertices_, kNumAdditionalVertices);
    num_landmarks_ = map.numLandmarks();
  }

 protected:
  // If these numbers need to be changed, compare with the functions in the
  // file src/optimization-terms-addition.cc adding residual blocks (search for
  // "addResidualBlock").
  size_t numConstParameterBlocksForVisualTerms() const {
    // There should be 4 different dummy values added.
    // Note: dummy values are added in the function
    // replaceUnusedArgumentsOfVisualCostFunctionWithDummies in
    // visual-error-term-factory-inl.h in ceres_error_terms. The number of
    // dummies per keypoint depends on the error term type which in turn
    // depends on whether the observing vertex is the storing vertex of the
    // keypoint and whether the storing vertex is in the same mission as the
    // observing vertex. In the case of storing vertex is not observing vertex,
    // but in the same mission, the maximum of 4 dummies are inserted.
    return 4u;
  }
  size_t numActiveParameterBlocksForVisualTerms() const {
    // Will add a parameter block for each landmark, one for each vertex, 4 for
    // the camera and 4 from the dummy values (see above).
    return num_landmarks_ + num_vertices_ + 4 + 4;
  }
  size_t numConstParameterBlocksForInertialTerms() const {
    return 0u;
  }
  size_t numActiveParameterBlocksForInertialTerms() const {
    // 4 elements per vertex (quaternion/position, gyro bias, velocity,
    // acceleration bias).
    return 4 * num_vertices_;
  }
  size_t numRepeatedConstParameterBlocksAfterAddingVisualAndInertialTerms()
      const {
    // At the moment, only visual terms add const blocks.
    return 0u;
  }
  size_t numRepeatedActiveParameterBlocksAfterAddingVisualAndInertialTerms()
      const {
    // Vertex quaternion/position are added by both inertial and visual terms.
    return num_vertices_;
  }

  static constexpr bool kFixLandmarkPositions = false;
  static constexpr bool kFixIntrinsics = false;
  static constexpr bool kFixExtrinsicsRotation = false;
  static constexpr bool kFixExtrinsicsTranslation = false;
  static constexpr size_t kMinLandmarksPerFrame = 0u;

  static constexpr bool kFixGyroBias = false;
  static constexpr bool kFixAccelBias = false;
  static constexpr bool kFixVelocity = false;
  static constexpr double kGravityMagnitude = 9.81;

  vi_map::VIMap map;
  vi_map::MissionIdSet mission_ids_;
  size_t num_vertices_;
  size_t num_landmarks_;
};

TEST_F(OptimizationTermAdditionTest, AddVisualTerms) {
  OptimizationProblem optimization_problem(&map, mission_ids_);
  addVisualTerms(
      kFixLandmarkPositions, kFixIntrinsics, kFixExtrinsicsRotation,
      kFixExtrinsicsTranslation, kMinLandmarksPerFrame, &optimization_problem);

  EXPECT_EQ(
      num_vertices_, optimization_problem.getProblemBookkeepingMutable()
                         ->keyframes_in_problem.size());
  const ceres_error_terms::ProblemInformation* problem_information =
      optimization_problem.getProblemInformationMutable();
  EXPECT_EQ(
      numConstParameterBlocksForVisualTerms(),
      problem_information->constant_parameter_blocks.size());
  EXPECT_EQ(
      numActiveParameterBlocksForVisualTerms(),
      problem_information->active_parameter_blocks.size());
}

TEST_F(OptimizationTermAdditionTest, AddInertialTerms) {
  OptimizationProblem optimization_problem(&map, mission_ids_);
  addInertialTerms(
      kFixGyroBias, kFixAccelBias, kFixVelocity, kGravityMagnitude,
      &optimization_problem);

  EXPECT_EQ(
      num_vertices_, optimization_problem.getProblemBookkeepingMutable()
                         ->keyframes_in_problem.size());

  const ceres_error_terms::ProblemInformation* problem_information =
      optimization_problem.getProblemInformationMutable();
  EXPECT_EQ(
      numConstParameterBlocksForInertialTerms(),
      problem_information->constant_parameter_blocks.size());
  EXPECT_EQ(
      numActiveParameterBlocksForInertialTerms(),
      problem_information->active_parameter_blocks.size());
}

TEST_F(OptimizationTermAdditionTest, AddVisualAndInertialTerms) {
  OptimizationProblem optimization_problem(&map, mission_ids_);

  addVisualTerms(
      kFixLandmarkPositions, kFixIntrinsics, kFixExtrinsicsRotation,
      kFixExtrinsicsTranslation, kMinLandmarksPerFrame, &optimization_problem);
  addInertialTerms(
      kFixGyroBias, kFixAccelBias, kFixVelocity, kGravityMagnitude,
      &optimization_problem);

  EXPECT_EQ(
      num_vertices_, optimization_problem.getProblemBookkeepingMutable()
                         ->keyframes_in_problem.size());

  const ceres_error_terms::ProblemInformation* problem_information =
      optimization_problem.getProblemInformationMutable();

  EXPECT_EQ(
      numConstParameterBlocksForVisualTerms() +
          numConstParameterBlocksForInertialTerms() -
          numRepeatedConstParameterBlocksAfterAddingVisualAndInertialTerms(),
      problem_information->constant_parameter_blocks.size());
  EXPECT_EQ(
      numActiveParameterBlocksForVisualTerms() +
          numActiveParameterBlocksForInertialTerms() -
          numRepeatedActiveParameterBlocksAfterAddingVisualAndInertialTerms(),
      problem_information->active_parameter_blocks.size());
}

}  // namespace map_optimization

MAPLAB_UNITTEST_ENTRYPOINT
