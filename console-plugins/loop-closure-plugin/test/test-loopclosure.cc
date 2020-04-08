#include <memory>

#include <descriptor-projection/flags.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <loopclosure-common/flags.h>
#include <loopclosure-common/types.h>
#include <map-optimization/solver-options.h>
#include <map-optimization/vi-map-optimizer.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <matching-based-loopclosure/detector-settings.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "loop-closure-plugin/vi-map-merger.h"

DECLARE_string(lc_detector_engine);
DECLARE_string(lc_scoring_function);
DECLARE_bool(lc_use_random_pnp_seed);

namespace loop_closure_plugin {

class LoopClosureAppTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    FLAGS_lc_detector_engine =
        matching_based_loopclosure::kMatchingLDInvertedMultiIndexString;
    FLAGS_feature_descriptor_type = loop_closure::kFeatureDescriptorBRISK;
    FLAGS_lc_scoring_function =
        matching_based_loopclosure::kProbabilisticString;
    FLAGS_lc_use_random_pnp_seed = false;

    test_app_.loadDataset("./test_maps/lc_app_test");
    constexpr std::nullptr_t kPlotterNullptr = nullptr;
    map_merger_.reset(
        new VIMapMerger(test_app_.getMapMutable(), kPlotterNullptr));
  }

  bool optimize();

  visual_inertial_mapping::VIMappingTestApp test_app_;
  std::unique_ptr<VIMapMerger> map_merger_;
};

bool LoopClosureAppTest::optimize() {
  vi_map::VIMap* map = CHECK_NOTNULL(test_app_.getMapMutable());

  map_optimization::ViProblemOptions vi_problem_options =
      map_optimization::ViProblemOptions::initFromGFlags();

  vi_problem_options.solver_options.max_num_iterations = 20;
  vi_problem_options.enable_visual_outlier_rejection = false;
  vi_problem_options.add_loop_closure_edges = true;

  // Initial visual-inertial optimization.
  constexpr bool kEnableSignalHandler = true;
  map_optimization::VIMapOptimizer optimizer(nullptr, kEnableSignalHandler);

  // Optimize all missions in the map
  vi_map::MissionIdSet all_mission_ids;
  map->getAllMissionIds(&all_mission_ids);

  const bool success =
      optimizer.optimize(vi_problem_options, all_mission_ids, map);
  return success;
}

TEST_F(LoopClosureAppTest, TestIsDatasetConsistent) {
  EXPECT_TRUE(test_app_.isMapConsistent());
}

// The provided map's start and end vertices should have almost identical
// position. However, due to the accumulated drift, the final vertex has a
// position offset. To test the loop-closure pipeline, loop-closure detection
// is performed first and followed up by bundle adjustment to minimize the
// drift.
TEST_F(LoopClosureAppTest, TestLoopClosureWithOptimization) {
  vi_map::VIMap* map_ptr = test_app_.getMapMutable();
  vi_map::MissionIdList mission_ids;
  map_ptr->getAllMissionIds(&mission_ids);
  CHECK_EQ(mission_ids.size(), 1u);
  pose_graph::VertexIdList vertex_id_list;
  map_ptr->getAllVertexIdsInMissionAlongGraph(mission_ids[0], &vertex_id_list);
  CHECK(!vertex_id_list.empty());

  const pose_graph::VertexId start_vertex = vertex_id_list.front();
  const pose_graph::VertexId end_vertex = vertex_id_list.back();

  const Eigen::Vector3d start_vertex_G_p_I_before_lc =
      map_ptr->getVertex_G_p_I(start_vertex);
  const Eigen::Vector3d end_vertex_G_p_I_before_lc =
      map_ptr->getVertex_G_p_I(end_vertex);

  constexpr double kUpperDriftLimitMeters = 0.05;
  const double initial_drift_meters =
      (start_vertex_G_p_I_before_lc - end_vertex_G_p_I_before_lc).norm();
  CHECK_GT(initial_drift_meters, kUpperDriftLimitMeters);

  landmark_triangulation::retriangulateLandmarks(map_ptr);
  map_merger_->findLoopClosuresBetweenAllMissions();
  EXPECT_TRUE(optimize());

  const Eigen::Vector3d start_vertex_G_p_I =
      map_ptr->getVertex_G_p_I(start_vertex);
  const Eigen::Vector3d end_vertex_G_p_I = map_ptr->getVertex_G_p_I(end_vertex);
  const double final_drift_meters =
      (start_vertex_G_p_I - end_vertex_G_p_I).norm();
  EXPECT_LE(final_drift_meters, kUpperDriftLimitMeters);
}

}  // namespace loop_closure_plugin

MAPLAB_UNITTEST_ENTRYPOINT
