#include <memory>

#include <descriptor-projection/flags.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <loopclosure-common/flags.h>
#include <loopclosure-common/types.h>
#include <map-optimization-legacy-plugin/vi-map-optimizer-legacy.h>
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
    FLAGS_feature_descriptor_type = loop_closure::kFeatureDescriptorFREAK;
    FLAGS_lc_scoring_function =
        matching_based_loopclosure::kProbabilisticString;
    FLAGS_lc_use_random_pnp_seed = false;

    test_app_.loadDataset("./test_maps/lc_app_test");
    constexpr std::nullptr_t kPlotterNullptr = nullptr;
    map_merger_.reset(
        new VIMapMerger(test_app_.getMapMutable(), kPlotterNullptr));
  }

  bool optimize(ceres::Solver::Summary* summary);

  visual_inertial_mapping::VIMappingTestApp test_app_;
  std::unique_ptr<VIMapMerger> map_merger_;
};

bool LoopClosureAppTest::optimize(ceres::Solver::Summary* summary) {
  CHECK_NOTNULL(summary);
  map_optimization_legacy::BaOptimizationOptions options;
  options.num_iterations = 20;
  options.include_loop_closure_edges = true;
  options.include_visual = true;
  options.include_inertial = true;

  map_optimization_legacy_plugin::VIMapOptimizer optimizer;
  vi_map::VIMap* map = CHECK_NOTNULL(test_app_.getMapMutable());
  return optimizer.optimize(options, map, summary) == common::kSuccess;
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
  ceres::Solver::Summary summary;
  EXPECT_TRUE(optimize(&summary));

  const Eigen::Vector3d start_vertex_G_p_I =
      map_ptr->getVertex_G_p_I(start_vertex);
  const Eigen::Vector3d end_vertex_G_p_I = map_ptr->getVertex_G_p_I(end_vertex);
  const double final_drift_meters =
      (start_vertex_G_p_I - end_vertex_G_p_I).norm();
  EXPECT_LE(final_drift_meters, kUpperDriftLimitMeters);
}

}  // namespace loop_closure_plugin

MAPLAB_UNITTEST_ENTRYPOINT
