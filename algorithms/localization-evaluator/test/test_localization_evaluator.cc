#include <gtest/gtest.h>

#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <maplab-common/vector-window-operations.h>
#include <vi-map/vi-map.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>

#include "localization-evaluator/localization-evaluator.h"
#include "localization-evaluator/mission-aligner.h"

DECLARE_bool(lc_use_random_pnp_seed);

namespace localization_evaluator {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    FLAGS_lc_use_random_pnp_seed = false;

    test_app_.loadDataset("./test_maps/vi_app_test");
  }

  void createMap() {
    vi_map::VIMap* vi_map = test_app_.getMapMutable();
    const vi_map::MissionId& mission_id = vi_map->getIdOfFirstMission();

    vi_map->duplicateMission(mission_id);

    ASSERT_TRUE(areVertexPositionsClose(1e-5));

    vi_map::MissionIdList mission_ids;
    vi_map->getAllMissionIds(&mission_ids);

    // Randomize baseframe of the first mission.
    pose::Transformation T_G_M;
    T_G_M.getPosition() = Eigen::Vector3d(-1.2, 1294, 0.23);
    T_G_M.getRotation().setRandom();
    vi_map->getMissionBaseFrameForMission(mission_ids[0]).set_T_G_M(T_G_M);

    // Second mission with an identity baseframe.
    pose::Transformation T_G_M1;
    T_G_M1.getRotation().setIdentity();
    T_G_M1.getPosition() = Eigen::Vector3d(1, 2, 3);
    vi_map->getMissionBaseFrameForMission(mission_ids[1]).set_T_G_M(T_G_M1);

    // Slightly perturb vertex positions.
    constexpr double kPositionStdDevM = 0.25;
    constexpr double kOrientationStdDevQuat = 0.02;
    constexpr int kPerturbEveryNth = 2;
    test_app_.corruptKeyframePoses(
        kPositionStdDevM, kOrientationStdDevQuat, kPerturbEveryNth);

    vi_map::LandmarkIdList landmark_ids;
    vi_map->getAllLandmarkIds(&landmark_ids);
    for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
      vi_map->getLandmark(landmark_id)
          .setQuality(vi_map::Landmark::Quality::kGood);
    }
  }

  bool areVertexPositionsClose(const double tolerance) {
    vi_map::VIMap* vi_map = test_app_.getMapMutable();

    vi_map::MissionIdList mission_ids;
    vi_map->getAllMissionIds(&mission_ids);
    std::vector<pose_graph::VertexIdList> vertices_by_mission(
        mission_ids.size());

    int idx = 0;
    for (const vi_map::MissionId& mission_id : mission_ids) {
      vi_map->getAllVertexIdsInMissionAlongGraph(
          mission_id, &(vertices_by_mission[idx]));
      ++idx;
    }

    constexpr double kMaxDifferingFraction = 0.9;
    for (size_t i = 1u; i < mission_ids.size(); ++i) {
      CHECK_EQ(vertices_by_mission[0].size(), vertices_by_mission[i].size());

      size_t not_close = 0u;
      for (size_t j = 0u; j < vertices_by_mission[0].size(); ++j) {
        if (!vi_map->getVertex_G_p_I(vertices_by_mission[0][j])
                 .isApprox(
                     vi_map->getVertex_G_p_I(vertices_by_mission[i][j]),
                     tolerance)) {
          ++not_close;
          if (not_close >
              kMaxDifferingFraction * vertices_by_mission[0].size()) {
            return false;
          }
        }
      }
    }
    return true;
  }

  vi_map::MissionId alignMissionsForEvaluation() {
    vi_map::VIMap* vi_map = test_app_.getMapMutable();
    constexpr bool kAlignMapMissions = false;
    constexpr bool kOptimizeOnlyQueryMission = false;

    vi_map::MissionIdList mission_ids;
    vi_map->getAllMissionIds(&mission_ids);
    CHECK_GE(mission_ids.size(), 2u);
    const vi_map::MissionId query_mission_id = mission_ids.front();

    const vi_map::MissionIdSet map_mission_ids(
        mission_ids.begin() + 1, mission_ids.end());

    localization_evaluator::alignAndCooptimizeMissionsWithoutLandmarkMerge(
        mission_ids.front(), map_mission_ids, kAlignMapMissions,
        kOptimizeOnlyQueryMission, vi_map);

    return query_mission_id;
  }

  void evaluateLocalization(const vi_map::MissionId& query_mission_id) {
    vi_map::VIMap* vi_map = test_app_.getMapMutable();

    vi_map::LandmarkIdSet landmark_ids;
    vi_map->getAllLandmarkIds(&landmark_ids);

    localization_evaluator::LocalizationEvaluator evaluator(
        landmark_ids, vi_map);
    localization_evaluator::MissionEvaluationStats stats;
    evaluator.evaluateMission(query_mission_id, &stats);

    // Evaluate recall.
    const double recall = static_cast<double>(stats.successful_localizations) /
                          stats.num_vertices;
    EXPECT_GT(recall, 0.95);

    // Evaluate inlier ratios.
    CHECK_EQ(stats.lc_matches_counts.size(), stats.inliers_counts.size());
    std::vector<double> inlier_ratios;
    for (size_t i = 0u; i < stats.inliers_counts.size(); ++i) {
      inlier_ratios.push_back(
          static_cast<double>(stats.inliers_counts[i]) /
          stats.lc_matches_counts[i]);
    }
    CHECK_EQ(stats.lc_matches_counts.size(), inlier_ratios.size());

    constexpr double kInvalidValue = -1.0;
    const double avg_inlier_ratio =
        common::window_vec_ops::computeAverage(inlier_ratios, kInvalidValue);
    EXPECT_GT(avg_inlier_ratio, 0.75);
  }

 private:
  visual_inertial_mapping::VIMappingTestApp test_app_;
};

TEST_F(ViMappingTest, LocalizationEvaluatorWorks) {
  createMap();
  // Make sure the positions are initially not aligned so that there is
  // something to align and evaluate.
  ASSERT_FALSE(areVertexPositionsClose(1e-1));

  const vi_map::MissionId query_mission_id = alignMissionsForEvaluation();
  // After alignment, the poses should be aligned with certain tolerance.
  ASSERT_TRUE(areVertexPositionsClose(1e-1));

  // Once the alignment is done, verify if we can localize one mission
  // with respect to the other.
  evaluateLocalization(query_mission_id);
}

}  // namespace localization_evaluator

MAPLAB_UNITTEST_ENTRYPOINT
