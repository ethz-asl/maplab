#include <gtest/gtest.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vi-mapping-test-app/vi-mapping-test-app.h>
#include <vio-common/vio-types.h>

#include "maplab-node/visual-localizer.h"

namespace maplab {

class ViMappingTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    test_app_.loadDataset("./test_maps/vi_app_test");
    summary_map_.reset(new summary_map::LocalizationSummaryMap);
  }

  void createSummaryMapAndInitLocalizer() {
    const vi_map::VIMap& vi_map = *test_app_.getMapMutable();
    vi_map::MissionIdSet mission_id_set;
    vi_map.getAllMissionIds(&mission_id_set);

    aslam::TransformationCovariance T_G_B_fixed_localization_covariance;

    // clang-format off
    T_G_B_fixed_localization_covariance <<  0.10, 0.00, 0.00, 0.00, 0.00, 0.00,
                                      0.00, 0.10, 0.00, 0.00, 0.00, 0.00,
                                      0.00, 0.00, 0.10, 0.00, 0.00, 0.00,
                                      0.00, 0.00, 0.00, 0.01, 0.00, 0.00,
                                      0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
                                      0.00, 0.00, 0.00, 0.00, 0.00, 0.01;
    // clang-format on

    CHECK_GT(mission_id_set.size(), 0);
    for (const vi_map::MissionId& mid : mission_id_set) {
      aslam::NCamera::Ptr ncam = vi_map.getMissionNCameraPtr(mid);
      ncam->set_T_G_B_fixed_localization_covariance(
          T_G_B_fixed_localization_covariance);
    }

    vi_map::LandmarkIdList landmark_ids;
    vi_map.getAllLandmarkIds(&landmark_ids);

    summary_map::createLocalizationSummaryMapFromLandmarkList(
        vi_map, landmark_ids, summary_map_.get());
    CHECK_EQ(
        summary_map_->GLandmarkPosition().cols(),
        static_cast<int>(vi_map.numLandmarks()));

    constexpr bool kVisualizeLocalization = false;

    localizer_.reset(new VisualLocalizer(
        vi_map.getSensorManager(), std::move(summary_map_),
        kVisualizeLocalization));
  }

  double evaluateRecall() {
    const vi_map::VIMap& vi_map = *test_app_.getMapMutable();

    pose_graph::VertexIdList vertex_ids;
    test_app_.getMapMutable()->getAllVertexIds(&vertex_ids);
    CHECK(!vertex_ids.empty());

    double recall = 0.;
    for (const pose_graph::VertexId& vertex_id : vertex_ids) {
      vio::LocalizationResult result;
      const bool success = localizer_->localizeNFrame(
          vi_map.getVertex(vertex_id).getVisualNFrameShared(), &result);
      if (success) {
        const double localization_error =
            (result.T_G_B.getPosition() - vi_map.getVertex_G_p_I(vertex_id))
                .norm();
        if (localization_error < kLocalizationPositionThresholdMeters) {
          ++recall;
        }
      }
    }

    recall /= vertex_ids.size();
    return recall;
  }

 private:
  VisualLocalizer::UniquePtr localizer_;
  std::unique_ptr<summary_map::LocalizationSummaryMap> summary_map_;
  visual_inertial_mapping::VIMappingTestApp test_app_;

  static constexpr double kLocalizationPositionThresholdMeters = 0.01;
};

TEST_F(ViMappingTest, DISABLED_LocalizerWithSummaryMapWorks) {
  createSummaryMapAndInitLocalizer();
  const double recall = evaluateRecall();

  constexpr double kRecallThreshold = 0.6;
  EXPECT_GT(recall, kRecallThreshold);
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
