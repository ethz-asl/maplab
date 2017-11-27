#include "rovioli/localizer.h"

#include <localization-summary-map/localization-summary-map.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <vio-common/vio-types.h>

namespace rovioli {

Localizer::Localizer(
    const summary_map::LocalizationSummaryMap& localization_summary_map,
    const bool visualize_localization)
    : localization_summary_map_(localization_summary_map) {
  current_localization_mode_ = Localizer::LocalizationMode::kGlobal;

  global_loop_detector_.reset(new loop_detector_node::LoopDetectorNode);

  CHECK(global_loop_detector_ != nullptr);
  if (visualize_localization) {
    global_loop_detector_->instantiateVisualizer();
  }

  LOG(INFO) << "Creating localization database...";
  global_loop_detector_->addLocalizationSummaryMapToDatabase(
      localization_summary_map_);
  LOG(INFO) << "Done.";
}

Localizer::LocalizationMode Localizer::getCurrentLocalizationMode() const {
  return current_localization_mode_;
}

bool Localizer::localizeNFrame(
    const aslam::VisualNFrame::ConstPtr& nframe,
    vio::LocalizationResult* localization_result) const {
  CHECK(nframe);
  CHECK_NOTNULL(localization_result);

  bool result = false;
  switch (current_localization_mode_) {
    case Localizer::LocalizationMode::kGlobal:
      result = localizeNFrameGlobal(nframe, &localization_result->T_G_I_lc_pnp);
      break;
    case Localizer::LocalizationMode::kMapTracking:
      result =
          localizeNFrameMapTracking(nframe, &localization_result->T_G_I_lc_pnp);
      break;
    default:
      LOG(FATAL) << "Unknown localization mode.";
      break;
  }

  localization_result->timestamp = nframe->getMinTimestampNanoseconds();
  localization_result->nframe_id = nframe->getId();
  localization_result->localization_type = current_localization_mode_;
  return result;
}

bool Localizer::localizeNFrameGlobal(
    const aslam::VisualNFrame::ConstPtr& nframe,
    aslam::Transformation* T_G_I_lc_pnp) const {
  constexpr bool kSkipUntrackedKeypoints = false;
  unsigned int num_lc_matches;
  vi_map::VertexKeyPointToStructureMatchList inlier_structure_matches;
  return global_loop_detector_->findNFrameInSummaryMapDatabase(
      *nframe, kSkipUntrackedKeypoints, localization_summary_map_, T_G_I_lc_pnp,
      &num_lc_matches, &inlier_structure_matches);
}

bool Localizer::localizeNFrameMapTracking(
    const aslam::VisualNFrame::ConstPtr& /*nframe*/,
    aslam::Transformation* /*T_G_I_lc_pnp*/) const {
  LOG(FATAL) << "Not implemented yet.";
  return false;
}

}  // namespace rovioli
