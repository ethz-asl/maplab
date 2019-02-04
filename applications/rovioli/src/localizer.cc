#include "rovioli/localizer.h"

#include <aslam/common/occupancy-grid.h>
#include <gflags/gflags.h>
#include <localization-summary-map/localization-summary-map.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "rovioli/localizer-helpers.h"

DEFINE_uint64(
    rovioli_max_num_localization_constraints, 25u,
    "Max. number of localization constraints to process per camera. "
    "No prunning when 0.");

namespace rovioli {
Localizer::Localizer(
    const summary_map::LocalizationSummaryMap& localization_summary_map,
    const bool visualize_localization)
    : localization_summary_map_(localization_summary_map),
      map_cached_lookup_(localization_summary_map) {
  current_localization_mode_ = Localizer::LocalizationMode::kGlobal;

  global_loop_detector_.reset(new loop_detector_node::LoopDetectorNode);

  CHECK(global_loop_detector_ != nullptr);
  if (visualize_localization) {
    global_loop_detector_->instantiateVisualizer();
  }

  LOG(INFO) << "Building localization database...";
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
      result = localizeNFrameGlobal(nframe, localization_result);
      break;
    case Localizer::LocalizationMode::kMapTracking:
      result = localizeNFrameMapTracking(nframe, localization_result);
      break;
    default:
      LOG(FATAL) << "Unknown localization mode.";
      break;
  }

  localization_result->summary_map_id = localization_summary_map_.id();
  localization_result->timestamp_ns = nframe->getMinTimestampNanoseconds();
  localization_result->nframe_id = nframe->getId();
  localization_result->localization_type = current_localization_mode_;
  return result;
}

bool Localizer::localizeNFrameGlobal(
    const aslam::VisualNFrame::ConstPtr& nframe,
    vio::LocalizationResult* localization_result) const {
  CHECK_NOTNULL(localization_result);

  constexpr bool kSkipUntrackedKeypoints = false;
  unsigned int num_lc_matches;
  vi_map::VertexKeyPointToStructureMatchList inlier_structure_matches;
  const bool success = global_loop_detector_->findNFrameInSummaryMapDatabase(
      *nframe, kSkipUntrackedKeypoints, localization_summary_map_,
      &localization_result->T_G_I_lc_pnp, &num_lc_matches,
      &inlier_structure_matches);

  if (!success || inlier_structure_matches.empty()) {
    return false;
  }

  // Optionally sub-select the localizations by ensuring a good coverage over
  // the image. In case of conflicts the largest disparity angle between all
  // rays of all observations of the landmark will be used as a score.
  if (FLAGS_rovioli_max_num_localization_constraints > 0) {
    subselectStructureMatches(
        localization_summary_map_, map_cached_lookup_, *nframe,
        FLAGS_rovioli_max_num_localization_constraints,
        &inlier_structure_matches);
  }
  convertVertexKeyPointToStructureMatchListToLocalizationResult(
      localization_summary_map_, *nframe, inlier_structure_matches,
      localization_result);

  return true;
}

bool Localizer::localizeNFrameMapTracking(
    const aslam::VisualNFrame::ConstPtr& /*nframe*/,
    vio::LocalizationResult* /*localization_result*/) const {
  LOG(FATAL) << "Not implemented yet.";
  return false;
}

}  // namespace rovioli
