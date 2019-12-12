#include "maplab-node/visual-localizer.h"

#include <memory>

#include <aslam/common/occupancy-grid.h>
#include <gflags/gflags.h>
#include <localization-summary-map/localization-summary-map-cache.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map-queries.h>
#include <localization-summary-map/localization-summary-map.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>

#include "maplab-node/visual-localizer-helpers.h"

DEFINE_uint64(
    max_num_localization_constraints, 25u,
    "Max. number of localization constraints to process per camera. "
    "No prunning when 0.");

namespace maplab {
VisualLocalizer::VisualLocalizer(
    const vi_map::SensorManager& sensor_manager,
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map,
    const bool visualize_localization)
    : VisualLocalizer(sensor_manager, visualize_localization) {
  setLocalizationMap(std::move(localization_map));
}

VisualLocalizer::VisualLocalizer(
    const vi_map::SensorManager& sensor_manager,
    const bool visualize_localization)
    : sensor_manager_(sensor_manager) {
  current_localization_mode_ = VisualLocalizer::LocalizationMode::kGlobal;

  global_loop_detector_.reset(new loop_detector_node::LoopDetectorNode);
  CHECK(global_loop_detector_);

  if (visualize_localization) {
    global_loop_detector_->instantiateVisualizer();
  }
}

VisualLocalizer::LocalizationMode VisualLocalizer::getCurrentLocalizationMode()
    const {
  return current_localization_mode_;
}

void VisualLocalizer::setLocalizationMap(
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map) {
  CHECK(localization_map);
  localization_map_ = std::move(localization_map);

  // Update localization map cache.
  map_cached_lookup_.reset(
      new summary_map::SummaryMapCachedLookups(*localization_map_));

  LOG(INFO) << "Building localization database...";
  global_loop_detector_->addLocalizationSummaryMapToDatabase(
      *localization_map_);
  LOG(INFO) << "Done.";
}

bool VisualLocalizer::localizeNFrame(
    const aslam::VisualNFrame::ConstPtr& nframe,
    vio::LocalizationResult* localization_result) const {
  CHECK(nframe);
  CHECK_NOTNULL(localization_result);

  if (!localization_map_) {
    LOG(WARNING) << "Localization failed: The localizer doesn't have a visual "
                    "localization map (yet)!";
    return false;
  }
  CHECK(map_cached_lookup_)
      << "SummaryMapCachedLookups has not been initialized!";

  bool result = false;
  switch (current_localization_mode_) {
    case VisualLocalizer::LocalizationMode::kGlobal:
      result = localizeNFrameGlobal(nframe, localization_result);
      break;
    case VisualLocalizer::LocalizationMode::kMapTracking:
      result = localizeNFrameMapTracking(nframe, localization_result);
      break;
    default:
      LOG(FATAL) << "Unknown localization mode.";
      break;
  }

  localization_result->summary_map_id = localization_map_->id();
  localization_result->timestamp_ns = nframe->getMinTimestampNanoseconds();
  localization_result->nframe_id = nframe->getId();
  localization_result->sensor_id = nframe->getNCamera().getId();
  localization_result->localization_mode = current_localization_mode_;
  // todo(nscheidt): compute a localization covariance rather than use a fixed
  // value from a config
  CHECK(nframe->getNCamera().getFixedLocalizationCovariance(
      &localization_result->T_G_B_covariance))
      << "Currently, a fixed covariance value is required for visual "
      << "localization. Please provide it as T_G_B_fixed_covariance in the "
      << "NCamera config file.";
  return result;
}

bool VisualLocalizer::localizeNFrameGlobal(
    const aslam::VisualNFrame::ConstPtr& nframe,
    vio::LocalizationResult* localization_result) const {
  CHECK_NOTNULL(localization_result);
  CHECK(localization_map_);

  constexpr bool kSkipUntrackedKeypoints = false;
  unsigned int num_lc_matches;
  vi_map::VertexKeyPointToStructureMatchList inlier_structure_matches;
  const bool success = global_loop_detector_->findNFrameInSummaryMapDatabase(
      *nframe, kSkipUntrackedKeypoints, *localization_map_,
      &localization_result->T_G_B, &num_lc_matches, &inlier_structure_matches);

  if (!success || inlier_structure_matches.empty()) {
    return false;
  }

  // Optionally sub-select the localizations by ensuring a good coverage over
  // the image. In case of conflicts the largest disparity angle between all
  // rays of all observations of the landmark will be used as a score.
  if (FLAGS_max_num_localization_constraints > 0) {
    subselectStructureMatches(
        *localization_map_, *map_cached_lookup_, *nframe,
        FLAGS_max_num_localization_constraints, &inlier_structure_matches);
  }
  convertVertexKeyPointToStructureMatchListToLocalizationResult(
      *localization_map_, *nframe, inlier_structure_matches,
      localization_result);

  return true;
}

bool VisualLocalizer::localizeNFrameMapTracking(
    const aslam::VisualNFrame::ConstPtr& /*nframe*/,
    vio::LocalizationResult* /*localization_result*/) const {
  CHECK(localization_map_);

  LOG(FATAL) << "Not implemented yet.";
  return false;
}

}  // namespace maplab
