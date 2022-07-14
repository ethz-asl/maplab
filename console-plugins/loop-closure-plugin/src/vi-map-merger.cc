#include "loop-closure-plugin/vi-map-merger.h"

#include <loop-closure-handler/loop-detector-node.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/vi-map.h>

namespace loop_closure_plugin {

DEFINE_bool(
    lc_only_against_other_missions, false,
    "If true, no inter-mission loop-closures are sought.");

DEFINE_bool(
    lc_against_cumulative_map, false,
    "Cumulate all landmarks simultaneously into the database and query "
    "against that. Only to be used as a refinement step at the end, because "
    "otherwise any misalignments will introduce bad associations.");

VIMapMerger::VIMapMerger(
    vi_map::VIMap* map, const visualization::ViwlsGraphRvizPlotter* plotter)
    : map_(map), plotter_(plotter) {
  CHECK_NOTNULL(map_);
}

int VIMapMerger::findLoopClosuresBetweenAllMissions() {
  if (map_->numMissions() == 0u) {
    LOG(ERROR) << "No missions in database.";
    return kNoData;
  }

  vi_map::MissionIdList mission_ids;
  map_->getAllMissionIds(&mission_ids);

  return findLoopClosuresBetweenMissions(mission_ids);
}

int VIMapMerger::findLoopClosuresBetweenMissions(
    const vi_map::MissionIdList& mission_ids) {
  VLOG(1) << "Trying to find loop-closures in and between "
          << mission_ids.size() << " missions.";

  if (mission_ids.empty()) {
    LOG(ERROR) << "There are no missions in the loaded map. Aborting.";
    return common::kUnknownError;
  }

  if (FLAGS_lc_against_cumulative_map) {
    CHECK(!FLAGS_lc_only_against_other_missions)
        << "When using a cumulative map all the missions are included in the "
           "query database and queried against. They can not be separated.";

    // Create joint global map of landmarks for queries
    loop_detector_node::LoopDetectorNode loop_detector;
    if (plotter_ != nullptr) {
      loop_detector.instantiateVisualizer();
    }

    for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
         it != mission_ids.end(); ++it) {
      CHECK(it->isValid());
      loop_detector.addMissionToDatabase(*it, *map_);
    }

    // Attempt to merge landmarks from each mission against the entire database
    for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
         it != mission_ids.end(); ++it) {
      loop_detector.detectLoopClosuresAndMergeLandmarks(*it, map_);
    }
  } else {
    // We want to try to loop close every mission pair
    for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
         it != mission_ids.end(); ++it) {
      CHECK(it->isValid());
      loop_detector_node::LoopDetectorNode loop_detector;
      if (plotter_ != nullptr) {
        loop_detector.instantiateVisualizer();
      }
      loop_detector.addMissionToDatabase(*it, *map_);
      for (vi_map::MissionIdList::const_iterator jt = mission_ids.begin();
           jt != mission_ids.end(); ++jt) {
        if (FLAGS_lc_only_against_other_missions && *jt == *it) {
          continue;
        }
        loop_detector.detectLoopClosuresAndMergeLandmarks(*jt, map_);
      }
    }
  }

  return common::kSuccess;
}

}  // namespace loop_closure_plugin
