#include "loop-closure-plugin/vi-map-merger.h"

#include <loop-closure-handler/loop-detector-node.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/vi-map.h>

namespace loop_closure_plugin {

DEFINE_bool(
    lc_only_against_other_missions, false,
    "If true, no inter-mission loop-closures are sought.");

VIMapMerger::VIMapMerger(
    vi_map::VIMap* map, visualization::ViwlsGraphRvizPlotter* plotter)
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
  std::string map_folder = "";
  if (map_->hasMapFolder()) {
    map_->getMapFolder(&map_folder);
  }

  const std::string& loop_detector_serialization_filename =
      loop_detector_node::LoopDetectorNode::getDefaultSerializationFilename();

  std::string loop_detector_serialization_filepath;
  common::concatenateFolderAndFileName(
      map_folder, loop_detector_serialization_filename,
      &loop_detector_serialization_filepath);

  if (common::fileExists(loop_detector_serialization_filepath)) {
    VLOG(1) << "Using serialized loop-detector from file "
            << loop_detector_serialization_filename << '.';
    loop_detector_node::LoopDetectorNode loop_detector;
    if (plotter_ != nullptr) {
      loop_detector.instantiateVisualizer();
    }
    CHECK(
        loop_detector.deserializeFromFile(
            loop_detector_serialization_filepath));
    LOG_IF(WARNING, FLAGS_lc_only_against_other_missions)
        << "Flag -lc_skip_self_lc is set "
        << "to true but has no effect since the loop-closure database is "
        << "loaded from file: " << loop_detector_serialization_filepath << '.';
    for (const vi_map::MissionId& mission_id : mission_ids) {
      CHECK(mission_id.isValid());
      loop_detector.detectLoopClosuresAndMergeLandmarks(mission_id, map_);
    }
  } else {
    // We want to match all missions to all, so we do the full upper triangle
    // including the matching of every mission to itself to find loop-closures
    // within the mission.
    for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
         it != mission_ids.end(); ++it) {
      CHECK(it->isValid());
      loop_detector_node::LoopDetectorNode loop_detector;
      if (plotter_ != nullptr) {
        loop_detector.instantiateVisualizer();
      }
      loop_detector.addMissionToDatabase(*it, *map_);
      for (vi_map::MissionIdList::const_iterator jt = it;
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
