#include "loop-closure-plugin/loop-detector-serialization.h"

#include <console-common/command-registerer.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <loop-closure-handler/loop-detector-node.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace loop_closure_plugin {

int generateLoopDetectorForVIMapAndSerialize(
    const std::string& map_folder, const vi_map::VIMap& vi_map) {
  loop_detector_node::LoopDetectorNode loop_detector_node;
  vi_map::MissionIdList mission_ids;
  vi_map.getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    LOG(ERROR)
        << "No missions in the currently loaded map(s). Cannot serialize "
        << "a loop-detector node without any missions.";
    return common::kStupidUserError;
  }

  for (const vi_map::MissionId& mission : mission_ids) {
    loop_detector_node.addMissionToDatabase(mission, vi_map);
  }

  const std::string loop_detector_serialization_filename =
      loop_detector_node.getDefaultSerializationFilename();
  std::string loop_detector_serialization_filepath;
  common::concatenateFolderAndFileName(
      map_folder, loop_detector_serialization_filename,
      &loop_detector_serialization_filepath);
  VLOG(1) << "Serializing loop detector to "
          << loop_detector_serialization_filepath;
  if (!loop_detector_node.serializeToFile(
          loop_detector_serialization_filepath)) {
    LOG(ERROR) << "Failed to serialize loop detector!";
    return common::kUnknownError;
  }
  return common::kSuccess;
}

}  // namespace loop_closure_plugin
