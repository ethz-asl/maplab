#include <iostream>

#include <console-common/console.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>
#include <yaml-cpp/yaml.h>

#include "maplab-console/maplab-console.h"

// This executable reads yaml files and executes the commands from it on all
// maps in the list. The files have the following format:
// The template string "<CURRENT_VIMAP_FOLDER>" is replaced by the currently
// processed map. This can be used e.g. to save the resulting map to a
// different folder:
//    save -vi_map_default_folder=<CURRENT_VIMAP_FOLDER>_result
//
// Yaml-format:
//   vi_map_folder_paths:
//     - /some/path/map1
//     - /some/path/map2
//   commands:
//     - command1
//     - command2
//     - command3

const std::string kMapFolderTemplate("<CURRENT_VIMAP_FOLDER>");
const std::string kConsoleName = "maplab-batch-runner";

struct BatchControlInformation {
  std::vector<std::string> vi_map_folder_paths;
  std::vector<std::string> commands;
};

namespace YAML {
template <>
struct convert<BatchControlInformation> {
  static Node encode(const BatchControlInformation& rhs) {
    Node node;
    node["vi_map_folder_paths"] = rhs.vi_map_folder_paths;
    node["commands"] = rhs.commands;
    return node;
  }
  static bool decode(const Node& node, BatchControlInformation& rhs) {
    rhs.vi_map_folder_paths =
        node["vi_map_folder_paths"].as<std::vector<std::string> >();
    rhs.commands = node["commands"].as<std::vector<std::string> >();
    return true;
  }
};
}  // namespace YAML

DEFINE_string(
    batch_control_file, "",
    "Filename of the yaml file that "
    "contains the batch processing information.");

bool replaceSubstring(
    const std::string& from, const std::string& to, std::string* full_string) {
  CHECK_NOTNULL(full_string);
  size_t start_pos = full_string->find(from);
  if (start_pos == std::string::npos) {
    return false;
  }
  full_string->replace(start_pos, from.length(), to);
  return true;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  CHECK_NE(FLAGS_batch_control_file, "")
      << "You have to provide the path to the batch control yaml-file.";

  visualization::ViwlsGraphRvizPlotter::Ptr plotter(
      new visualization::ViwlsGraphRvizPlotter());

  BatchControlInformation control_information;
  if (!YAML::Load(FLAGS_batch_control_file, &control_information)) {
    LOG(FATAL) << "Failed to read batch control file: "
               << FLAGS_batch_control_file;
  }

  const size_t num_maps = control_information.vi_map_folder_paths.size();
  const size_t num_cmds = control_information.commands.size();
  LOG_IF(FATAL, num_maps == 0u) << "No maps supplied with file: "
                               << FLAGS_batch_control_file;
  LOG_IF(FATAL, num_cmds == 0u) << "No commands supplied with file: "
                               << FLAGS_batch_control_file;

  LOG(INFO) << "Got " << num_cmds << " commands to apply on " << num_maps
            << " maps.";

  // Process all commands for all maps.
  maplab::MapLabConsole console(kConsoleName, argc, argv);

  size_t map_idx = 1u;
  for (const std::string& map_folder :
       control_information.vi_map_folder_paths) {
    LOG(INFO) << "Running map (" << map_idx << " / " << num_maps
              << "): " << map_folder;

    // Release all maps from memory.
    vi_map::VIMapManager map_manager;
    std::unordered_set<std::string> all_map_keys;
    map_manager.getAllMapKeys(&all_map_keys);
    for (const std::string& key : all_map_keys) {
      map_manager.deleteMap(key);
    }
    const std::string kNoMapSelected = "";
    console.setSelectedMapKey(kNoMapSelected);

    // Run all commands on this map.
    size_t cmd_idx = 1u;
    for (const std::string& command : control_information.commands) {
      // Replace the map_folder template string for the current command.
      std::string actual_command = command;
      replaceSubstring(kMapFolderTemplate, map_folder, &actual_command);

      LOG(INFO) << "\t Running command (" << cmd_idx << " / " << num_cmds
                << "): " << actual_command;

      // Run the command.
      if (console.RunCommand(actual_command) != common::kSuccess) {
        LOG(ERROR) << "\t Command failed!";
      } else {
        LOG(INFO) << "\t Command successful.";
      }
      ++cmd_idx;
    }

    LOG(INFO) << "Done running map.";
    ++map_idx;
  }

  LOG(INFO) << "Done. Processed " << num_cmds << " commands for " << num_maps
            << " maps.";
}
