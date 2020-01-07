#include "maplab-server-node/maplab-server-config.h"

namespace maplab {

bool MaplabServerNodeConfig::deserialize(const YAML::Node& config_node) {
  if (!config_node.IsDefined() || config_node.IsNull()) {
    LOG(ERROR) << "Invalid YAML node for maplab server config deserialization.";
    return false;
  }

  if (!config_node.IsMap()) {
    LOG(WARNING) << "Maplab server config YAML node must be a map.";
    return false;
  }

  // Parse submap and global map commands.
  const YAML::Node& submap_commands_node =
      config_node[kYamlFieldNameSubmapCommands];
  if (!submap_commands_node.IsDefined() || submap_commands_node.IsNull()) {
    LOG(ERROR) << "Invalid maplab server config YAML. Can not parse "
               << "submap commands, since the '" << kYamlFieldNameSubmapCommands
               << "' field is missing!";
    return false;
  }

  if (!submap_commands_node.IsSequence()) {
    LOG(ERROR) << "Invalid maplab server config YAML. Can not parse "
               << "submap commands, since the '" << kYamlFieldNameSubmapCommands
               << "' field is not a sequence!";
    return false;
  }
  const size_t num_commands = submap_commands_node.size();

  for (size_t command_index = 0; command_index < num_commands;
       ++command_index) {
    const std::string command =
        submap_commands_node[command_index].as<std::string>();

    CHECK(!command.empty()) << "Invalid submap command in config, is empty!";

    submap_commands.push_back(command);
  }

  const YAML::Node& global_map_commands_node =
      config_node[kYamlFieldNameGlobalMapCommands];
  if (!global_map_commands_node.IsDefined() ||
      global_map_commands_node.IsNull()) {
    LOG(ERROR) << "Invalid maplab server config YAML. Can not parse "
               << "global map commands, since the '"
               << kYamlFieldNameGlobalMapCommands << "' field is missing!";
    return false;
  }

  if (!global_map_commands_node.IsSequence()) {
    LOG(ERROR) << "Invalid maplab server config YAML. Can not parse "
               << "global map commands, since the '"
               << kYamlFieldNameGlobalMapCommands
               << "' field is not a sequence!";
    return false;
  }
  const size_t num_global_commands = global_map_commands_node.size();

  for (size_t command_index = 0; command_index < num_global_commands;
       ++command_index) {
    const std::string command =
        global_map_commands_node[command_index].as<std::string>();

    CHECK(!command.empty())
        << "Invalid global map command in config, is empty!";

    global_map_commands.push_back(command);
  }
  return true;
}

void MaplabServerNodeConfig::serialize(YAML::Node* /*config_node_ptr*/) const {
  LOG(FATAL) << "Currently not implemented!";
}

}  // namespace maplab
