#include "maplab-server-node/maplab-server-config.h"

namespace maplab {

bool MaplabServerRosNodeConfig::deserialize(const YAML::Node& config_node) {
  if (!config_node.IsDefined() || config_node.IsNull()) {
    LOG(ERROR) << "Invalid YAML node for maplab server config deserialization.";
    return false;
  }

  if (!config_node.IsMap()) {
    LOG(WARNING) << "Maplab server config YAML node must be a map.";
    return false;
  }

  const YAML::Node& server_config_node =
      config_node[kYamlFieldNameServerConfig];
  server_config.deserialize(server_config_node);

  // Parse the robot fleet config.
  const YAML::Node& robot_fleet_node = config_node[kYamlFieldNameRobotFleet];
  if (!robot_fleet_node.IsDefined() || robot_fleet_node.IsNull()) {
    LOG(ERROR) << "Invalid maplab server config YAML. Can not parse robot "
               << "fleet config, since the '" << kYamlFieldNameRobotFleet
               << "' field is missing!";
    return false;
  }

  if (!robot_fleet_node.IsSequence()) {
    LOG(ERROR) << "Invalid maplab server config YAML. Can not parse robot "
               << "fleet config, since the '" << kYamlFieldNameRobotFleet
               << "' field is not a sequence!";
    return false;
  }
  size_t num_robots = robot_fleet_node.size();
  if (num_robots == 0) {
    LOG(ERROR) << "Invalid maplab server config YAML. Can not parse robot "
               << "fleet config, since the '" << kYamlFieldNameRobotFleet
               << "' field does not contain any entries!";
    return false;
  }

  for (size_t robot_index = 0; robot_index < num_robots; ++robot_index) {
    // Decode the camera
    const YAML::Node& robot_node = robot_fleet_node[robot_index];
    if (!robot_node) {
      LOG(ERROR) << "Unable to get YAML node for robot fleet config at index "
                 << robot_index;
      return false;
    }

    if (!robot_node.IsMap()) {
      LOG(ERROR) << "YAML node for robot fleet config at index " << robot_index
                 << " is not a map!";
      return false;
    }

    RobotConnectionConfig robot_config;

    if (YAML::safeGet(
            robot_node, kYamlFieldNameRobotName, &robot_config.name)) {
      CHECK(!robot_config.name.empty());
    } else {
      LOG(ERROR) << "YAML node for robot fleet config at index " << robot_index
                 << " does not contain the '" << kYamlFieldNameRobotName
                 << "' field!";
      return false;
    }

    if (YAML::safeGet(robot_node, kYamlFieldNameRobotIp, &robot_config.ip)) {
      CHECK(!robot_config.ip.empty());
    } else {
      LOG(ERROR) << "YAML node for robot fleet config at index " << robot_index
                 << " does not contain the '" << kYamlFieldNameRobotIp
                 << "' field!";
      return false;
    }

    if (YAML::safeGet(
            robot_node, kYamlFieldNameRobotTopic, &robot_config.topic)) {
      CHECK(!robot_config.topic.empty());
    } else {
      LOG(ERROR) << "YAML node for robot fleet config at index " << robot_index
                 << " does not contain the '" << kYamlFieldNameRobotTopic
                 << "' field!";
      return false;
    }

    if (YAML::safeGet(
            robot_node, kYamlFieldNameRobotUser, &robot_config.user)) {
      CHECK(!robot_config.user.empty());
    } else {
      LOG(ERROR) << "YAML node for robot fleet config at index " << robot_index
                 << " does not contain the '" << kYamlFieldNameRobotUser
                 << "' field!";
      return false;
    }

    connection_config.push_back(robot_config);
  }
  return true;
}

void MaplabServerRosNodeConfig::serialize(
    YAML::Node* /*config_node_ptr*/) const {
  LOG(FATAL) << "Currently not implemented!";
}

bool MaplabServerNodeConfig::deserialize(const YAML::Node& config_node) {
  if (!config_node.IsDefined() || config_node.IsNull()) {
    LOG(ERROR) << "Invalid YAML node for maplab server config deserialization.";
    return false;
  }

  if (!config_node.IsMap()) {
    LOG(WARNING) << "Maplab server config YAML node must be a map.";
    return false;
  }

  if (YAML::safeGet(config_node, kYamlFieldNameMapFolder, &map_folder)) {
    CHECK(!map_folder.empty());
  } else {
    LOG(WARNING) << "Maplab server config YAML needs a '"
                 << kYamlFieldNameMapFolder << "' field!";
    return false;
  }

  YAML::safeGet(config_node, kYamlFieldNameResourceFolder, &resource_folder);
  YAML::safeGet(
      config_node, kYamlFieldNameBackupInterval, &map_backup_interval_s);

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
