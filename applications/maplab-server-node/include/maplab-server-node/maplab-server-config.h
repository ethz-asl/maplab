#ifndef MAPLAB_SERVER_NODE_MAPLAB_SERVER_CONFIG_H_
#define MAPLAB_SERVER_NODE_MAPLAB_SERVER_CONFIG_H_

#include <string>
#include <vector>

#include <aslam/common/yaml-file-serialization.h>
#include <aslam/common/yaml-serialization.h>

namespace maplab {

class MaplabServerNodeConfig : public aslam::YamlFileSerializable {
 public:
  // Server config:
  // Where the finished/intermediate maps should be stored. Not optional.
  std::string map_folder = "";

  // If empty, the standard map resource folder is used.
  std::string resource_folder = "";

  // Create a backup of the current map every n seconds. 0 = no backups.
  size_t map_backup_interval_s = 0u;

  std::vector<std::string> submap_commands;
  std::vector<std::string> global_map_commands;

  bool deserialize(const YAML::Node& config_node) override;
  void serialize(YAML::Node* config_node_ptr) const override;

  const std::string kYamlFieldNameMapFolder = "map_folder";
  const std::string kYamlFieldNameResourceFolder = "resource_folder";
  const std::string kYamlFieldNameBackupInterval = "map_backup_interval_s";

  const std::string kYamlFieldNameSubmapCommands = "submap_commands";
  const std::string kYamlFieldNameGlobalMapCommands = "global_map_commands";
};

struct RobotConnectionConfig {
  std::string name;

  // Topic on which map updates and locations are published.
  std::string topic;

  // Login info to transfer maps from the robot.
  std::string ip;
  std::string user;
};

struct MaplabServerRosNodeConfig : public aslam::YamlFileSerializable {
  MaplabServerNodeConfig server_config;

  // Robot fleet config:
  std::vector<RobotConnectionConfig> connection_config;

  bool deserialize(const YAML::Node& config_node) override;
  void serialize(YAML::Node* config_node_ptr) const override;

  const std::string kYamlFieldNameServerConfig = "server_config";

  const std::string kYamlFieldNameRobotFleet = "connection_config";
  const std::string kYamlFieldNameRobotName = "name";
  const std::string kYamlFieldNameRobotIp = "ip";
  const std::string kYamlFieldNameRobotTopic = "topic";
  const std::string kYamlFieldNameRobotUser = "user";
};

}  // namespace maplab

#endif  // MAPLAB_SERVER_NODE_MAPLAB_SERVER_CONFIG_H_
