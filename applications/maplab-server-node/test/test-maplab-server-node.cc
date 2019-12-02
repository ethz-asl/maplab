#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <maplab-server-node/maplab-server-config.h>
#include <maplab-server-node/maplab-server-node.h>
#include <maplab-server-node/maplab-server-ros-node.h>

DECLARE_bool(overwrite);

namespace maplab {

class MaplabServerNodeTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    FLAGS_overwrite = true;
  }

 public:
  const std::string kBasePath = "./test_maps/submap_test/";
  const std::string kSubmap0 = kBasePath + "euroc_v1_01_submap_0";
  const std::string kSubmap1 = kBasePath + "euroc_v1_01_submap_1";
  const std::string kSubmap2 = kBasePath + "euroc_v1_01_submap_2";
  const std::string kSubmap3 = kBasePath + "euroc_v1_01_submap_3";
  const std::string kSubmap4 = kBasePath + "euroc_v1_01_submap_4";
  const std::string kSubmap5 = kBasePath + "euroc_v1_01_submap_5";
  const std::string kSubmap6 = kBasePath + "euroc_v1_01_submap_6";
  const std::string kSubmap7 = kBasePath + "euroc_v1_01_submap_7";
};

TEST_F(MaplabServerNodeTest, TestLoadingConfig) {
  MaplabServerRosNodeConfig config;

  const std::string path = "./test-data/test-config.yaml";
  ASSERT_TRUE(common::fileExists(path));

  EXPECT_TRUE(config.deserializeFromFile(path));

  EXPECT_EQ(
      config.server_config.map_folder, "/tmp/maplab_server_node/combined_map");
  EXPECT_EQ(
      config.server_config.resource_folder,
      "/tmp/maplab_server_node/combined_map_resources");
  EXPECT_EQ(config.server_config.map_backup_interval_s, 300);

  ASSERT_EQ(config.server_config.submap_commands.size(), 2u);
  config.server_config.submap_commands[0] = "lc";
  config.server_config.submap_commands[1] = "optvi";

  ASSERT_EQ(config.server_config.global_map_commands.size(), 3u);
  config.server_config.global_map_commands[0] = "lc";
  config.server_config.global_map_commands[1] = "optvi";
  config.server_config.global_map_commands[2] =
      "elq --vi_map_landmark_quality_min_observers=2";

  ASSERT_EQ(config.connection_config.size(), 1u);

  const RobotConnectionConfig& robot_connection = config.connection_config[0];
  ASSERT_EQ(robot_connection.name, "local_maplab_node");
  ASSERT_EQ(robot_connection.topic, "/maplab_node/map_update_notification");
  ASSERT_EQ(robot_connection.ip, "localhost");
  ASSERT_EQ(robot_connection.user, "user");
}

TEST_F(MaplabServerNodeTest, TestMaplabServerNode) {
  MaplabServerRosNodeConfig config;
  ASSERT_TRUE(config.deserializeFromFile("./test-data/test-config.yaml"));

  MaplabServerNode maplab_server_node(config.server_config);

  maplab_server_node.start();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap0));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap1));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap2));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap3));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap4));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap5));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap6));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kSubmap7));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  maplab_server_node.shutdown();
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.saveMap());
}

TEST_F(MaplabServerNodeTest, DISABLED_TestMaplabServerRosNodeLocal) {
  MaplabServerRosNodeConfig config;
  ASSERT_TRUE(config.deserializeFromFile("./test-data/test-config.yaml"));

  MaplabServerRosNode maplab_server_ros_node(config);

  maplab_server_ros_node.start();

  ASSERT_EQ(config.connection_config.size(), 1u);
  RobotConnectionConfig robot_connection_config = config.connection_config[0];
  ASSERT_EQ(robot_connection_config.ip, "localhost");

  std_msgs::StringPtr string_msgs(new std_msgs::String());

  string_msgs->data = kSubmap0;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  string_msgs->data = kSubmap1;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  string_msgs->data = kSubmap2;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  string_msgs->data = kSubmap3;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  string_msgs->data = kSubmap4;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  string_msgs->data = kSubmap5;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  string_msgs->data = kSubmap6;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  string_msgs->data = kSubmap7;
  maplab_server_ros_node.submapLoadingCallback(
      string_msgs, robot_connection_config);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  maplab_server_ros_node.visualizeMap();

  maplab_server_ros_node.shutdown();

  EXPECT_TRUE(maplab_server_ros_node.saveMap());
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
