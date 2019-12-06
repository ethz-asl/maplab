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
  const std::string kRobotName = "euroc_ml1";
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
  MaplabServerNodeConfig config;

  const std::string path = "./test-data/test-config.yaml";
  ASSERT_TRUE(common::fileExists(path));

  EXPECT_TRUE(config.deserializeFromFile(path));

  ASSERT_EQ(config.submap_commands.size(), 2u);
  config.submap_commands[0] = "lc";
  config.submap_commands[1] = "optvi";

  ASSERT_EQ(config.global_map_commands.size(), 3u);
  config.global_map_commands[0] = "lc";
  config.global_map_commands[1] = "optvi";
  config.global_map_commands[2] =
      "elq --vi_map_landmark_quality_min_observers=2";
}

TEST_F(MaplabServerNodeTest, TestMaplabServerNode) {
  MaplabServerNodeConfig config;
  ASSERT_TRUE(config.deserializeFromFile("./test-data/test-config.yaml"));

  MaplabServerNode maplab_server_node(config);

  maplab_server_node.start();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap0));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap1));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap2));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap3));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap4));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap5));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap6));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap7));
  std::this_thread::sleep_for(std::chrono::seconds(5));
  maplab_server_node.visualizeMap();

  maplab_server_node.shutdown();
  maplab_server_node.visualizeMap();

  EXPECT_TRUE(maplab_server_node.saveMap());
}

TEST_F(MaplabServerNodeTest, DISABLED_TestMaplabServerRosNodeLocal) {
  MaplabServerNodeConfig config;
  ASSERT_TRUE(config.deserializeFromFile("./test-data/test-config.yaml"));

  MaplabServerRosNode maplab_server_ros_node(config);

  maplab_server_ros_node.start();

  diagnostic_msgs::KeyValuePtr key_value_msg(new diagnostic_msgs::KeyValue());

  key_value_msg->key.data = kRobotName;

  key_value_msg->value.data = kSubmap0;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  key_value_msg->value.data = kSubmap1;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  key_value_msg->value.data = kSubmap2;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  key_value_msg->value.data = kSubmap3;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  key_value_msg->value.data = kSubmap4;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  key_value_msg->value.data = kSubmap5;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  key_value_msg->value.data = kSubmap6;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  key_value_msg->value.data = kSubmap7;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  maplab_server_ros_node.visualizeMap();

  maplab_server_ros_node.shutdown();

  EXPECT_TRUE(maplab_server_ros_node.saveMap());
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
