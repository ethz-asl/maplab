#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <maplab-server-node/maplab-server-node.h>
#include <maplab-server-node/maplab-server-ros-node.h>

DECLARE_bool(overwrite);
DECLARE_bool(ros_free);

namespace maplab {

class MaplabServerNodeTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    FLAGS_overwrite = true;
    FLAGS_ros_free = true;
  }
  uint32_t getTotalNumMergedSubmaps() const {
    CHECK_NOTNULL(maplab_server_node_);
    return maplab_server_node_->getTotalNumMergedSubmaps();
  }
  MaplabServerNode* maplab_server_node_;

 public:
  const size_t kNumSubmaps = 8u;
  const std::string kBasePath = "./test_maps/submap_test/";
  const std::string kRobotName = "euroc_mh1";
  const std::string kSubmap0 = kBasePath + "euroc_v1_01_submap_0";
  const std::string kSubmap1 = kBasePath + "euroc_v1_01_submap_1";
  const std::string kSubmap2 = kBasePath + "euroc_v1_01_submap_2";
  const std::string kSubmap3 = kBasePath + "euroc_v1_01_submap_3";
  const std::string kSubmap4 = kBasePath + "euroc_v1_01_submap_4";
  const std::string kSubmap5 = kBasePath + "euroc_v1_01_submap_5";
  const std::string kSubmap6 = kBasePath + "euroc_v1_01_submap_6";
  const std::string kSubmap7 = kBasePath + "euroc_v1_01_submap_7";
};

TEST_F(MaplabServerNodeTest, TestMaplabServerNode) {
  MaplabServerNode maplab_server_node;
  maplab_server_node_ = &maplab_server_node;
  maplab_server_node.start();
  ros::Time::init();

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

  while (getTotalNumMergedSubmaps() < kNumSubmaps) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  const std::string merged_map_path = kBasePath + "/merged_map_1";
  EXPECT_TRUE(maplab_server_node.saveMap(merged_map_path));

  maplab_server_node.shutdown();
}

TEST_F(MaplabServerNodeTest, DISABLED_TestMaplabServerRosNodeLocal) {
  MaplabServerRosNode maplab_server_ros_node;

  maplab_server_ros_node.start();

  diagnostic_msgs::KeyValuePtr key_value_msg(new diagnostic_msgs::KeyValue());

  key_value_msg->key = kRobotName;

  key_value_msg->value = kSubmap0;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  key_value_msg->value = kSubmap1;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  key_value_msg->value = kSubmap2;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  key_value_msg->value = kSubmap3;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  key_value_msg->value = kSubmap4;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  key_value_msg->value = kSubmap5;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  key_value_msg->value = kSubmap6;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  key_value_msg->value = kSubmap7;
  maplab_server_ros_node.submapLoadingCallback(key_value_msg);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  maplab_server_ros_node.visualizeMap();

  const std::string merged_map_path = kBasePath + "/merged_map_2";
  EXPECT_TRUE(maplab_server_ros_node.saveMap(merged_map_path));

  maplab_server_ros_node.shutdown();
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
