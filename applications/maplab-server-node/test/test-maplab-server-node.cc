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

 public:
  const std::string kBasePath = "./test_maps/submap_test/";
  const std::string kRobotName = "euroc_mh1";
  const std::string kSubmap0 = kBasePath + "submap_0";
  const std::string kSubmap1 = kBasePath + "submap_1";
  const std::string kSubmap2 = kBasePath + "submap_2";
  const std::string kSubmap3 = kBasePath + "submap_3";
  const std::string kSubmap4 = kBasePath + "submap_4";
  const std::string kSubmap5 = kBasePath + "submap_5";
  const std::string kSubmap6 = kBasePath + "submap_6";
  const uint32_t kNumSubmaps = 7;
};

TEST_F(MaplabServerNodeTest, TestMaplabServerNode) {
  MaplabServerNode maplab_server_node;
  maplab_server_node.start();
  ros::Time::init();

  const uint32_t kSleepBetweenSubmapsSec = 3;

  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap0));
  std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));
  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap1));
  std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));
  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap2));
  std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));
  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap3));
  std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));
  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap4));
  std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));
  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap5));
  std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));
  EXPECT_TRUE(maplab_server_node.loadAndProcessSubmap(kRobotName, kSubmap6));
  std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));

  while (maplab_server_node.getNumMergedSubmaps() < kNumSubmaps) {
    maplab_server_node.visualizeMap();
    std::this_thread::sleep_for(std::chrono::seconds(kSleepBetweenSubmapsSec));
  }

  const std::string merged_map_path = "./merged_map";
  EXPECT_TRUE(maplab_server_node.saveMap(merged_map_path));

  maplab_server_node.shutdown();
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
