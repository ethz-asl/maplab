#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/parallel-process.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <transfolder_msgs/RobotSubfoldersArray.h>

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
  transfolder_msgs::RobotSubfoldersArrayPtr subfolders(
      new transfolder_msgs::RobotSubfoldersArray);
  transfolder_msgs::RobotSubfolders subfolder;
  subfolder.robot_name = kRobotName;
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap0);
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap1);
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap2);
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap3);
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap4);
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap5);
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap6);
  subfolder.absolute_subfolder_paths.emplace_back(kSubmap7);
  subfolders->robots_with_subfolders.emplace_back(subfolder);
  maplab_server_ros_node.submapLoadingCallback(subfolders);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  maplab_server_ros_node.visualizeMap();

  const std::string merged_map_path = kBasePath + "/merged_map_2";
  EXPECT_TRUE(maplab_server_ros_node.saveMap(merged_map_path));

  maplab_server_ros_node.shutdown();
}

}  // namespace maplab

MAPLAB_UNITTEST_ENTRYPOINT
