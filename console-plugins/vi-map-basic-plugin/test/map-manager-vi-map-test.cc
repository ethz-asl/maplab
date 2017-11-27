#include <string>
#include <thread>
#include <unordered_set>

#include <aslam/common/memory.h>
#include <gtest/gtest.h>
#include <map-manager/map-manager.h>
#include <map-manager/test/test-strings.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/vi-map.h>

class MapManagerVIMapTest : public ::testing::Test {
 protected:
  MapManagerVIMapTest() : map_(aligned_unique<vi_map::VIMap>()) {}

  virtual ~MapManagerVIMapTest() {
    // Delete all entries from MapStorage to have a clean state in subsequent
    // tests.
    std::unordered_set<std::string> all_keys;
    map_manager_.getAllMapKeys(&all_keys);
    for (const std::string key : all_keys) {
      map_manager_.deleteMap(key);
    }
    EXPECT_EQ(0u, map_manager_.numberOfMaps());
  }

  static const std::string kBasePath;

  // Also refills map_.
  void addSampleMapToStorage() {
    map_manager_.addMap(TestStrings::kFirstMapKey, map_);
    EXPECT_EQ(map_, nullptr);
    map_.reset(new vi_map::VIMap());
    EXPECT_NE(map_, nullptr);
  }

  void addTwoMapsToStorage() {
    map_manager_.addMap(TestStrings::kFirstMapKey, map_);
    EXPECT_EQ(map_, nullptr);
    map_.reset(new vi_map::VIMap());
    map_manager_.addMap(TestStrings::kSecondMapKey, map_);
    map_.reset(new vi_map::VIMap());
    EXPECT_NE(map_, nullptr);
  }

  vi_map::VIMapManager map_manager_;
  AlignedUniquePtr<vi_map::VIMap> map_;
  AlignedUniquePtr<vi_map::VIMap> empty_map_;
};

const std::string MapManagerVIMapTest::kBasePath = "test_map_save/";

/// Runs two threads: the first thread adds new missions to the same map, while
/// the second thread
/// removes all available missions in parallel. This test checks if a threadsafe
/// access to the map
/// is possible.
TEST_F(MapManagerVIMapTest, ThreadSafeMapAccess) {
  addSampleMapToStorage();
  constexpr size_t kMaxIterationsPerThread = 25u;
  constexpr size_t kThreadSleepTimeMs = 10u;

  std::thread add_map_data_thread([&]() {
    for (size_t i = 0u; i < kMaxIterationsPerThread; ++i) {
      vi_map::VIMapManager::MapWriteAccess map =
          map_manager_.getMapWriteAccess(TestStrings::kFirstMapKey);
      const size_t num_missions_begin = map->numMissions();

      vi_map::MissionId mission_id;
      common::generateId(&mission_id);
      constexpr size_t kNumCameras = 1u;
      map->addNewMissionWithBaseframe(
          mission_id, pose::Transformation(),
          Eigen::Matrix<double, 6, 6>::Identity(),
          aslam::NCamera::createTestNCamera(kNumCameras),
          vi_map::Mission::BackBone::kViwls);
      EXPECT_EQ(num_missions_begin + 1, map->numMissions());

      // Sleep for a while to see if the other thread wants to delete the
      // missions.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(5 * kThreadSleepTimeMs));
      EXPECT_EQ(num_missions_begin + 1, map->numMissions());

      std::this_thread::sleep_for(
          std::chrono::milliseconds(kThreadSleepTimeMs));
    }
  });

  std::thread delete_map_data_thread([&]() {
    for (size_t i = 0u; i < kMaxIterationsPerThread; ++i) {
      vi_map::VIMapManager::MapWriteAccess map =
          map_manager_.getMapWriteAccess(TestStrings::kFirstMapKey);
      const size_t num_missions_begin = map->numMissions();
      vi_map::MissionIdList mission_id_list;
      map->getAllMissionIds(&mission_id_list);
      EXPECT_EQ(num_missions_begin, mission_id_list.size());

      // Delete all found missions.
      for (const vi_map::MissionId& mission_id : mission_id_list) {
        constexpr bool kRemoveBaseframe = false;
        map->removeMission(mission_id, kRemoveBaseframe);
      }

      EXPECT_EQ(0u, map->numMissions());

      // Sleep and check again if other thread didn't create a new mission.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kThreadSleepTimeMs));
      EXPECT_EQ(0u, map->numMissions());
    }
  });

  if (add_map_data_thread.joinable()) {
    add_map_data_thread.join();
  }
  if (delete_map_data_thread.joinable()) {
    delete_map_data_thread.join();
  }
}

TEST_F(MapManagerVIMapTest, CopyMap) {
  addSampleMapToStorage();
  EXPECT_DEATH(
      map_manager_.copyMap(
          TestStrings::kFirstMapKey, TestStrings::kFirstMapKey),
      TestStrings::kFailureMapKeyAlreadyExists);
  EXPECT_DEATH(
      map_manager_.copyMap(
          TestStrings::kSecondMapKey, TestStrings::kFirstMapKey),
      TestStrings::kFailureKeyDoesNotExist);
  map_manager_.copyMap(TestStrings::kFirstMapKey, TestStrings::kSecondMapKey);
}

TEST_F(MapManagerVIMapTest, CopyMapWithData) {
  addSampleMapToStorage();
  map_manager_.getMapMutable(TestStrings::kFirstMapKey)
      ->setMapFolder(kBasePath);
  map_manager_.copyMap(TestStrings::kFirstMapKey, TestStrings::kSecondMapKey);
  std::string second_map_folder;
  map_manager_.getMap(TestStrings::kSecondMapKey)
      .getMapFolder(&second_map_folder);
  EXPECT_EQ(common::getRealPath(kBasePath), second_map_folder);
}

MAPLAB_UNITTEST_ENTRYPOINT
