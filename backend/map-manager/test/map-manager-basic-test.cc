#include <chrono>
#include <string>
#include <thread>
#include <unordered_set>

#include <aslam/common/memory.h>
#include <gtest/gtest.h>
#include <maplab-common/map-traits.h>
#include <maplab-common/test/testing-entrypoint.h>

#include "map-manager/map-manager.h"
#include "map-manager/test/test-map-type.h"
#include "map-manager/test/test-strings.h"

class MapManagerBasicTest : public ::testing::Test {
 protected:
  MapManagerBasicTest() : map_(aligned_unique<backend::TestMapType>()) {}

  virtual ~MapManagerBasicTest() {
    // Delete all entries from MapStorage to have a clean state in subsequent
    // tests.
    std::unordered_set<std::string> all_keys;
    map_manager_.getAllMapKeys(&all_keys);
    for (const std::string key : all_keys) {
      map_manager_.deleteMap(key);
    }
    EXPECT_EQ(0u, map_manager_.numberOfMaps());
  }

  // Also refills map_.
  void addSampleMapToStorage() {
    map_manager_.addMap(TestStrings::kFirstMapKey, map_);
    EXPECT_EQ(map_, nullptr);
    map_.reset(new backend::TestMapType());
    EXPECT_NE(map_, nullptr);
  }

  void addTwoMapsToStorage() {
    map_manager_.addMap(TestStrings::kFirstMapKey, map_);
    EXPECT_EQ(map_, nullptr);
    map_.reset(new backend::TestMapType());
    map_manager_.addMap(TestStrings::kSecondMapKey, map_);
    map_.reset(new backend::TestMapType());
    EXPECT_NE(map_, nullptr);
  }

  backend::MapManager<backend::TestMapType> map_manager_;
  AlignedUniquePtr<backend::TestMapType> map_;
  AlignedUniquePtr<backend::TestMapType> empty_map_;
};

// Add a map with a non-empty key (normal case).
TEST_F(MapManagerBasicTest, AddMapBasic) {
  addSampleMapToStorage();
  EXPECT_EQ(1u, map_manager_.numberOfMaps());

  // Check if all maps are in the storage.
  std::unordered_set<std::string> all_keys;
  map_manager_.getAllMapKeys(&all_keys);
  EXPECT_EQ(1u, all_keys.size());
  EXPECT_EQ(1u, all_keys.count(TestStrings::kFirstMapKey));
  EXPECT_EQ(0u, all_keys.count(TestStrings::kSecondMapKey));
}

// Adding nullptr map should fail.
TEST_F(MapManagerBasicTest, AddMapNullptr) {
  EXPECT_DEATH(
      map_manager_.addMap(TestStrings::kSecondMapKey, empty_map_),
      TestStrings::kFailureMapCannotBeNull);
}

// Trying to add a map with an already existing key.
TEST_F(MapManagerBasicTest, AddMapAlreadyExistingKey) {
  addSampleMapToStorage();
  EXPECT_DEATH(
      map_manager_.addMap(TestStrings::kFirstMapKey, map_),
      TestStrings::kFailureMapKeyAlreadyExists);
  EXPECT_EQ(1u, map_manager_.numberOfMaps());
}

// Add a second map.
TEST_F(MapManagerBasicTest, AddMapSecondMap) {
  addSampleMapToStorage();
  map_manager_.addMap(TestStrings::kSecondMapKey, map_);
  EXPECT_EQ(2u, map_manager_.numberOfMaps());

  // Check if all maps are in the storage.
  std::unordered_set<std::string> all_keys;
  map_manager_.getAllMapKeys(&all_keys);
  EXPECT_EQ(2u, all_keys.size());
  EXPECT_EQ(1u, all_keys.count(TestStrings::kFirstMapKey));
  EXPECT_EQ(1u, all_keys.count(TestStrings::kSecondMapKey));
}

// Add a map with an empty key.
TEST_F(MapManagerBasicTest, AddMapEmptyKey) {
  EXPECT_DEATH(
      map_manager_.addMap("", map_), TestStrings::kFailureCannotBeEmpty);
}

// Add a map with an invalid key.
TEST_F(MapManagerBasicTest, AddMapInvalidKey) {
  EXPECT_DEATH(
      map_manager_.addMap(TestStrings::kInvalidMapKey, map_),
      TestStrings::kMapStorageFileName);
}

TEST_F(MapManagerBasicTest, GetMapBasic) {
  addSampleMapToStorage();
  ASSERT_EQ(1u, map_manager_.numberOfMaps());

  // Get map.
  map_manager_.getMap(TestStrings::kFirstMapKey);
}

// Get map with non-existent key.
TEST_F(MapManagerBasicTest, GetMapNonExistentKey) {
  EXPECT_DEATH(
      map_manager_.getMap(TestStrings::kSecondMapKey),
      TestStrings::kFailureKeyDoesNotExist);
  addSampleMapToStorage();
  EXPECT_DEATH(
      map_manager_.getMap(TestStrings::kSecondMapKey),
      TestStrings::kFailureKeyDoesNotExist);
}

// Get map with empty key.
TEST_F(MapManagerBasicTest, GetMapEmptyKey) {
  EXPECT_DEATH(map_manager_.getMap(""), TestStrings::kFailureKeyDoesNotExist);
}

// Add a second map.
TEST_F(MapManagerBasicTest, GetMapSecondMap) {
  addTwoMapsToStorage();
  ASSERT_EQ(2u, map_manager_.numberOfMaps());

  // Get two existing maps.
  map_manager_.getMap(TestStrings::kFirstMapKey);
  map_manager_.getMap(TestStrings::kSecondMapKey);
}

// Delete existing map.
TEST_F(MapManagerBasicTest, DeleteMapBasic) {
  addSampleMapToStorage();
  ASSERT_EQ(1u, map_manager_.numberOfMaps());
  map_manager_.deleteMap(TestStrings::kFirstMapKey);
  ASSERT_EQ(0u, map_manager_.numberOfMaps());
  EXPECT_FALSE(map_manager_.hasMap(TestStrings::kFirstMapKey));

  // Readd deleted map.
  map_manager_.addMap(TestStrings::kFirstMapKey, map_);
  EXPECT_EQ(1u, map_manager_.numberOfMaps());
}

TEST_F(MapManagerBasicTest, DeleteMapEmptyStorage) {
  // Delete map on empty storage.
  EXPECT_DEATH(
      map_manager_.deleteMap(TestStrings::kFirstMapKey),
      TestStrings::kFailureKeyDoesNotExist);
  EXPECT_DEATH(
      map_manager_.deleteMap(""), TestStrings::kFailureKeyDoesNotExist);
}

// Delete map with empty key.
TEST_F(MapManagerBasicTest, DeleteMapInvalidKey) {
  EXPECT_FALSE(map_manager_.hasMap(""));

  // Delete non-existent map, storage not empty.
  EXPECT_DEATH(
      map_manager_.deleteMap(TestStrings::kSecondMapKey),
      TestStrings::kFailureKeyDoesNotExist);
  EXPECT_DEATH(
      map_manager_.deleteMap(""), TestStrings::kFailureKeyDoesNotExist);

  addSampleMapToStorage();
  EXPECT_DEATH(
      map_manager_.deleteMap(TestStrings::kSecondMapKey),
      TestStrings::kFailureKeyDoesNotExist);
  EXPECT_DEATH(
      map_manager_.deleteMap(""), TestStrings::kFailureKeyDoesNotExist);
}

// Valid rename.
TEST_F(MapManagerBasicTest, RenameMapBasic) {
  addSampleMapToStorage();
  ASSERT_EQ(1u, map_manager_.numberOfMaps());

  map_manager_.renameMap(TestStrings::kFirstMapKey, TestStrings::kSecondMapKey);
  ASSERT_EQ(1u, map_manager_.numberOfMaps());

  // Check if rename was successful.
  EXPECT_FALSE(map_manager_.hasMap(TestStrings::kFirstMapKey));
  EXPECT_TRUE(map_manager_.hasMap(TestStrings::kSecondMapKey));

  // Move back to original location.
  map_manager_.renameMap(TestStrings::kSecondMapKey, TestStrings::kFirstMapKey);
  ASSERT_EQ(1u, map_manager_.numberOfMaps());

  // Check if rename was successful.
  EXPECT_TRUE(map_manager_.hasMap(TestStrings::kFirstMapKey));
  EXPECT_FALSE(map_manager_.hasMap(TestStrings::kSecondMapKey));
}

// Empty map storage.
TEST_F(MapManagerBasicTest, RenameMapEmptyStorage) {
  EXPECT_DEATH(
      map_manager_.renameMap(
          TestStrings::kFirstMapKey, TestStrings::kFirstMapKey),
      TestStrings::kFailureKeyDoesNotExist);
  EXPECT_DEATH(
      map_manager_.renameMap(
          TestStrings::kFirstMapKey, TestStrings::kSecondMapKey),
      TestStrings::kFailureKeyDoesNotExist);
}

// Empty keys.
TEST_F(MapManagerBasicTest, RenameMapEmptyKeys) {
  addSampleMapToStorage();
  EXPECT_DEATH(
      map_manager_.renameMap("", ""), TestStrings::kFailureCannotBeEmpty);
  EXPECT_DEATH(
      map_manager_.renameMap("", TestStrings::kFirstMapKey),
      TestStrings::kFailureCannotBeEmpty);
  EXPECT_DEATH(
      map_manager_.renameMap(TestStrings::kFirstMapKey, ""),
      TestStrings::kFailureCannotBeEmpty);
}

// Rename to existing key.
TEST_F(MapManagerBasicTest, RenameMapToExisting) {
  addSampleMapToStorage();
  EXPECT_DEATH(
      map_manager_.renameMap(
          TestStrings::kFirstMapKey, TestStrings::kFirstMapKey),
      TestStrings::kFailureMapKeyAlreadyExists);
  ASSERT_EQ(1u, map_manager_.numberOfMaps());
}

TEST_F(MapManagerBasicTest, FixInvalidKey) {
  for (const std::string& key :
       {TestStrings::kInvalidMapKey, TestStrings::kKeyStartingWithPeriod}) {
    EXPECT_FALSE(map_manager_.isKeyValid(key));
    const std::string legalized_invalid_key =
        map_manager_.removeProhibitedCharactersFromKey(key);
    EXPECT_TRUE(map_manager_.isKeyValid(legalized_invalid_key));
  }
}

TEST_F(MapManagerBasicTest, ReleaseMap) {
  addSampleMapToStorage();
  AlignedUniquePtr<backend::TestMapType> released_map =
      map_manager_.releaseMap(TestStrings::kFirstMapKey);
  EXPECT_NE(released_map, nullptr);
  EXPECT_FALSE(map_manager_.hasMap(TestStrings::kFirstMapKey));
}

TEST_F(MapManagerBasicTest, ReleaseMapWhileInUse) {
  addSampleMapToStorage();
  constexpr size_t kNumberOfThreads = 10u;
  std::vector<std::thread> worker_threads;

  std::mutex connected_counter_mutex;
  size_t num_connected_threads = 0u;

  for (size_t i = 0u; i < kNumberOfThreads; ++i) {
    worker_threads.emplace_back([&]() {
      constexpr size_t kSleepTimeMs = 50u;
      {
        std::unique_lock<std::mutex> lock(connected_counter_mutex);
        ++num_connected_threads;
      }
      backend::MapManager<backend::TestMapType>::MapWriteAccess map =
          map_manager_.getMapWriteAccess(TestStrings::kFirstMapKey);
      const size_t counter_begin = map->getCounter();
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
      map->incrementCounter();
      EXPECT_EQ(counter_begin + 1, map->getCounter());
    });
  }

  for (size_t i = 0u; i < kNumberOfThreads; ++i) {
    worker_threads.emplace_back([&]() {
      constexpr size_t kSleepTimeMs = 50u;
      {
        std::unique_lock<std::mutex> lock(connected_counter_mutex);
        ++num_connected_threads;
      }
      backend::MapManager<backend::TestMapType>::MapReadAccess map =
          map_manager_.getMapReadAccess(TestStrings::kFirstMapKey);
      const size_t counter_begin = map->getCounter();
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
      EXPECT_EQ(counter_begin, map->getCounter());
    });
  }
  // Try to release map.
  constexpr size_t kSleepTimeMs = 5u;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    std::unique_lock<std::mutex> lock(connected_counter_mutex);
    if (num_connected_threads == 2u * kNumberOfThreads) {
      break;
    }
  }
  AlignedUniquePtr<backend::TestMapType> released_map =
      map_manager_.releaseMap(TestStrings::kFirstMapKey);
  EXPECT_EQ(released_map->getCounter(), kNumberOfThreads);

  for (std::thread& thread : worker_threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

MAPLAB_UNITTEST_ENTRYPOINT
