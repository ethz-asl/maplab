#include <fstream>  // NOLINT
#include <string>
#include <unordered_set>

#include <aslam/common/memory.h>
#include <gtest/gtest.h>
#include <map-manager/map-manager.h>
#include <map-manager/test/test-strings.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/network-common.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <vi-map/test/vi-map-test-helpers.h>
#include <vi-map/vi-map-serialization.h>
#include <vi-map/vi-map.h>

class MapManagerFileIOTest : public ::testing::Test {
 protected:
  MapManagerFileIOTest()
      : first_vi_map_(aligned_unique<vi_map::VIMap>()),
        second_vi_map_(aligned_unique<vi_map::VIMap>()) {
    dont_overwrite_files.overwrite_existing_files = false;
    force_overwrite_files.overwrite_existing_files = true;
  }

  virtual void SetUp() {
    // Path shouldn't exist yet.
    if (common::pathExists(kPathWithoutFileName)) {
      LOG(WARNING) << "Previous execution of these tests did not shut down "
                      "properly. Deleting \""
                   << kPathWithoutFileName << "\" now.";
      common::removePath(kPathWithoutFileName);
    }
  }

  virtual ~MapManagerFileIOTest() {
    // Clean up file system.
    deleteFolderWithAllContents(kBasePath);

    clearMapStorage();

    EXPECT_FALSE(common::pathExists(kPathWithoutFileName))
        << "The test folder \"" << kPathWithoutFileName
        << "\" wasn't properly deleted at the end of test. Please check that "
           "you removed all "
        << "created files and try again after manually removing the folder.";
  }

  // File paths for save/load.
  static const std::string kBasePath;
  static const std::string kPathWithoutFileName;
  static const std::string kPathWithoutFileNameAlternative;
  static const std::string kPathWithoutFileNameWithoutSlash;
  static const std::string kPathWithFileName;
  static const std::string kInvalidPath;
  static const std::string kPathWithDefaultFileNameFirstEntry;
  static const std::string kPathWithDefaultFileNameFirstEntryAlternative;
  static const std::string kPathWithDefaultFileNameSecondEntry;
  static const std::string kPathToMapFilePathWithFileName;
  static const std::string kPathToMapFileFirstEntry;
  static const std::string kPathToMapFileFirstEntryAlternative;
  static const std::string kPathToMapFileSecondEntry;

  backend::SaveConfig dont_overwrite_files;
  backend::SaveConfig force_overwrite_files;

  // Delete all entries from MapStorage to have a clean state in subsequent
  // tests.
  void clearMapStorage();

  void deleteFolderWithAllContents(const std::string& folder_path) const;

  void addFirstMapToStorage() {
    map_manager_.addMap(TestStrings::kFirstMapKey, first_vi_map_);
    ASSERT_TRUE(map_manager_.hasMap(TestStrings::kFirstMapKey));
  }

  void addSecondMapToStorage() {
    map_manager_.addMap(TestStrings::kSecondMapKey, second_vi_map_);
    ASSERT_TRUE(map_manager_.hasMap(TestStrings::kSecondMapKey));
  }

  void saveFirstMapToFileSystem() {
    addFirstMapToStorage();
    EXPECT_TRUE(
        map_manager_.saveMapToFolder(
            TestStrings::kFirstMapKey, kPathWithDefaultFileNameFirstEntry));
    EXPECT_TRUE(common::pathExists(kPathToMapFileFirstEntry));
    EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  }

  void saveSecondMapToFileSystem() {
    addSecondMapToStorage();
    EXPECT_TRUE(
        map_manager_.saveMapToFolder(
            TestStrings::kSecondMapKey, kPathWithDefaultFileNameSecondEntry));
    EXPECT_TRUE(common::pathExists(kPathToMapFileSecondEntry));
    EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameSecondEntry));
  }

  void saveMapsToFileSystem() {
    saveMapsToFileSystemWithoutClearingStorage();
    // Clear map storage.
    clearMapStorage();
  }

  void saveMapsToFileSystemWithoutClearingStorage() {
    saveFirstMapToFileSystem();
    saveSecondMapToFileSystem();
  }

  vi_map::VIMapManager map_manager_;

  vi_map::VIMap::UniquePtr first_vi_map_;
  vi_map::VIMap::UniquePtr second_vi_map_;
};

const std::string MapManagerFileIOTest::kBasePath = "test_map_save/";
const std::string MapManagerFileIOTest::kPathWithoutFileName =
    MapManagerFileIOTest::kBasePath + "folder/";
const std::string MapManagerFileIOTest::kPathWithoutFileNameAlternative =
    MapManagerFileIOTest::kBasePath + "folder/alternative/";
const std::string MapManagerFileIOTest::kPathWithoutFileNameWithoutSlash =
    MapManagerFileIOTest::kBasePath + "folder";
const std::string MapManagerFileIOTest::kPathWithFileName =
    MapManagerFileIOTest::kBasePath + "folder/filename";
const std::string MapManagerFileIOTest::kInvalidPath = "/a/b/c/d/e";

const std::string MapManagerFileIOTest::kPathWithDefaultFileNameFirstEntry =
    MapManagerFileIOTest::kPathWithoutFileName + TestStrings::kFirstMapKey;
const std::string
    MapManagerFileIOTest::kPathWithDefaultFileNameFirstEntryAlternative =
        MapManagerFileIOTest::kPathWithoutFileNameAlternative +
        TestStrings::kFirstMapKey;
const std::string MapManagerFileIOTest::kPathWithDefaultFileNameSecondEntry =
    MapManagerFileIOTest::kPathWithoutFileName + TestStrings::kSecondMapKey;

const std::string MapManagerFileIOTest::kPathToMapFilePathWithFileName =
    MapManagerFileIOTest::kPathWithFileName + "/" +
    vi_map::serialization::internal::kFolderName;
const std::string MapManagerFileIOTest::kPathToMapFileFirstEntry =
    MapManagerFileIOTest::kPathWithDefaultFileNameFirstEntry + "/" +
    vi_map::serialization::internal::kFolderName;
const std::string MapManagerFileIOTest::kPathToMapFileFirstEntryAlternative =
    MapManagerFileIOTest::kPathWithDefaultFileNameFirstEntryAlternative + "/" +
    vi_map::serialization::internal::kFolderName;
const std::string MapManagerFileIOTest::kPathToMapFileSecondEntry =
    MapManagerFileIOTest::kPathWithDefaultFileNameSecondEntry + "/" +
    vi_map::serialization::internal::kFolderName;

void MapManagerFileIOTest::clearMapStorage() {
  std::unordered_set<std::string> all_keys;
  map_manager_.getAllMapKeys(&all_keys);
  for (const std::string key : all_keys) {
    map_manager_.deleteMap(key);
  }
  EXPECT_EQ(0u, map_manager_.numberOfMaps());
}

void MapManagerFileIOTest::deleteFolderWithAllContents(
    const std::string& folder_path) const {
  if (common::pathExists(folder_path)) {
    std::vector<std::string> all_files, all_folders;
    common::getAllFilesAndFoldersInFolder(
        folder_path, &all_files, &all_folders);
    for (const std::string& file : all_files) {
      common::deleteFile(file);
    }
    // Order folder list so that innermost folders can be deleted last.
    std::sort(all_folders.begin(), all_folders.end());
    for (std::vector<std::string>::const_reverse_iterator it =
             all_folders.crbegin();
         it != all_folders.crend(); ++it) {
      common::deleteFile(*it);
    }
    common::deleteFile(folder_path);
  }
}

TEST_F(MapManagerFileIOTest, SaveMapNonExistentKey) {
  addFirstMapToStorage();

  // Key doesn't exist.
  EXPECT_FALSE(
      map_manager_.saveMapToFolder(
          TestStrings::kSecondMapKey, kPathWithFileName, dont_overwrite_files));
  EXPECT_FALSE(
      map_manager_.saveMapToFolder(
          TestStrings::kSecondMapKey, kPathWithoutFileName));
  EXPECT_FALSE(common::fileExists(kPathToMapFilePathWithFileName));
  EXPECT_FALSE(common::pathExists(kPathWithFileName));
  EXPECT_FALSE(common::fileExists(kPathToMapFileSecondEntry));
  EXPECT_FALSE(common::pathExists(kPathWithDefaultFileNameSecondEntry));
}

// Save file in folder.
TEST_F(MapManagerFileIOTest, SaveMapToFolder) {
  addFirstMapToStorage();
  EXPECT_TRUE(
      map_manager_.saveMapToFolder(
          TestStrings::kFirstMapKey, kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(common::pathExists(kPathWithoutFileName));
  EXPECT_TRUE(
      map_manager_.hasMapOnFileSystem(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));

  // Save file in folder, however file already exists.
  EXPECT_FALSE(
      map_manager_.saveMapToFolder(
          TestStrings::kFirstMapKey, kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(
      map_manager_.saveMapToFolder(
          TestStrings::kFirstMapKey, kPathWithDefaultFileNameFirstEntry,
          force_overwrite_files));

  // Add second file.
  addSecondMapToStorage();
  EXPECT_TRUE(
      map_manager_.saveMapToFolder(
          TestStrings::kSecondMapKey, kPathWithDefaultFileNameSecondEntry));
  EXPECT_TRUE(
      map_manager_.hasMapOnFileSystem(kPathWithDefaultFileNameSecondEntry));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameSecondEntry));

  // Overwrite first map.
  EXPECT_TRUE(
      map_manager_.saveMapToFolder(
          TestStrings::kSecondMapKey, kPathWithDefaultFileNameFirstEntry,
          force_overwrite_files));
}

TEST_F(MapManagerFileIOTest, SaveMapToMapFolder) {
  addFirstMapToStorage();
  EXPECT_DEATH(
      map_manager_.saveMapToMapFolder(TestStrings::kFirstMapKey),
      TestStrings::kFailureMapFolderEmpty);

  map_manager_.setMapFolder(
      TestStrings::kFirstMapKey, kPathWithDefaultFileNameFirstEntryAlternative);

  EXPECT_TRUE(map_manager_.saveMapToMapFolder(TestStrings::kFirstMapKey));
  EXPECT_TRUE(
      common::pathExists(kPathWithDefaultFileNameFirstEntryAlternative));
  EXPECT_TRUE(common::pathExists(kPathToMapFileFirstEntryAlternative));
  EXPECT_FALSE(map_manager_.saveMapToMapFolder(TestStrings::kFirstMapKey));
  EXPECT_TRUE(
      map_manager_.saveMapToMapFolder(
          TestStrings::kFirstMapKey, force_overwrite_files));
}

// Can't write to path.
TEST_F(MapManagerFileIOTest, SaveMapToFileInvalidPath) {
  addFirstMapToStorage();
  EXPECT_FALSE(
      map_manager_.saveMapToFolder(
          TestStrings::kFirstMapKey, kInvalidPath, dont_overwrite_files));
  EXPECT_FALSE(
      map_manager_.saveMapToFolder(
          TestStrings::kFirstMapKey, kInvalidPath, force_overwrite_files));
}

TEST_F(MapManagerFileIOTest, SaveMapToFolderInvalidPath) {
  addFirstMapToStorage();
  EXPECT_FALSE(
      map_manager_.saveMapToFolder(TestStrings::kFirstMapKey, kInvalidPath));
  EXPECT_FALSE(
      map_manager_.saveMapToFolder(
          TestStrings::kFirstMapKey, kInvalidPath, force_overwrite_files));
}

TEST_F(MapManagerFileIOTest, SaveAllMaps) {
  // Add a few maps.
  addFirstMapToStorage();
  addSecondMapToStorage();

  // Save all maps in clean path.
  EXPECT_TRUE(map_manager_.saveAllMapsToFolder(kPathWithoutFileName));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameSecondEntry));

  // Save all maps, with map already existing in path.
  EXPECT_FALSE(map_manager_.saveAllMapsToFolder(kPathWithoutFileName));
  EXPECT_TRUE(
      map_manager_.saveAllMapsToFolder(
          kPathWithoutFileName, force_overwrite_files));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameSecondEntry));

  deleteFolderWithAllContents(kPathWithDefaultFileNameFirstEntry);
  EXPECT_FALSE(map_manager_.saveAllMapsToFolder(kPathWithoutFileName));
  // Second map still exists (wasn't deleted), first map didn't get saved.
  EXPECT_FALSE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameSecondEntry));

  deleteFolderWithAllContents(kPathToMapFileSecondEntry);
  deleteFolderWithAllContents(kPathWithDefaultFileNameSecondEntry);
  EXPECT_FALSE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  EXPECT_FALSE(common::pathExists(kPathWithDefaultFileNameSecondEntry));

  // Save maps, folder already exists.
  EXPECT_TRUE(
      map_manager_.saveAllMapsToFolder(kPathWithoutFileNameWithoutSlash));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameSecondEntry));
}

TEST_F(MapManagerFileIOTest, SaveAllMapsToMapFolder) {
  addFirstMapToStorage();
  addSecondMapToStorage();
  // This tests saving all maps to their map folders, which need to be set
  // beforehand.

  // Doesn't work because the first map does not have a map folder.
  EXPECT_FALSE(map_manager_.saveAllMapsToMapFolder());
  map_manager_.setMapFolder(
      TestStrings::kFirstMapKey, kPathWithDefaultFileNameFirstEntry);

  // Doesn't work because the second map does not have a map folder.
  EXPECT_FALSE(map_manager_.saveAllMapsToMapFolder());
  map_manager_.setMapFolder(
      TestStrings::kSecondMapKey, kPathWithDefaultFileNameSecondEntry);

  // Now it works, because all map folders are set.
  EXPECT_TRUE(map_manager_.saveAllMapsToMapFolder());

  // Check if map folders now exist.
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(common::pathExists(kPathWithDefaultFileNameSecondEntry));

  // Doesn't work because we should not overwrite map folders by default.
  EXPECT_FALSE(map_manager_.saveAllMapsToMapFolder());

  // Now it should work, because we force it to overwrite.
  EXPECT_TRUE(map_manager_.saveAllMapsToMapFolder(force_overwrite_files));
}

TEST_F(MapManagerFileIOTest, SaveAllWithEmptyStorage) {
  EXPECT_FALSE(map_manager_.saveAllMapsToFolder(kPathWithoutFileName));
}

// Save maps in invalid location (e.g. no write access).
TEST_F(MapManagerFileIOTest, SaveAllMapsInvalidPath) {
  addFirstMapToStorage();
  addSecondMapToStorage();
  EXPECT_FALSE(map_manager_.saveAllMapsToFolder(kInvalidPath));
}

TEST_F(MapManagerFileIOTest, LoadMapInvalidPath) {
  saveMapsToFileSystem();

  // Invalid paths and paths to non-existing files won't work.
  EXPECT_FALSE(map_manager_.loadMapFromFolder(kPathWithFileName));
  EXPECT_FALSE(map_manager_.loadMapFromFolder(kInvalidPath));
}

// Load individual map, automatic key.
TEST_F(MapManagerFileIOTest, LoadMapAutomaticKey) {
  saveMapsToFileSystem();
  EXPECT_TRUE(
      map_manager_.loadMapFromFolder(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(map_manager_.hasMap(TestStrings::kFirstMapKey));

  // Load individual map, but key already defined in map storage.
  EXPECT_FALSE(
      map_manager_.loadMapFromFolder(kPathWithDefaultFileNameFirstEntry));
}

// Load individual map, custom key.
TEST_F(MapManagerFileIOTest, LoadMapCustomKey) {
  saveMapsToFileSystem();
  EXPECT_TRUE(
      map_manager_.loadMapFromFolder(
          kPathWithDefaultFileNameFirstEntry, TestStrings::kSecondMapKey));
  map_manager_.getMap(TestStrings::kSecondMapKey);
}

// Load folder, clean map storage.
TEST_F(MapManagerFileIOTest, LoadAllMapsCleanStorage) {
  saveMapsToFileSystem();

  EXPECT_TRUE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
  map_manager_.getMap(TestStrings::kFirstMapKey);
  map_manager_.getMap(TestStrings::kSecondMapKey);
}

// Load folder without trailing slash.
TEST_F(MapManagerFileIOTest, LoadAllMapsWithoutTrailingSlash) {
  saveMapsToFileSystem();
  EXPECT_TRUE(
      map_manager_.loadAllMapsFromFolder(kPathWithoutFileNameWithoutSlash));
  map_manager_.getMap(TestStrings::kFirstMapKey);
  map_manager_.getMap(TestStrings::kSecondMapKey);
}

TEST_F(MapManagerFileIOTest, LoadAllWithEmptyFileSystem) {
  EXPECT_FALSE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
  common::createPath(kPathWithoutFileName);
  EXPECT_FALSE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
}

// Load folder, with key existing in map storage.
TEST_F(MapManagerFileIOTest, LoadAllMapsWithExistingMapInStorage) {
  saveMapsToFileSystem();
  EXPECT_TRUE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
  EXPECT_FALSE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));

  // Load folder with existing key shouldn't load any maps.
  map_manager_.deleteMap(TestStrings::kFirstMapKey);
  EXPECT_FALSE(map_manager_.hasMap(TestStrings::kFirstMapKey));
  EXPECT_FALSE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
  EXPECT_FALSE(map_manager_.hasMap(TestStrings::kFirstMapKey));
}

TEST_F(MapManagerFileIOTest, LoadAllMapsWithNonMapFiles) {
  saveFirstMapToFileSystem();
  clearMapStorage();

  // Create some files and folders that should be ignored.
  common::createPath(kPathWithDefaultFileNameSecondEntry);
  const std::string kNotLoadedFile =
      kPathWithoutFileName + "/" + vi_map::serialization::getSubFolderName();
  {
    std::ofstream o(kNotLoadedFile);
    ASSERT_TRUE(o.is_open());
  }

  EXPECT_TRUE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
  EXPECT_EQ(1u, map_manager_.numberOfMaps());
  EXPECT_TRUE(map_manager_.hasMap(TestStrings::kFirstMapKey));
  EXPECT_FALSE(map_manager_.hasMap(TestStrings::kSecondMapKey));

  deleteFolderWithAllContents(kNotLoadedFile);
}

TEST_F(MapManagerFileIOTest, LoadAllMapsWithCollidingKeys) {
  saveFirstMapToFileSystem();
  map_manager_.saveMapToFolder(
      TestStrings::kFirstMapKey, kPathWithDefaultFileNameFirstEntryAlternative);
  ASSERT_TRUE(common::pathExists(kPathWithoutFileNameAlternative));
  ASSERT_TRUE(
      common::pathExists(kPathWithDefaultFileNameFirstEntryAlternative));
  ASSERT_TRUE(common::pathExists(kPathToMapFileFirstEntryAlternative));
  clearMapStorage();

  EXPECT_FALSE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
}

TEST_F(MapManagerFileIOTest, CheckSplitVIMapSerialization) {
  vi_map::test::generateMap(first_vi_map_.get());
  vi_map::proto::VIMap vertices_proto, edges_proto, missions_proto,
      landmark_index_proto, optional_sensor_data_proto;
  network::RawMessageData sensor_manager_raw_data;
  vi_map::serialization::serializeSensorManagerToArray(
      *first_vi_map_, &sensor_manager_raw_data);
  vi_map::serialization::serializeMissionsAndBaseframes(
      *first_vi_map_, &missions_proto);
  vi_map::serialization::serializeVertices(*first_vi_map_, &vertices_proto);
  vi_map::serialization::serializeEdges(*first_vi_map_, &edges_proto);
  vi_map::serialization::serializeLandmarkIndex(
      *first_vi_map_, &landmark_index_proto);
  vi_map::serialization::serializeOptionalSensorData(
      *first_vi_map_, &optional_sensor_data_proto);

  vi_map::serialization::deserializeSensorManagerFromArray(
      sensor_manager_raw_data, second_vi_map_.get());
  vi_map::serialization::deserializeMissionsAndBaseframes(
      missions_proto, second_vi_map_.get());
  vi_map::serialization::deserializeVertices(
      vertices_proto, second_vi_map_.get());
  vi_map::serialization::deserializeEdges(edges_proto, second_vi_map_.get());
  vi_map::serialization::deserializeLandmarkIndex(
      landmark_index_proto, second_vi_map_.get());
  vi_map::serialization::deserializeOptionalSensorData(
      optional_sensor_data_proto, second_vi_map_.get());
  EXPECT_TRUE(vi_map::test::compareVIMap(*first_vi_map_, *second_vi_map_));
}

TEST_F(MapManagerFileIOTest, CheckSplitVIMapSerializationManyVertices) {
  constexpr size_t kNumberOfVertices = 400u;
  vi_map::test::generateMap(kNumberOfVertices, first_vi_map_.get());
  vi_map::proto::VIMap vertices_proto, edges_proto, missions_proto,
      landmark_index_proto, optional_sensor_data_proto;

  network::RawMessageData sensor_manager_raw_data;
  vi_map::serialization::serializeSensorManagerToArray(
      *first_vi_map_, &sensor_manager_raw_data);
  vi_map::serialization::serializeMissionsAndBaseframes(
      *first_vi_map_, &missions_proto);
  vi_map::serialization::serializeVertices(*first_vi_map_, &vertices_proto);
  vi_map::serialization::serializeEdges(*first_vi_map_, &edges_proto);
  vi_map::serialization::serializeLandmarkIndex(
      *first_vi_map_, &landmark_index_proto);
  vi_map::serialization::serializeOptionalSensorData(
      *first_vi_map_, &optional_sensor_data_proto);

  vi_map::serialization::deserializeSensorManagerFromArray(
      sensor_manager_raw_data, second_vi_map_.get());
  vi_map::serialization::deserializeMissionsAndBaseframes(
      missions_proto, second_vi_map_.get());
  vi_map::serialization::deserializeVertices(
      vertices_proto, second_vi_map_.get());
  vi_map::serialization::deserializeEdges(edges_proto, second_vi_map_.get());
  vi_map::serialization::deserializeLandmarkIndex(
      landmark_index_proto, second_vi_map_.get());
  vi_map::serialization::deserializeOptionalSensorData(
      optional_sensor_data_proto, second_vi_map_.get());
  EXPECT_TRUE(vi_map::test::compareVIMap(*first_vi_map_, *second_vi_map_));
}

// Save and load map with data.
TEST_F(MapManagerFileIOTest, SaveLoadMapWithActualData) {
  vi_map::test::generateMap(first_vi_map_.get());
  saveMapsToFileSystemWithoutClearingStorage();
  map_manager_.deleteMap(TestStrings::kSecondMapKey);

  ASSERT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  EXPECT_TRUE(
      map_manager_.loadMapFromFolder(
          kPathWithDefaultFileNameFirstEntry, TestStrings::kSecondMapKey));
  ASSERT_TRUE(map_manager_.hasMap(TestStrings::kSecondMapKey));

  const vi_map::VIMap& original_map =
      map_manager_.getMap(TestStrings::kFirstMapKey);
  const vi_map::VIMap& loaded_map =
      map_manager_.getMap(TestStrings::kSecondMapKey);

  // Compare original and loaded map.
  EXPECT_TRUE(vi_map::test::compareVIMap(original_map, loaded_map));
}

// Save and load map with many vertices to test vertices splitting.
TEST_F(MapManagerFileIOTest, SaveLoadMapWithManyVertices) {
  constexpr size_t kNumberOfVertices = 400u;
  vi_map::test::generateMap(kNumberOfVertices, first_vi_map_.get());
  saveMapsToFileSystemWithoutClearingStorage();
  map_manager_.deleteMap(TestStrings::kSecondMapKey);

  ASSERT_TRUE(common::pathExists(kPathWithDefaultFileNameFirstEntry));
  map_manager_.loadMapFromFolder(
      kPathWithDefaultFileNameFirstEntry, TestStrings::kSecondMapKey);
  ASSERT_TRUE(map_manager_.hasMap(TestStrings::kSecondMapKey));

  const vi_map::VIMap& original_map =
      map_manager_.getMap(TestStrings::kFirstMapKey);
  const vi_map::VIMap& loaded_map =
      map_manager_.getMap(TestStrings::kSecondMapKey);

  // Compare original and loaded map.
  EXPECT_TRUE(vi_map::test::compareVIMap(original_map, loaded_map));
}

// Loads a map that contains prohibited characters in its filename and would
// therefore generate an
// invalid key.
TEST_F(MapManagerFileIOTest, LoadMapWithFilenameWithSpaces) {
  for (const std::string& key :
       {TestStrings::kInvalidMapKey, TestStrings::kKeyStartingWithPeriod}) {
    const std::string file_path = kPathWithoutFileName + key;
    first_vi_map_.reset(new vi_map::VIMap());
    addFirstMapToStorage();
    ASSERT_TRUE(
        map_manager_.saveMapToFolder(
            TestStrings::kFirstMapKey, file_path, dont_overwrite_files));
    ASSERT_TRUE(common::pathExists(file_path));

    ASSERT_TRUE(map_manager_.loadMapFromFolder(file_path));
    EXPECT_FALSE(map_manager_.hasMap(key));

    const std::string legalized_invalid_key =
        map_manager_.removeProhibitedCharactersFromKey(key);
    EXPECT_TRUE(map_manager_.hasMap(legalized_invalid_key));
    clearMapStorage();

    EXPECT_TRUE(map_manager_.loadAllMapsFromFolder(kPathWithoutFileName));
    clearMapStorage();
  }
}

TEST_F(MapManagerFileIOTest, ListMapsOnFileSystem) {
  std::vector<std::string> map_list;
  map_manager_.listAllMapsInFolder(kPathWithoutFileName, &map_list);
  EXPECT_TRUE(map_list.empty());

  saveMapsToFileSystem();

  map_manager_.listAllMapsInFolder(kPathWithoutFileName, &map_list);
  EXPECT_EQ(map_list.size(), 2u);

  deleteFolderWithAllContents(kBasePath);

  map_manager_.listAllMapsInFolder(kPathWithoutFileName, &map_list);
  EXPECT_TRUE(map_list.empty());
}

TEST_F(MapManagerFileIOTest, GetDefaultMapKeys) {
  saveMapsToFileSystem();
  std::vector<std::string> map_list;
  map_manager_.listAllMapsInFolder(kPathWithoutFileName, &map_list);
  EXPECT_EQ(map_list.size(), 2u);

  std::vector<std::string> map_keys;
  map_manager_.getDefaultMapKeys(map_list, &map_keys);
  EXPECT_EQ(map_keys.size(), 2u);

  std::unordered_set<std::string> map_keys_set(map_keys.begin(),
                                               map_keys.end());
  EXPECT_EQ(map_keys_set.size(), 2u);
  EXPECT_EQ(map_keys_set.count(TestStrings::kFirstMapKey), 1u);
  EXPECT_EQ(map_keys_set.count(TestStrings::kSecondMapKey), 1u);
}

MAPLAB_UNITTEST_ENTRYPOINT
