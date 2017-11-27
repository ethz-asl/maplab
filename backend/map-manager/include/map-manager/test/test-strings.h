#ifndef MAP_MANAGER_TEST_STRINGS_H_
#define MAP_MANAGER_TEST_STRINGS_H_

#include <string>

#include "map-manager/map-manager.h"

struct TestStrings {
  // Map Keys.
  static const std::string kFirstMapKey;
  static const std::string kSecondMapKey;
  static const std::string kInvalidMapKey;
  static const std::string kKeyStartingWithPeriod;

  // Death tests.
  static const std::string kMapStorageFileName;
  static const std::string kNullOptions;
  static const std::string kFailureMapCannotBeNull;
  static const std::string kFailureMapKeyAlreadyExists;
  static const std::string kFailureCannotBeEmpty;
  static const std::string kFailureKeyDoesNotExist;
  static const std::string kFailureMapFolderEmpty;
  static const std::string kFailureCannotDeleteFile;
};

const std::string TestStrings::kFirstMapKey = "first_test_map";
const std::string TestStrings::kSecondMapKey = "second_test_map";
const std::string TestStrings::kInvalidMapKey = "This key is invalid.";
const std::string TestStrings::kKeyStartingWithPeriod = ".invalid";

const std::string TestStrings::kMapStorageFileName = "map-storage-inl.h";
const std::string TestStrings::kNullOptions = "(null|NULL|nullptr)";
const std::string TestStrings::kFailureMapCannotBeNull =
    kMapStorageFileName + ".*map.*nullptr.*(M|m)ap can('t|not) be " +
    kNullOptions;
const std::string TestStrings::kFailureMapKeyAlreadyExists =
    ".*key.*already exists";
const std::string TestStrings::kFailureCannotBeEmpty =
    kMapStorageFileName + ".*key(s)? can(not|'t) be empty";
const std::string TestStrings::kFailureKeyDoesNotExist =
    ".*(M|m)ap (with )?key.*does(n't| not) exist";
const std::string TestStrings::kFailureMapFolderEmpty = "map_folder.empty()";
const std::string TestStrings::kFailureCannotDeleteFile =
    "Could not delete file";

#endif  // MAP_MANAGER_TEST_STRINGS_H_
