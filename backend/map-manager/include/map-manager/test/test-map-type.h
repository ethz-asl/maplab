#ifndef MAP_MANAGER_TEST_TEST_MAP_TYPE_H_
#define MAP_MANAGER_TEST_TEST_MAP_TYPE_H_

#include <string>

#include <maplab-common/map-traits.h>

namespace backend {

// Test class to unit test basic map manager functionality.
class TestMapType : public MapInterface<TestMapType> {
 public:
  TestMapType() : counter_(0u) {}

  bool hasMapFolder() {
    return false;
  }
  void getMapFolder(std::string* /*map_folder*/) {}
  void setMapFolder(const std::string& /*map_folder*/) {}

  virtual void deepCopy(const TestMapType& /*other*/) {}
  virtual void mergeAllMissionsFromMap(
      const TestMapType& /*source_map_merge_from*/) {}

  static std::string getSubFolderName() {
    return "";
  }
  static bool hasMapOnFilSytem(const std::string /*folder_path*/) {
    return false;
  }
  virtual bool loadFromFolder(const std::string& /*folder_path*/) {
    return false;
  }
  virtual bool saveToFolder(
      const std::string& /*folder_path*/, const SaveConfig& /*config*/) {
    return false;
  }

  void incrementCounter() {
    ++counter_;
  }
  size_t getCounter() const {
    return counter_;
  }

 private:
  size_t counter_;
};

template <>
struct traits<TestMapType> : public MapTraits<TestMapType> {};

}  // namespace backend

#endif  // MAP_MANAGER_TEST_TEST_MAP_TYPE_H_
