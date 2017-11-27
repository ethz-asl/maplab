#include <unordered_map>

#include "maplab-common/accessors.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {
TEST(MaplabCommon, CheckedAccessors) {
  std::unordered_map<int, std::pair<int, double> > map;
  map.insert(std::make_pair(1, std::make_pair(2, 5.)));
  EXPECT_EQ(getChecked(map, 1).first, 2);
  EXPECT_EQ(getChecked(map, 1).second, 5.);
}

TEST(MaplabCommon, CheckedAccessorsDeathOnNonExistingEntry) {
  std::unordered_map<int, std::pair<int, double> > map;
  map.insert(std::make_pair(1, std::make_pair(2, 5.)));
  EXPECT_DEATH(getChecked(map, 2), "^");
}

TEST(MaplabCommon, CheckedAccessorsAllocator) {
  AlignedUnorderedMap<int, std::pair<int, double> > map;
  map.insert(std::make_pair(1, std::make_pair(2, 5.)));
  EXPECT_EQ(getChecked(map, 1).first, 2);
  EXPECT_EQ(getChecked(map, 1).second, 5.);
}

TEST(MaplabCommon, GetValuePtr) {
  std::unordered_map<int, double> map;
  map.insert(std::make_pair(1, 5));
  EXPECT_EQ(getValuePtr(map, 2), nullptr);

  ASSERT_NE(getValuePtr(map, 1), nullptr);
  EXPECT_EQ(*getValuePtr(map, 1), 5);
}
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
