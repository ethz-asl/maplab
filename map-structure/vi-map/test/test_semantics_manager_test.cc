#include <maplab-common/test/testing-entrypoint.h>

#include "vi-map/semantics-manager.h"
#include "vi-map/unique-id.h"

namespace vi_map {

class SemanticsManagerTest : public ::testing::Test {
 protected:
  SemanticsManager semantics_;
  static const std::string kNames[2];
};

const std::string SemanticsManagerTest::kNames[2] = {"name1", "name2"};

TEST_F(SemanticsManagerTest, QueryInexistent) {
  EXPECT_FALSE(semantics_.getMissionIdForName(kNames[0]).isValid());
}

TEST_F(SemanticsManagerTest, AddAndQuery) {
  MissionId id;
  common::generateId(&id);
  semantics_.nameMission(id, kNames[0]);
  EXPECT_EQ(semantics_.getMissionIdForName(kNames[0]), id);
}

TEST_F(SemanticsManagerTest, NameSameMissionTwice) {
  MissionId id;
  common::generateId(&id);
  semantics_.nameMission(id, kNames[0]);
  EXPECT_DEATH(semantics_.nameMission(id, kNames[1]), "^");
}

TEST_F(SemanticsManagerTest, UseSameNameTwice) {
  MissionId ids[2];
  common::generateId(&ids[0]);
  common::generateId(&ids[1]);
  semantics_.nameMission(ids[0], kNames[0]);
  EXPECT_DEATH(semantics_.nameMission(ids[1], kNames[0]), "^");
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
