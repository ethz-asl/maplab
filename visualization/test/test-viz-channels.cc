#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <maplab-common/test/testing-entrypoint.h>

#include "visualization/internal/viz-channel.h"

namespace visualization {
namespace internal {

const std::string kChannelName("test-channel");
const std::string kTestData("test-data");

TEST(VizChannel, PushAndReadData) {
  VizChannelGroup channel_group;
  channel_group.setChannel<std::string>(kChannelName, kTestData);

  const std::string* data;
  EXPECT_TRUE(channel_group.getChannelSafe<std::string>(kChannelName, &data));
  EXPECT_TRUE(data != nullptr);
  EXPECT_EQ(*data, kTestData);

  EXPECT_EQ(channel_group.getChannel<std::string>(kChannelName), kTestData);
}

TEST(VizChannel, DeathOnSetSameChannelTwice) {
  VizChannelGroup channel_group;
  channel_group.setChannel<std::string>(kChannelName, kTestData);
  EXPECT_DEATH(
      channel_group.setChannel<std::string>(kChannelName, kTestData), "^");
}

TEST(VizChannel, DeathOnGetUnavailableChannelUnsafe) {
  VizChannelGroup channel_group;
  EXPECT_DEATH(channel_group.getChannel<std::string>(kChannelName), "^");
}

TEST(VizChannel, TestGetUnavailableChannelSafe) {
  VizChannelGroup channel_group;
  const std::string* data = nullptr;
  EXPECT_FALSE(channel_group.getChannelSafe<std::string>(kChannelName, &data));
  EXPECT_EQ(data, nullptr);
}

TEST(VizChannel, TestHasChannel) {
  VizChannelGroup channel_group;
  EXPECT_FALSE(channel_group.hasChannel(kChannelName));
  channel_group.setChannel<std::string>(kChannelName, kTestData);
  EXPECT_TRUE(channel_group.hasChannel(kChannelName));
}

}  // namespace internal
}  // namespace visualization

MAPLAB_UNITTEST_ENTRYPOINT
