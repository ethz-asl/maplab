#include <gtest/gtest.h>

#include "maplab-common/conversions.h"
#include "maplab-common/delayed-notification.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {

TEST(DelayedNotificationTest, Test) {
  int var = 0;

  {
    DelayedNotification dn(100, [&]() { ++var; });
    usleep(50 * kMillisecondsToMicroseconds);
  }
  usleep(100 * kMillisecondsToMicroseconds);
  EXPECT_EQ(0, var);

  {
    DelayedNotification dn(100, [&]() { ++var; });
    usleep(300 * kMillisecondsToMicroseconds);
  }
  EXPECT_EQ(1, var);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
