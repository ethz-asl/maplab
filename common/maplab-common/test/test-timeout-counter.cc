#include <thread>

#include "maplab-common/conversions.h"
#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/timeout-counter.h"

namespace common {
TEST(TimeoutCounter, ResetAndReached) {
  constexpr int64_t duration_ns = 0.05 * kSecondsToNanoSeconds;

  TimeoutCounter timeout_counter(duration_ns);
  EXPECT_FALSE(timeout_counter.reached());
  std::this_thread::sleep_for(std::chrono::nanoseconds(duration_ns));
  EXPECT_TRUE(timeout_counter.reached());

  timeout_counter.reset();
  EXPECT_FALSE(timeout_counter.reached());
  std::this_thread::sleep_for(std::chrono::nanoseconds(duration_ns));
  EXPECT_TRUE(timeout_counter.reached());
}
}  // namespace common
MAPLAB_UNITTEST_ENTRYPOINT
