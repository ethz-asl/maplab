#include <thread>

#include "maplab-common/ordered-monitors.h"
#include "maplab-common/parallel-process.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {

class OrderedMonitorsTest : public ::testing::Test {
 public:
  OrderedMonitorsTest() : monitors_(1, 'c') {}

 protected:
  enum ProtectedObjects { A, B };
  // This enforces that the contained int and char can only be accessed by
  // locking them and that, if both are accessed, the int must be accessed
  // first. The above enum is for convenience: A corresponds to the int and B to
  // the char. See the OrderedMonitors header to understand why this is useful.
  typedef OrderedMonitors<int, char> Monitors;
  Monitors monitors_;
};

TEST_F(OrderedMonitorsTest, ProperAccess) {
  Monitors::OrderedAccess access(&monitors_);
  EXPECT_EQ(1, *access.get<A>());
  EXPECT_EQ('c', *access.get<B>());
}

TEST_F(OrderedMonitorsTest, ImProperAccess) {
  Monitors::OrderedAccess access(&monitors_);
  EXPECT_EQ('c', *access.get<B>());
  EXPECT_DEATH(*access.get<A>(), "^");
}

// This tests that the locking actually works, and that no two threads access
// a resource at the same time. Although the test is technically indterministic,
// failure is very highly unlikely with proper implementation.
TEST_F(OrderedMonitorsTest, Grind) {
  // Number must be even for test to succeed.
  int kNumber = 1000;
  const bool kAlwaysParallelize = true;
  const size_t kNumThreads = 8;
  ParallelProcess(
      kNumber,
      [this](const std::vector<size_t>& range) {
        for (const size_t i : range) {
          Monitors::OrderedAccess access(&monitors_);
          ++(*access.get<A>());
          // Switches between 'c' and 'd'.
          *access.get<B>() += ((i % 2) ? 1 : -1);
        }
      },
      kAlwaysParallelize, kNumThreads);

  Monitors::OrderedAccess access(&monitors_);
  EXPECT_EQ(kNumber + 1, *access.get<A>());
  EXPECT_EQ('c', *access.get<B>());
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
