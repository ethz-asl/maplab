#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "maplab-common/file-lock.h"
#include "maplab-common/test/testing-entrypoint.h"

constexpr int kNumThreads = 500;
constexpr int kAdditionValue = 42;
const char* kFileName = "test_lock.txt";

namespace common {

TEST(FileLockTest, FileLockTest) {
  int value = 0;
  std::vector<std::thread> threads;
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&value]() {
      ScopedFileLock file_lock(kFileName);
      value += kAdditionValue;
      usleep(200);
      value += kAdditionValue;
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(2 * kNumThreads * kAdditionValue, value);
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
