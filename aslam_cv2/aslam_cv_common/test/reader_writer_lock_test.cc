#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "aslam/common/entrypoint.h"
#include "./reader_writer_mutex_fixture.h"

constexpr int kNumThreads = 20;

namespace aslam {

TEST_F(ReaderWriterMutexFixture, ReaderWriterLock) {
  aslam::ReaderWriterMutex mutex;
  std::vector<std::thread> threads;
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([this]() { reader(); });
    threads.emplace_back([this]() { writer(); });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(0, value() % kMagicNumber);
}

TEST_F(ReaderWriterMutexFixture, UpgradeReaderLock) {
  std::vector<std::thread> threads;
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([this]() { delayedReader(); });
    threads.emplace_back([this]() { readerUpgrade(); });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  VLOG(3) << "Number of writes after upgrade: " << num_writes();
  VLOG(3) << "Number of failed upgrades: " << num_upgrade_failures();
  EXPECT_NE(0, value());
  EXPECT_NE(0, num_writes());
  EXPECT_EQ(value(), num_writes() * kMagicNumber);
}

}  // namespace aslam

ASLAM_UNITTEST_ENTRYPOINT
