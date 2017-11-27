#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include <aslam/common/timer.h>
#include <gtest/gtest.h>

#include "maplab-common/conversions.h"
#include "maplab-common/monitor.h"
#include "maplab-common/test/testing-entrypoint.h"

namespace common {

class MonitorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    started_threads_ = 0u;
  }

  void launchThreads(
      const size_t numThreads, const std::function<void()>& function) {
    for (size_t i = 0u; i < numThreads; ++i) {
      thread_list_.emplace_back(function);
    }
  }

  void joinAllThreads() {
    for (std::thread& thread : thread_list_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

  void waitForNotification() {
    std::unique_lock<std::mutex> lock(thread_start_mutex_);
    ++started_threads_;
    thread_start_notification_.wait(lock);
  }

  void waitForThreadsAndNotifyAll(const size_t num_threads) {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(thread_start_mutex_);
        if (started_threads_ == num_threads) {
          break;
        }
      }
      static constexpr size_t kSleepTimeMs = 25u;
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    }
    thread_start_notification_.notify_all();
  }

  std::vector<std::thread> thread_list_;
  std::condition_variable thread_start_notification_;
  std::mutex thread_start_mutex_;
  size_t started_threads_;
};

TEST_F(MonitorTest, ReadLockOnly) {
  constexpr int kStartNumber = 10;
  constexpr size_t kSleepTimeMs = 25u;
  constexpr size_t kNumThreads = 10u;

  Monitor<int> monitor(kStartNumber);

  std::function<void()> read_function = [&kStartNumber, &kSleepTimeMs, this,
                                         &monitor]() {
    waitForNotification();
    Monitor<int>::ReadAccess read_access = monitor.getReadAccess();
    EXPECT_EQ(kStartNumber, *read_access);
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    EXPECT_EQ(kStartNumber, *read_access);
  };

  timing::TimerImpl timer("monitor_test_read_lock_only");
  launchThreads(kNumThreads, read_function);
  waitForThreadsAndNotifyAll(kNumThreads);
  joinAllThreads();

  constexpr double kTimeSafetyMargin = 1.1;
  EXPECT_LE(
      timer.Stop(), kMilliSecondsToSeconds * (kNumThreads * kSleepTimeMs) *
                        kTimeSafetyMargin);
}

TEST_F(MonitorTest, WriteLockOnly) {
  constexpr int kStartNumber = 10;
  constexpr size_t kSleepTimeMs = 25u;
  constexpr size_t kNumThreads = 5u;

  Monitor<int> monitor(kStartNumber);

  std::function<void()> write_function = [&kStartNumber, &kSleepTimeMs, this,
                                          &monitor]() {
    waitForNotification();
    Monitor<int>::WriteAccess write_access = monitor.getWriteAccess();
    ++(*write_access);
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    --(*write_access);
    EXPECT_EQ(kStartNumber, *write_access);
  };

  timing::TimerImpl timer("monitor_test_write_lock_only");
  launchThreads(kNumThreads, write_function);
  waitForThreadsAndNotifyAll(kNumThreads);
  joinAllThreads();

  constexpr double kTimeSafetyMargin = 1.1;
  EXPECT_GE(
      timer.Stop(), kMilliSecondsToSeconds *
                        static_cast<double>(kNumThreads * kSleepTimeMs) /
                        kTimeSafetyMargin);
}

TEST_F(MonitorTest, ReadAndWriteLock) {
  constexpr int kStartNumber = 10;
  constexpr size_t kSleepTimeMs = 25u;
  constexpr size_t kNumThreads = 5u;

  Monitor<int> monitor(kStartNumber);

  std::function<void()> read_function = [&kStartNumber, &kSleepTimeMs, this,
                                         &monitor]() {
    waitForNotification();
    Monitor<int>::ReadAccess read_access = monitor.getReadAccess();
    EXPECT_EQ(kStartNumber, *read_access);
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    EXPECT_EQ(kStartNumber, *read_access);
  };

  std::function<void()> write_function = [&kStartNumber, &kSleepTimeMs, this,
                                          &monitor]() {
    waitForNotification();
    Monitor<int>::WriteAccess write_access = monitor.getWriteAccess();
    ++(*write_access);
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    --(*write_access);
    EXPECT_EQ(kStartNumber, *write_access);
  };

  launchThreads(kNumThreads, read_function);
  launchThreads(kNumThreads, write_function);
  waitForThreadsAndNotifyAll(2u * kNumThreads);
  joinAllThreads();
}

}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
