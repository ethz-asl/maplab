#include <gtest/gtest.h>

#include <aslam/common/entrypoint.h>
#include <aslam/common/thread-pool.h>

int increment(int a){
  std::chrono::milliseconds dura(a);
  std::this_thread::sleep_for(dura);
  ++a;
  return a;
}

TEST(ThreadPoolTests, TestBasic) {
  aslam::ThreadPool pool(2);
  auto job1 = pool.enqueue(&increment, 500);
  auto job2 = pool.enqueue(&increment, 400);
  auto job3 = pool.enqueue(&increment, 300);
  auto job4 = pool.enqueue(&increment, 200);
  auto job5 = pool.enqueue(&increment, 100);
  pool.waitForEmptyQueue();

  EXPECT_EQ(501, job1.get());
  EXPECT_EQ(401, job2.get());
  EXPECT_EQ(301, job3.get());
  EXPECT_EQ(201, job4.get());
  EXPECT_EQ(101, job5.get());
}

TEST(ThreadPoolTests, TestOrdered) {
  aslam::ThreadPool pool(2);

  constexpr int kGroupId = 0u;
  auto job1 = pool.enqueueOrdered(kGroupId, &increment, 500);
  auto job2 = pool.enqueueOrdered(kGroupId, &increment, 400);
  auto job3 = pool.enqueueOrdered(kGroupId, &increment, 300);
  auto job4 = pool.enqueueOrdered(kGroupId, &increment, 200);
  auto job5 = pool.enqueueOrdered(kGroupId, &increment, 100);
  pool.waitForEmptyQueue();

  EXPECT_EQ(501, job1.get());
  EXPECT_EQ(401, job2.get());
  EXPECT_EQ(301, job3.get());
  EXPECT_EQ(201, job4.get());
  EXPECT_EQ(101, job5.get());
}

TEST(ThreadPoolTests, ExecutionOrdering) {
  constexpr size_t kNumThreads = 16u;
  aslam::ThreadPool pool(kNumThreads);

  constexpr size_t kNumGroups = 3u;
  std::vector<std::vector<double>> receive_queues(kNumGroups);
  std::vector<std::mutex> receive_queue_mutexes(kNumGroups);

  auto task = [&](size_t exclusivity_id, double value) {
    // Let's wait a small random amount of time to check if the threads respect
    // the incoming order of the messages when being delivered by multiple
    // threads.
    std::this_thread::sleep_for(std::chrono::microseconds(rand() % 10));
    std::lock_guard<std::mutex> lock(receive_queue_mutexes.at(exclusivity_id));
    receive_queues[exclusivity_id].emplace_back(value);
  };

  // Run the task multiple times through the thread-pool in multiple groups.
  size_t kNumNumbers = 1000;
  for (size_t number = 0u; number < kNumNumbers; ++number) {
    for (int group_id = 0; group_id < kNumGroups; ++group_id) {
      pool.enqueueOrdered(group_id, task, group_id, number);
    }
  }
  pool.waitForEmptyQueue();

  // Check order of the result queue.
  double value;
  for (int group_id = 0; group_id < kNumGroups; ++group_id) {
    const std::vector<double>& receive_queue = receive_queues.at(group_id);
    ASSERT_EQ(receive_queue.size(), kNumNumbers);
    for (size_t number = 0u; number < kNumNumbers; ++number) {
      EXPECT_EQ(receive_queue.at(number), number);
    }
  }
}

ASLAM_UNITTEST_ENTRYPOINT
