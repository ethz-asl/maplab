#include <array>
#include <mutex>
#include <thread>

#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/threadsafe-queue.h"

namespace common {
struct Args {
  common::ThreadSafeQueue<int>* queue_jobs;
  common::ThreadSafeQueue<int>* queue_finished;
  size_t num;
};

constexpr size_t kNumConsumers = 10u;
constexpr size_t kNumProducers = 10u;
constexpr size_t kNumJobs = 10u;
enum { num_consumers = 10, num_producers = 10, num_jobs = 100 };

void producer_block(Args* args);
void consumer_block(Args* args);

void producer_block_full(Args* args);
void consumer_block_full(Args* args);

void producer_nonblock(Args* args);
void consumer_nonblock(Args* args);

bool test_funcs(
    const std::function<void(Args*)>& consumer_function,
    const std::function<void(Args*)>& producer_function,
    const std::string& testname, bool /*expect_jobs_nondrop*/) {
  CHECK(consumer_function);
  CHECK(producer_function);
  CHECK(!testname.empty());

  std::array<std::thread, kNumProducers> producer_threads;
  std::array<std::thread, kNumConsumers> consumer_threads;

  common::ThreadSafeQueue<int> queue_jobs;
  common::ThreadSafeQueue<int> queue_finished;

  Args args;
  args.queue_jobs = &queue_jobs;
  args.queue_finished = &queue_finished;
  args.num = kNumJobs;

  std::cout << testname << ": Jobs in " << args.num << std::endl;

  for (size_t i = 0u; i < kNumProducers; ++i) {
    producer_threads[i] = std::thread(producer_function, &args);
  }
  for (size_t i = 0u; i < kNumConsumers; ++i) {
    consumer_threads[i] = std::thread(consumer_function, &args);
  }

  for (int i = 0; i < num_producers; ++i) {
    producer_threads[i].join();
  }
  for (int i = 0; i < num_consumers; ++i) {
    consumer_threads[i].join();
  }

  queue_jobs.Shutdown();
  queue_finished.Shutdown();

  return true;
}

void producer_block(Args* args) {
  CHECK_NOTNULL(args);
  for (size_t i = 0u; i < args->num; ++i) {
    args->queue_jobs->Push(i);
  }
}

void consumer_block(Args* args) {
  CHECK_NOTNULL(args);
  for (size_t i = 0u; i < args->num; ++i) {
    int val;
    bool got_value = args->queue_jobs->Pop(&val);
    if (!got_value)
      continue;
    args->queue_finished->Push(val);
  }
}

void producer_block_full(Args* args) {
  CHECK_NOTNULL(args);
  for (size_t i = 0u; i < args->num; ++i) {
    args->queue_jobs->PushBlockingIfFull(i, 5);
  }
}

void consumer_block_full(Args* args) {
  CHECK_NOTNULL(args);
  for (size_t i = 0u; i < args->num; ++i) {
    int val;
    bool got_value = args->queue_jobs->PopBlocking(&val);
    if (!got_value)
      continue;
    // This queue will fill up, so don't do blocking on full.
    args->queue_finished->Push(val);
  }
}

void producer_nonblock(Args* args) {
  CHECK_NOTNULL(args);
  for (size_t i = 0u; i < args->num; ++i) {
    args->queue_jobs->PushNonBlockingDroppingOldestElementIfFull(i, 5);
  }
}

void consumer_nonblock(Args* args) {
  CHECK_NOTNULL(args);
  for (size_t i = 0u; i < args->num; ++i) {
    int val;
    bool ok = args->queue_jobs->PopNonBlocking(&val);
    if (!ok) {
      continue;
    }
    args->queue_finished->Push(val);
  }
}

TEST(MaplabCommon, ThreadsafeQueue_ProducerBlockConsumerBlock) {
  test_funcs(producer_block, consumer_block, "Test-blocking", true);
}
TEST(MaplabCommon, ThreadsafeQueue_ProducerBlockFullConsumerBlockFull) {
  test_funcs(
      producer_block_full, consumer_block_full, "Test-blocking-full", true);
}
TEST(MaplabCommon, ThreadsafeQueue_ProducerNonBlockConsumerNonBlock) {
  test_funcs(producer_nonblock, consumer_nonblock, "Test-nonblocking", false);
}
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
