#include "maplab-common/test/testing-entrypoint.h"
#include "maplab-common/threadsafe-queue.h"

namespace common {
struct Args {
  common::ThreadSafeQueue<int>* queue_jobs;
  common::ThreadSafeQueue<int>* queue_finished;
  int num;
};

enum { num_consumers = 10, num_producers = 10, num_jobs = 100 };

bool test_funcs(
    void* (*)(void*), void* (*)(void*),  // NOLINT
    const std::string&, bool);

void* producer_block(void* arguments);
void* consumer_block(void* arguments);

void* producer_block_full(void* arguments);
void* consumer_block_full(void* arguments);

void* producer_nonblock(void* arguments);
void* consumer_nonblock(void* arguments);

bool test_funcs(
    void* (*consumer_ptr)(void*),  // NOLINT
    void* (*producer_ptr)(void*),  // NOLINT
    const std::string& testname, bool /*expect_jobs_nondrop*/) {
  pthread_t producer_threads[num_producers];  // NOLINT
  pthread_t consumer_threads[num_consumers];  // NOLINT

  common::ThreadSafeQueue<int> queue_jobs;
  common::ThreadSafeQueue<int> queue_finished;

  Args args;
  args.queue_jobs = &queue_jobs;
  args.queue_finished = &queue_finished;
  args.num = num_jobs;

  std::cout << testname << ": Jobs in " << args.num << std::endl;

  for (int i = 0; i < num_producers; ++i) {
    pthread_create(
        &producer_threads[i], NULL, producer_ptr, static_cast<void*>(&args));
  }
  for (int i = 0; i < num_consumers; ++i) {
    pthread_create(
        &consumer_threads[i], NULL, consumer_ptr, static_cast<void*>(&args));
  }

  for (int i = 0; i < num_producers; ++i) {
    pthread_join(producer_threads[i], NULL);
  }
  for (int i = 0; i < num_consumers; ++i) {
    pthread_join(consumer_threads[i], NULL);
  }
  return true;
}

void* producer_block(void* arguments) {
  assert(arguments);
  Args* args = static_cast<Args*>(arguments);
  for (int i = 0; i < args->num; ++i) {
    args->queue_jobs->Push(i);
  }
  pthread_exit(NULL);
}

void* consumer_block(void* arguments) {
  assert(arguments);
  Args* args = static_cast<Args*>(arguments);
  for (int i = 0; i < args->num; ++i) {
    int val;
    bool got_value = args->queue_jobs->Pop(&val);
    if (!got_value)
      continue;
    args->queue_finished->Push(val);
  }
  pthread_exit(NULL);
}

void* producer_block_full(void* arguments) {
  assert(arguments);
  Args* args = static_cast<Args*>(arguments);
  for (int i = 0; i < args->num; ++i) {
    args->queue_jobs->PushBlockingIfFull(i, 5);
  }
  pthread_exit(NULL);
}

void* consumer_block_full(void* arguments) {
  assert(arguments);
  Args* args = static_cast<Args*>(arguments);
  for (int i = 0; i < args->num; ++i) {
    int val;
    bool got_value = args->queue_jobs->PopBlocking(&val);
    if (!got_value)
      continue;
    // This queue will fill up, so don't do blocking on full.
    args->queue_finished->Push(val);
  }
  pthread_exit(NULL);
}

void* producer_nonblock(void* arguments) {
  assert(arguments);
  Args* args = static_cast<Args*>(arguments);
  for (int i = 0; i < args->num; ++i) {
    args->queue_jobs->PushNonBlockingDroppingOldestElementIfFull(i, 5);
  }
  pthread_exit(NULL);
}

void* consumer_nonblock(void* arguments) {
  assert(arguments);
  Args* args = static_cast<Args*>(arguments);
  for (int i = 0; i < args->num; ++i) {
    int val;
    bool ok = args->queue_jobs->PopNonBlocking(&val);
    if (!ok) {
      continue;
    }
    args->queue_finished->Push(val);
  }
  pthread_exit(NULL);
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
