#include "maplab-common/test/testing-entrypoint.h"

#include <thread>

#include <maplab-common/multi-threaded-progress-bar.h>

namespace common {

const size_t kNumElementsPerThread = 1000u;

void setTotalNumberOfElementsAndFillProgressBar(
    size_t stop_index, size_t total_num_elements_to_process,
    common::MultiThreadedProgressBar* mt_progress_bar) {
  CHECK_NOTNULL(mt_progress_bar);

  mt_progress_bar->setNumElements(total_num_elements_to_process);

  for (size_t i = 0; i <= stop_index; ++i) {
    mt_progress_bar->update(i);
    usleep(100);
  }
}

void fillProgressBarWithTotalNumElements(
    size_t total_num_elements, size_t stop_index,
    common::MultiThreadedProgressBar* mt_progress_bar) {
  CHECK_NOTNULL(mt_progress_bar);
  for (size_t i = 0; i <= stop_index; ++i) {
    mt_progress_bar->update(i, total_num_elements);
    usleep(100);
  }
}

TEST(ProgressBarTest, VerySimpleCase) {
  const size_t kNumThreads = 2u;
  std::vector<size_t> kNumElements(kNumThreads, kNumElementsPerThread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads.emplace_back(
        setTotalNumberOfElementsAndFillProgressBar, kNumElementsPerThread,
        kNumElementsPerThread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads[thread_index].join();
  }
}

TEST(ProgressBarTest, SimpleCase) {
  const size_t kNumThreads = 8u;
  std::vector<size_t> kNumElements(kNumThreads, kNumElementsPerThread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads.emplace_back(
        setTotalNumberOfElementsAndFillProgressBar, kNumElementsPerThread,
        kNumElementsPerThread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads[thread_index].join();
  }
}

TEST(ProgressBarTest, Boundaries) {
  const size_t kNumThreads = 3u;

  std::vector<size_t> kNumElements(kNumThreads, kNumElementsPerThread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads.emplace_back(
        setTotalNumberOfElementsAndFillProgressBar, kNumElementsPerThread,
        kNumElementsPerThread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads[thread_index].join();
    EXPECT_DEATH(mt_progress_bar.update(kNumElementsPerThread + 1), "");
  }
}

TEST(ProgressBarTest, Reset) {
  size_t num_threads = 5u;
  size_t num_elements_per_thread = kNumElementsPerThread;
  const size_t stop_index = num_elements_per_thread / 2;

  std::vector<size_t> num_elements(num_threads, num_elements_per_thread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads.emplace_back(
        setTotalNumberOfElementsAndFillProgressBar, stop_index,
        num_elements_per_thread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads[thread_index].join();
  }

  num_threads = 7u;
  num_elements_per_thread = 400;

  num_elements.resize(num_threads, num_elements_per_thread);
  mt_progress_bar.reset();

  threads.clear();
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads.emplace_back(
        setTotalNumberOfElementsAndFillProgressBar, num_elements_per_thread,
        num_elements_per_thread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads[thread_index].join();
  }
}

TEST(ProgressBarTest, SimpleCase2) {
  const size_t kNumThreads = 8u;
  std::vector<size_t> kNumElements(kNumThreads, kNumElementsPerThread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads.emplace_back(
        fillProgressBarWithTotalNumElements, kNumElementsPerThread,
        kNumElementsPerThread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads[thread_index].join();
  }
}

TEST(ProgressBarTest, BoundariesSimple2) {
  const size_t kNumThreads = 3u;

  std::vector<size_t> kNumElements(kNumThreads, kNumElementsPerThread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads.emplace_back(
        fillProgressBarWithTotalNumElements, kNumElementsPerThread,
        kNumElementsPerThread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads[thread_index].join();
    EXPECT_DEATH(
        mt_progress_bar.update(
            kNumElementsPerThread + 1, kNumElementsPerThread),
        "");
  }
}

TEST(ProgressBarTest, BoundariesSimple3) {
  const size_t kNumThreads = 3u;

  std::vector<size_t> kNumElements(kNumThreads, kNumElementsPerThread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads.emplace_back(
        fillProgressBarWithTotalNumElements, kNumElementsPerThread,
        kNumElementsPerThread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < kNumThreads; ++thread_index) {
    threads[thread_index].join();
    EXPECT_DEATH(mt_progress_bar.update(1), "");
  }
}

TEST(ProgressBarTest, ResetSimple) {
  size_t num_threads = 5u;
  size_t num_elements_per_thread = kNumElementsPerThread;
  const size_t stop_index = num_elements_per_thread / 2;

  std::vector<size_t> num_elements(num_threads, num_elements_per_thread);
  common::MultiThreadedProgressBar mt_progress_bar;

  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads.emplace_back(
        fillProgressBarWithTotalNumElements, num_elements_per_thread,
        stop_index, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads[thread_index].join();
  }

  num_threads = 7u;
  num_elements_per_thread = 400;

  num_elements.resize(num_threads, num_elements_per_thread);
  mt_progress_bar.reset();

  threads.clear();
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads.emplace_back(
        fillProgressBarWithTotalNumElements, num_elements_per_thread,
        num_elements_per_thread, &mt_progress_bar);
  }
  for (size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
    threads[thread_index].join();
  }
}
}  // namespace common

MAPLAB_UNITTEST_ENTRYPOINT
