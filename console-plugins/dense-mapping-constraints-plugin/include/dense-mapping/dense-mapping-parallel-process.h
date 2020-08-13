#ifndef DENSE_MAPPING_DENSE_MAPPING_PARALLEL_PROCESS_H_
#define DENSE_MAPPING_DENSE_MAPPING_PARALLEL_PROCESS_H_

#include <algorithm>
#include <functional>
#include <thread>
#include <vector>

#include <glog/logging.h>

namespace dense_mapping {

// Simplified and adapted version of maplab-common parallel-process.h. This
// version also provides a thread index, which allows for per-thread result
// vectors and per-thread input data and external variables.
template <typename Functor>
size_t parallelProcess(
    const Functor& functor, const size_t global_start_index,
    const size_t global_end_index, const size_t num_threads) {
  CHECK_GT(num_threads, 0u);
  const size_t num_elements = global_end_index - global_start_index;
  if (num_elements == 0u) {
    // Nothing todo.
    return 0u;
  }

  const size_t num_processing_threads = std::min(num_elements, num_threads);
  const size_t num_elements_per_thread =
      std::ceil(static_cast<double>(num_elements) / num_processing_threads);

  // If we have less elements than threads, we simply run one thread.
  std::vector<std::thread> thread_pool;
  for (size_t thread_idx = 0u; thread_idx < num_processing_threads;
       ++thread_idx) {
    const size_t start_idx =
        global_start_index + num_elements_per_thread * thread_idx;

    // This is needed when there are more threads than elements.
    if (start_idx >= global_end_index) {
      // Just add this as a sanity check.
      CHECK_GT(num_elements, num_processing_threads);
      break;
    }

    const size_t end_idx =
        std::min(global_end_index, start_idx + num_elements_per_thread);

    CHECK_GE(start_idx, global_start_index);
    CHECK_LT(start_idx, end_idx);
    CHECK_LE(end_idx, global_end_index);

    thread_pool.emplace_back(
        std::thread([&functor, thread_idx, start_idx, end_idx]() -> void {
          functor(thread_idx, start_idx, end_idx);
        }));
  }
  const size_t actual_num_threads = thread_pool.size();
  CHECK_GT(actual_num_threads, 0u);

  // Wait for threads to finish.
  for (size_t thread_idx = 0u; thread_idx < actual_num_threads; ++thread_idx) {
    if (thread_pool[thread_idx].joinable()) {
      thread_pool[thread_idx].join();
    }
  }
  thread_pool.clear();
  return actual_num_threads;
}

}  // namespace dense_mapping

#endif  // DENSE_MAPPING_DENSE_MAPPING_PARALLEL_PROCESS_H_
