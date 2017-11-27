#ifndef MAPLAB_COMMON_MULTI_THREADED_PROGRESS_BAR_H_
#define MAPLAB_COMMON_MULTI_THREADED_PROGRESS_BAR_H_

#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace common {

/// \brief Prints out progress on a single line for multi-threaded work.
class MultiThreadedProgressBar {
 public:
  MultiThreadedProgressBar();
  virtual ~MultiThreadedProgressBar() {}

  /// Updates the progress bar given the current number of processed elements.
  /// Expecting num_elements_processed <= num_elements
  /// defined for this thread through a previous call to setNumElements(...).
  void update(size_t num_elements_processed);
  /// Updates the progress bar given the current number of processed elements
  /// and the total number of elements to process. Does not require a previous
  /// call to setNumElements(...).
  void update(
      size_t num_elements_processed,
      size_t total_number_of_elements_to_process);

  /// Assigns the total number of elements associated with the calling thread.
  void setNumElements(size_t num_elemens);

  /// Reset the progress bar state back to construction time.
  void reset();

 private:
  void update(
      size_t thread_index, size_t num_elements_processed,
      size_t total_number_of_elements_to_process);
  void print(bool force);
  void reevaluateParameters();

  size_t num_threads_;

  std::vector<size_t> num_elements_;

  struct ProgressInformation {
    ProgressInformation() : num_progress_symbols(0u), percentage(0.0) {}
    size_t num_progress_symbols;
    double percentage;
  };
  std::vector<ProgressInformation> progress_bar_heads_;

  std::mutex mutex_;

  std::vector<unsigned char> complete_;
  size_t num_complete_;

  size_t progress_bar_width_;
  double progress_bar_width_floating_point_;

  bool percentage_only_;

  typedef std::unordered_map<std::thread::id, size_t> ThreadIdToThreadIndexMap;
  ThreadIdToThreadIndexMap thread_id_to_thread_index_map_;

  std::chrono::system_clock::time_point last_print_timestamp_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_MULTI_THREADED_PROGRESS_BAR_H_
