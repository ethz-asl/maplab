#include "maplab-common/multi-threaded-progress-bar.h"

#include <chrono>
#include <iomanip>
#include <iostream>  // NOLINT
#include <math.h>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_bool(
    show_progress_bar, true,
    "Set to false to disable the output of the progress bar (useful for "
    "benchmarking).");

namespace common {
const std::string kPreText = "Progress: ";
const std::string kProgressSymbol = "#";
const std::string kFillSymbol = "-";
const std::string kPostText = "]";
constexpr size_t kTotalProgressBarWidth = 100u;
const std::string kSplitText = " ";
constexpr int64_t kMinTimeBetweenTwoPrintOutsMilliseconds = 250;

MultiThreadedProgressBar::MultiThreadedProgressBar()
    : num_threads_(0u),
      num_complete_(0u),
      progress_bar_width_(0u),
      progress_bar_width_floating_point_(0.0),
      percentage_only_(false) {}

void MultiThreadedProgressBar::reevaluateParameters() {
  double space_to_divide = static_cast<double>(
      static_cast<int>(kTotalProgressBarWidth) -
      static_cast<int>(kPreText.size()) -
      (static_cast<int>(num_threads_) *
       (10 + 2 * (static_cast<int>(kProgressSymbol.size())) +
        static_cast<int>(kFillSymbol.size()))));

  percentage_only_ =
      (static_cast<int>(space_to_divide) <
       (3 * static_cast<int>(num_threads_)));

  if (!percentage_only_) {
    progress_bar_width_ = static_cast<size_t>(
        floor(space_to_divide / static_cast<double>(num_threads_)));
    progress_bar_width_floating_point_ =
        static_cast<double>(progress_bar_width_);
  }
}

void MultiThreadedProgressBar::reset() {
  num_threads_ = 0u;
  num_complete_ = 0u;
  percentage_only_ = false;
  progress_bar_width_ = 0u;
  progress_bar_width_floating_point_ = 0.0;
  num_elements_.clear();
  progress_bar_heads_.clear();
  thread_id_to_thread_index_map_.clear();
  if (FLAGS_show_progress_bar) {
    std::cout << std::endl;
  }
}

void MultiThreadedProgressBar::update(size_t num_elements_processed) {
  std::thread::id thread_id = std::this_thread::get_id();

  size_t thread_index;
  size_t num_elements;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ThreadIdToThreadIndexMap::const_iterator it =
        thread_id_to_thread_index_map_.find(thread_id);
    CHECK(it != thread_id_to_thread_index_map_.end());
    thread_index = it->second;
    CHECK_LT(thread_index, complete_.size());

    // num_elements_ is resized on setNumElements called by a new thread and
    // needs to be protected.
    num_elements = num_elements_[thread_index];
  }
  update(thread_index, num_elements_processed, num_elements);
}

void MultiThreadedProgressBar::update(
    size_t num_elements_processed, size_t total_number_of_elements_to_process) {
  std::thread::id thread_id = std::this_thread::get_id();

  size_t thread_index;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // num_threads_ is not threadsafe as it might be modified when a new
    // thread appears.
    thread_index = num_threads_;

    ThreadIdToThreadIndexMap::const_iterator it =
        thread_id_to_thread_index_map_.find(thread_id);
    if (it == thread_id_to_thread_index_map_.end()) {
      // This is a new thread.
      thread_id_to_thread_index_map_.emplace(thread_id, thread_index);
      ++num_threads_;

      complete_.resize(num_threads_);
      complete_[thread_index] = false;

      progress_bar_heads_.resize(num_threads_);
      progress_bar_heads_[thread_index] = ProgressInformation();

      reevaluateParameters();
    } else {
      thread_index = it->second;
    }
  }

  update(
      thread_index, num_elements_processed,
      total_number_of_elements_to_process);
}
void MultiThreadedProgressBar::setNumElements(size_t num_elements) {
  std::thread::id thread_id = std::this_thread::get_id();
  std::lock_guard<std::mutex> lock(mutex_);
  size_t thread_index = num_threads_;
  ThreadIdToThreadIndexMap::const_iterator it =
      thread_id_to_thread_index_map_.find(thread_id);
  if (it == thread_id_to_thread_index_map_.end()) {
    // This is a new thread.
    thread_id_to_thread_index_map_.emplace(thread_id, thread_index);
    ++num_threads_;

    complete_.resize(num_threads_);
    complete_[thread_index] = false;

    progress_bar_heads_.resize(num_threads_);
    progress_bar_heads_[thread_index] = ProgressInformation();

    num_elements_.resize(num_threads_);

    reevaluateParameters();
  } else {
    thread_index = it->second;
  }

  num_elements_[thread_index] = num_elements;
}
void MultiThreadedProgressBar::update(
    size_t thread_index, size_t num_elements_processed,
    size_t total_number_of_elements_to_process) {
  CHECK_LE(num_elements_processed, total_number_of_elements_to_process)
      << "The given number of "
      << "processed elements (" << num_elements_processed << ") is greater than"
      << " the total number of elements to process ("
      << total_number_of_elements_to_process << "). "
      << "Use reset(new_total_number_of_elements) to reset the progress bar "
      << "to a new range (caution: this also resets the number of processed "
      << "elements to zero!).";
  CHECK_LT(thread_index, complete_.size());
  if (!complete_[thread_index]) {
    bool force = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double num_progress_symbols =
          progress_bar_width_ * static_cast<double>(num_elements_processed) /
          static_cast<double>(total_number_of_elements_to_process);
      const double percentage =
          100.0 * static_cast<double>(num_elements_processed) /
          static_cast<double>(total_number_of_elements_to_process);

      const size_t num_progress_symbols_integer = floor(num_progress_symbols);
      CHECK_LE(num_progress_symbols_integer, progress_bar_width_)
          << "Number of "
          << "progress bar symbols is greater than the progress bar width. "
             "Make "
          << "sure the number of processed elements (" << num_elements_processed
          << ") is at most equal to the total number of elements to be "
             "processed "
             "("
          << total_number_of_elements_to_process << ")";

      progress_bar_heads_[thread_index].num_progress_symbols =
          num_progress_symbols_integer;
      progress_bar_heads_[thread_index].percentage = percentage;

      if (num_elements_processed >= total_number_of_elements_to_process) {
        complete_[thread_index] = true;
        ++num_complete_;
        force = true;
      }
    }

    print(force);
  }
}

void MultiThreadedProgressBar::print(bool force) {
  if (!FLAGS_show_progress_bar) {
    return;
  }

  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  int64_t ms_since_print =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_print_timestamp_)
          .count();
  if (force || (ms_since_print > kMinTimeBetweenTwoPrintOutsMilliseconds)) {
    std::stringstream stringstr;

    std::cout << "\r";
    stringstr << kPreText;
    // Iterate over all threads and update their progress bar.
    for (size_t thread_index = 0; thread_index < num_threads_; ++thread_index) {
      const ProgressInformation& progress_information =
          progress_bar_heads_[thread_index];

      stringstr << thread_index << ":" << std::setfill(' ') << std::fixed
                << std::setw(5) << std::setprecision(1)
                << progress_information.percentage << "% ";
      if (!percentage_only_) {
        stringstr << "[";
        for (size_t idx = 0; idx < progress_bar_width_; ++idx) {
          if (idx < progress_information.num_progress_symbols) {
            stringstr << kProgressSymbol;
          } else {
            stringstr << kFillSymbol;
          }
        }
        stringstr << kPostText;
      }

      if ((thread_index + 1) < num_threads_) {
        stringstr << kSplitText;
      }
    }
    size_t text_width = stringstr.str().size();
    for (size_t idx = text_width; idx < kTotalProgressBarWidth; ++idx) {
      stringstr << " ";
    }
    std::cout << stringstr.str();

    if (num_complete_ == num_threads_) {
      std::cout << "\r" << std::endl << "--->Complete!" << std::endl;
    }
    std::cout.flush();
    last_print_timestamp_ = now;
  }
}

}  // namespace common
