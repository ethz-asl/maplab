#ifndef MAPLAB_COMMON_FREQUENCY_ENFORCER_H_
#define MAPLAB_COMMON_FREQUENCY_ENFORCER_H_

#include <chrono>

#include <glog/logging.h>

namespace common {

class FrequencyEnforcer {
 public:
  FrequencyEnforcer() : first_run_(true), falling_behind_warning_count_(0) {}

  void EnforceFrequency(double target_frequency) {
    using std::chrono::duration_cast;
    using std::chrono::steady_clock;

    if (target_frequency <= 0)
      return;
    if (first_run_) {
      previous_deadline_ = steady_clock::now();
      first_run_ = false;
      return;
    }

    std::chrono::duration<double> target_period(1. / target_frequency);
    steady_clock::time_point new_deadline =
        previous_deadline_ +
        duration_cast<steady_clock::duration>(target_period);
    steady_clock::time_point current = steady_clock::now();
    if (current < new_deadline) {
      int desired_sleep_us =
          duration_cast<std::chrono::microseconds>(new_deadline - current)
              .count();
      usleep(desired_sleep_us);
      falling_behind_warning_count_ = 0;
    } else {
      // Complain politely if we're clearly falling behind.
      static const int kWarnEvery = 60;
      if (falling_behind_warning_count_ > 0 &&
          falling_behind_warning_count_ % kWarnEvery == 0) {
        int behind_by_ms =
            duration_cast<std::chrono::milliseconds>(current - new_deadline)
                .count();
        LOG(WARNING) << "Not keeping up with desired frequency "
                     << target_frequency << "Hz. Behind for last "
                     << falling_behind_warning_count_
                     << " frames for a total of " << behind_by_ms << "ms.";
      }
      ++falling_behind_warning_count_;
    }
    previous_deadline_ = new_deadline;
  }

 private:
  bool first_run_;
  std::chrono::steady_clock::time_point previous_deadline_;
  int falling_behind_warning_count_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_FREQUENCY_ENFORCER_H_
