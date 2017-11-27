#ifndef MAPLAB_COMMON_TIMEOUT_COUNTER_H_
#define MAPLAB_COMMON_TIMEOUT_COUNTER_H_
#include <chrono>
#include <memory>

namespace common {
class TimeoutCounter {
 public:
  explicit TimeoutCounter(int64_t duration_nanoseconds)
      : duration_ns_(duration_nanoseconds),
        start_time_(std::chrono::high_resolution_clock::now()) {}

  void reset() {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  bool reached() const {
    return (std::chrono::high_resolution_clock::now() - start_time_) >=
        duration_ns_;
  }

 private:
  const std::chrono::duration<int64_t, std::nano> duration_ns_;
  std::chrono::high_resolution_clock::time_point start_time_;
};
}  // namespace common
#endif  // MAPLAB_COMMON_TIMEOUT_COUNTER_H_
