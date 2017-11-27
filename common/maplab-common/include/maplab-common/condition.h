#ifndef MAPLAB_COMMON_CONDITION_H_
#define MAPLAB_COMMON_CONDITION_H_

#include <condition_variable>
#include <mutex>

namespace common {

class Condition {
 public:
  Condition();
  void wait() const;
  void notify();

 private:
  bool bool_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_CONDITION_H_
