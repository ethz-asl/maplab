#ifndef MAPLAB_COMMON_DELAYED_NOTIFICATION_H_
#define MAPLAB_COMMON_DELAYED_NOTIFICATION_H_

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include <glog/logging.h>

// Using a macro extracts the correct file and line.
#define DELAYED_NOTIFICATION(level, message, timeout_ms) \
  common::DelayedNotification __dn__##__LINE__(          \
      timeout_ms, []() { LOG(level) << message; })
#define DELAYED_NOTIFICATION_1S(level, message) \
  DELAYED_NOTIFICATION(level, message, 1000);

namespace common {

// Executes action unless destroyed or discarded within timeout_ms.
class DelayedNotification {
 public:
  DelayedNotification(
      const size_t timeout_ms, const std::function<void(void)>& action);
  ~DelayedNotification();
  void discard();

 private:
  class Helper {
   public:
    Helper(const std::function<void(void)>& action);
    void invokeUnlessDiscarded() const;
    void discard();

   private:
    std::function<void(void)> action_;
    std::atomic<bool> discarded_;
  };

  std::shared_ptr<Helper> helper_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_DELAYED_NOTIFICATION_H_
