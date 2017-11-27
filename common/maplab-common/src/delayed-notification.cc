#include "maplab-common/delayed-notification.h"

#include <condition_variable>
#include <thread>

#include <glog/logging.h>

#include "maplab-common/conversions.h"

namespace common {

DelayedNotification::DelayedNotification(
    const size_t timeout_ms, const std::function<void(void)>& action)
    : helper_(new Helper(action)) {
  std::mutex m_thread_ready;
  std::condition_variable cv_thread_ready;
  std::unique_lock<std::mutex> lock(m_thread_ready);

  std::thread([&m_thread_ready, &cv_thread_ready, &timeout_ms, this]() {
    std::unique_lock<std::mutex> lock(m_thread_ready);
    std::shared_ptr<Helper> helper = this->helper_;
    lock.unlock();
    cv_thread_ready.notify_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    helper->invokeUnlessDiscarded();
  }).detach();

  cv_thread_ready.wait(lock);
}

DelayedNotification::~DelayedNotification() {
  discard();
}

void DelayedNotification::discard() {
  helper_->discard();
}

DelayedNotification::Helper::Helper(const std::function<void(void)>& action)
    : action_(action), discarded_(false) {}

void DelayedNotification::Helper::invokeUnlessDiscarded() const {
  if (!discarded_) {
    action_();
  }
}

void DelayedNotification::Helper::discard() {
  discarded_ = true;
}

}  // namespace common
