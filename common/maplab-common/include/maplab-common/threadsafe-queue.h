#ifndef MAPLAB_COMMON_THREADSAFE_QUEUE_H_
#define MAPLAB_COMMON_THREADSAFE_QUEUE_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <sys/time.h>

#include <glog/logging.h>
#include <maplab-common/macros.h>

namespace common {

class ThreadSafeQueueBase {
 public:
  MAPLAB_POINTER_TYPEDEFS(ThreadSafeQueueBase);
  ThreadSafeQueueBase() = default;
  virtual ~ThreadSafeQueueBase() {}
  virtual void NotifyAll() const = 0;
  virtual void Shutdown() = 0;
  virtual void Resume() = 0;
  virtual size_t Size() const = 0;
  virtual bool Empty() const = 0;
};

template <typename QueueType>
class ThreadSafeQueue final : public ThreadSafeQueueBase {
  friend bool test_funcs(
      void* (*)(void*), void* (*)(void*),  // NOLINT
      const std::string&, bool);

 public:
  MAPLAB_POINTER_TYPEDEFS(ThreadSafeQueue);
  void NotifyAll() const override {
    condition_empty_.notify_all();
    condition_full_.notify_all();
  }

  ThreadSafeQueue() : shutdown_(false) {}

  ~ThreadSafeQueue() override {
    Shutdown();
  }

  void Shutdown() override {
    shutdown_ = true;
    NotifyAll();
  }

  void Resume() override {
    shutdown_ = false;
    NotifyAll();
  }

  // Push to the queue.
  void Push(const QueueType& value) {
    PushNonBlocking(value);
  }

  // Push to the queue.
  void PushNonBlocking(const QueueType& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
    condition_empty_.notify_all();
  }

  size_t Size() const override {  // NOLINT
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  bool Empty() const override {  // NOLINT
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  // Push to the queue if the size is less than max_queue_size, else block.
  bool PushBlockingIfFull(const QueueType& value, size_t max_queue_size) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutdown_) {
      const size_t size = queue_.size();
      if (size >= max_queue_size) {
        condition_full_.wait(lock);
      }
      if (size >= max_queue_size) {
        continue;
      }
      queue_.push(value);
      condition_empty_.notify_all();
      return true;
    }
    return false;
  }

  // Returns true if oldest was dropped because queue was full.
  bool PushNonBlockingDroppingOldestElementIfFull(
      const QueueType& value, size_t max_queue_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool dropped_oldest = false;
    if (queue_.size() >= max_queue_size) {
      queue_.pop();
      dropped_oldest = true;
    }
    queue_.push(value);
    condition_empty_.notify_all();
    return dropped_oldest;
  }

  // Pops from the queue blocking if queue is empty.
  bool Pop(QueueType* value) {
    return PopBlocking(value);
  }

  // Pops from the queue blocking if queue is empty.
  bool PopBlocking(QueueType* value) {
    CHECK_NOTNULL(value);
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutdown_) {
      if (queue_.empty()) {
        condition_empty_.wait(lock);
      }
      if (queue_.empty()) {
        continue;
      }
      *value = queue_.front();
      queue_.pop();
      condition_full_.notify_all();
      return true;
    }
    return false;
  }

  // Check queue is empty, if yes return false, not altering value. If queue not
  // empty update value and return true.
  bool PopNonBlocking(QueueType* value) {
    CHECK_NOTNULL(value);
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    *value = queue_.front();
    queue_.pop();
    return true;
  }
  // Check queue is empty, if yes return false, not altering value. If queue not
  // empty update value and return true.
  bool PopTimeout(QueueType* value, int64_t timeout_nanoseconds) {
    CHECK_NOTNULL(value);
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      condition_empty_.wait_for(
          lock, std::chrono::nanoseconds(timeout_nanoseconds));
    }
    if (queue_.empty()) {
      return false;
    }
    QueueType _value = queue_.front();
    queue_.pop();
    condition_full_.notify_all();
    *value = _value;
    return true;
  }

  // Get a copy of the front element of the queue. Returns false if empty.
  bool getCopyOfFront(QueueType* value) {
    CHECK_NOTNULL(value);
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    // COPY the value.
    *value = queue_.front();
    return true;
  }

  // Get a copy of the front element of the queue. Returns false if the queue is
  // shut down.
  bool getCopyOfFrontBlocking(QueueType* value) {
    CHECK_NOTNULL(value);
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutdown_) {
      if (queue_.empty()) {
        condition_empty_.wait(lock);
      }
      if (queue_.empty()) {
        continue;
      }
      *value = queue_.front();
      return true;
    }
    return false;
  }

  // Get a copy of the back element of the queue. Returns false if empty.
  bool getCopyOfBack(QueueType* value) {
    CHECK_NOTNULL(value);
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    // COPY the value.
    *value = queue_.back();
    return true;
  }

  // Get a copy of the back element of the queue. Returns false if the queue is
  // shut down.
  bool getCopyOfBackBlocking(QueueType* value) {
    CHECK_NOTNULL(value);
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutdown_) {
      if (queue_.empty()) {
        condition_empty_.wait(lock);
      }
      if (queue_.empty()) {
        continue;
      }
      *value = queue_.back();
      return true;
    }
    return false;
  }

  mutable std::mutex mutex_;
  mutable std::condition_variable condition_empty_;
  mutable std::condition_variable condition_full_;
  std::queue<QueueType> queue_;
  std::atomic_bool shutdown_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_THREADSAFE_QUEUE_H_
