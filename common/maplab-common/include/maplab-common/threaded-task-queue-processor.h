#ifndef MAPLAB_COMMON_THREADED_TASK_QUEUE_PROCESSOR_H_
#define MAPLAB_COMMON_THREADED_TASK_QUEUE_PROCESSOR_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <maplab-common/macros.h>
#include <maplab-common/threadsafe-queue.h>

namespace common {

typedef std::function<void()> WrappedCall;

class TaskQueue {
 public:
  MAPLAB_POINTER_TYPEDEFS(TaskQueue);
  virtual ~TaskQueue() {}
  virtual void shutdown() {}
  virtual bool processSpin() = 0;
  virtual void addTask(const WrappedCall& call) = 0;
};

class ProcessNewestDropRestQueue : public TaskQueue {
 public:
  ProcessNewestDropRestQueue() : new_data_(false), shutdown_(false) {}
  virtual ~ProcessNewestDropRestQueue() {}

  virtual void addTask(const WrappedCall& call) {
    CHECK(call);
    std::lock_guard<std::mutex> lock(m_last_call_);
    last_call_ = call;
    new_data_ = true;
    cv_last_call_.notify_all();
  }

  virtual void shutdown() override {
    shutdown_ = true;
    cv_last_call_.notify_all();
  }

  virtual bool processSpin() override final {
    // Wait for new data.
    std::unique_lock<std::mutex> lock(m_last_call_);
    while (!new_data_) {
      cv_last_call_.wait(lock);

      if (shutdown_) {
        return false;
      }
    }

    // Grab the most recent queued call.
    WrappedCall last_call_copy;
    last_call_copy = last_call_;
    new_data_ = false;

    // Run the call.
    lock.unlock();
    CHECK(last_call_copy);
    last_call_copy();
    return true;
  }

 private:
  WrappedCall last_call_;
  std::mutex m_last_call_;
  std::condition_variable cv_last_call_;
  bool new_data_;
  std::atomic<bool> shutdown_;
};

class ProcessAllSequentiallyQueue : public TaskQueue {
 public:
  virtual ~ProcessAllSequentiallyQueue() {}
  virtual void addTask(const WrappedCall& call) {
    CHECK(call);
    queue_.Push(call);
  }

  virtual void shutdown() override {
    queue_.Shutdown();
  }

  virtual bool processSpin() override final {
    WrappedCall delayed_call;
    if (!queue_.PopBlocking(&delayed_call)) {
      // Queue was shut down.
      return false;
    }
    CHECK(delayed_call);
    delayed_call();
    return true;
  }

 private:
  common::ThreadSafeQueue<WrappedCall> queue_;
};

template <typename ProcessingPolicy = ProcessAllSequentiallyQueue>
class ThreadedTaskQueueProcessor {
 public:
  MAPLAB_POINTER_TYPEDEFS(ThreadedTaskQueueProcessor);

  explicit ThreadedTaskQueueProcessor(bool launch_processing = true) {
    processing_policy_.reset(new ProcessingPolicy);
    if (launch_processing) {
      startProcessing();
    }
  }

  ~ThreadedTaskQueueProcessor() {
    stopProcessingBlocking();
  }

  template <typename FunctionType, typename... Args>
  void addTask(FunctionType&& func, Args&&... args) {
    CHECK(processing_policy_);
    processing_policy_->addTask(
        std::bind(
            std::forward<FunctionType>(func), std::forward<Args>(args)...));
  }

  void startProcessing() {
    shutdown_requested_ = false;
    if (!queue_processor_) {
      queue_processor_.reset(
          new std::thread(&ThreadedTaskQueueProcessor::workThread, this));
    }
  }

  void stopProcessingBlocking() {
    processing_policy_->shutdown();
    shutdown_requested_ = true;
    if (queue_processor_) {
      queue_processor_->join();
    }
  }

 private:
  void workThread() {
    while (!shutdown_requested_) {
      if (!processing_policy_->processSpin()) {
        return;
      }
    }
  }

  TaskQueue::UniquePtr processing_policy_;
  std::unique_ptr<std::thread> queue_processor_;
  std::atomic<bool> shutdown_requested_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_THREADED_TASK_QUEUE_PROCESSOR_H_
