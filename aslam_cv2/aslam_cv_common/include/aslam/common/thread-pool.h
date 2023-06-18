#ifndef ASLAM_THREAD_POOL_H
#define ASLAM_THREAD_POOL_H

// Adapted from https://github.com/progschj/ThreadPool on September 3, 2014
//
// Original copyright:
// Copyright (c) 2012 Jakob Progsch
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
//   1. The origin of this software must not be misrepresented; you must not
//   claim that you wrote the original software. If you use this software
//   in a product, an acknowledgment in the product documentation would be
//   appreciated but is not required.
//
//   2. Altered source versions must be plainly marked as such, and must not be
//   misrepresented as being the original software.
//
//   3. This notice may not be removed or altered from any source
//   distribution.
#include <condition_variable>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

namespace aslam {

class ThreadPool {
 public:
  /// \brief Create a thread pool.
  ///
  /// \param[in] numThreads The number of threads in the pool.
  ThreadPool(const size_t numThreads);
  ~ThreadPool();

  /// \brief Enqueue work for the thread pool
  ///
  /// Pass in a function and its arguments to enqueue work in the thread pool
  /// \param[in] function A function pointer to be called by a thread.
  /// \param[in] exclusivity_group_id All tasks belonging to the same group id
  ///            are executed in series and the order is guaranteed. A group id
  ///            of kGroupdIdNonExclusiveTask means there are no such guarantees.
  /// \returns A std::future that will return the result of calling function.
  ///          If this function is called after the thread pool has been
  ///          stopped, it will return an uninitialized future that will return
  ///          future.valid() == false
  template<class Function, class ... Args>
  std::future<typename std::result_of<Function(Args...)>::type>
  enqueueOrdered(const size_t exclusivity_group_id, Function&& function,
                 Args&&... args);
  /// Same as method enqueueOrdered but the group id is set to -1 per default
  /// and all tasks are started in order but there is no guarantee on the result
  /// order.
  template<class Function, class... Args>
  std::future<typename std::result_of<Function(Args...)>::type>
  enqueue(Function&& function, Args&&... args);

  /// \brief Stop the thread pool. This method is non-blocking.
  void stop(){ stop_ = true; }

  // Number of queued tasks.
  size_t numQueuedTasks() const;
  /// This method blocks until the queue is empty.
  void waitForEmptyQueue() const;

  static constexpr size_t kGroupdIdNonExclusiveTask =
      std::numeric_limits<size_t>::max();

  size_t numActiveThreads() const;
 private:
  // This version is not threadsafe.
  size_t numQueuedTasksImpl() const;

  /// \brief Run a single thread.
  void run();
  /// Need to keep track of threads so we can join them.
  std::vector<std::thread> workers_;

  // The group id is a size_t where the number kGroupdIdNonExclusiveTask
  // represents a non-exclusive task that needs no guarantees on its execution
  // order. All tasks with other group ids have guaranteed execution order that
  // corresponds to order of enqueing the task.
  typedef std::deque<std::pair<size_t, std::function<void()>>> TaskDeque;
  TaskDeque groupid_tasks_;
  // The guard map should never contain negative group ids.
  typedef std::unordered_map<size_t, bool> GuardMap;
  GuardMap groupid_exclusivity_guards_;
  // Count the number of non-exclusive tasks in groupid_tasks_;
  // where groupid == kGroupdIdNonExclusiveTask.
  size_t num_queued_nonexclusive_tasks = 0u;

  // A mutex to protect the list of tasks of the group list.
  mutable std::mutex tasks_mutex_;

  // A condition variable that signals a change in the task queue; either a
  // removed or inserted element.
  mutable std::condition_variable tasks_queue_change_;

  // A counter of active threads
  unsigned active_threads_;
  // A signal to stop the threads.
  volatile bool stop_;
};

// Add new work item to the pool.
template<class Function, class... Args>
std::future<typename std::result_of<Function(Args...)>::type>
ThreadPool::enqueue(Function&& function, Args&&... args) {
  return enqueueOrdered(kGroupdIdNonExclusiveTask, function,
                        std::forward<Args>(args)...);
}

// Add new work item to the pool.
template<class Function, class... Args>
std::future<typename std::result_of<Function(Args...)>::type>
ThreadPool::enqueueOrdered(const size_t exclusivity_group_id,
                           Function&& function, Args&&... args) {
  typedef typename std::result_of<Function(Args...)>::type return_type;
  // Don't allow enqueueing after stopping the pool.
  if(stop_) {
    LOG(ERROR) << "enqueue() called on stopped ThreadPool";
    // An empty future will return valid() == false.
    return std::future<typename std::result_of<Function(Args...)>::type>();
  }
  auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(function, std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(tasks_mutex_);
    groupid_tasks_.emplace_back(exclusivity_group_id, [task](){ (*task)();});

    // Initialize a group id exclusivity guard.
    if (exclusivity_group_id == kGroupdIdNonExclusiveTask) {
      ++num_queued_nonexclusive_tasks;
    } else if (groupid_exclusivity_guards_.count(
        exclusivity_group_id) == 0u) {
      groupid_exclusivity_guards_.emplace(exclusivity_group_id, false);
    }
  }
  tasks_queue_change_.notify_one();
  return res;
}

}  // namespace aslam

#endif // ASLAM_THREAD_POOL_H
