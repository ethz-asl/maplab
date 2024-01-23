#include <algorithm>

#include <aslam/common/thread-pool.h>

namespace aslam {

// The constructor just launches some amount of workers.
ThreadPool::ThreadPool(const size_t threads)
    : active_threads_(0),
      stop_(false) {
  for (size_t i = 0; i < threads; ++i)
    workers_.emplace_back(std::bind(&ThreadPool::run, this));
}

// The destructor joins all threads.
ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(tasks_mutex_);
    stop_ = true;
  }
  tasks_queue_change_.notify_all();
  for (size_t i = 0u; i < workers_.size(); ++i) {
    workers_[i].join();
  }
}

void ThreadPool::run() {
  while (true) {
    std::unique_lock<std::mutex> lock(this->tasks_mutex_);

    // Here we need to select the next task from a queue that is not already
    // serviced.
    std::function<void()> task;
    size_t group_id = kGroupdIdNonExclusiveTask;
    while (true) {
      const bool all_guards_active = std::all_of(
          groupid_exclusivity_guards_.begin(),
          groupid_exclusivity_guards_.end(),
          [](const GuardMap::value_type& value) {
            CHECK_NE(value.second, kGroupdIdNonExclusiveTask)
              << "There should never be a guard for a non-exclusive task.";
            return value.second;
          });

      if (!all_guards_active || num_queued_nonexclusive_tasks > 0u) {
        // If not all guards are active, we select a task to process; otherwise
        // we can go back to sleep until a thread reports back for work.
        size_t index = 0u;
        for (const TaskDeque::value_type& groupid_task : groupid_tasks_) {
          // We have found a task to process if no thread is already working on
          // this group id.
          const bool is_exclusive_task =
              groupid_task.first != kGroupdIdNonExclusiveTask;
          bool guard_active = false;
          if (is_exclusive_task) {
            guard_active = groupid_exclusivity_guards_[groupid_task.first];
          }

          if (!(is_exclusive_task && guard_active)) {
            group_id = groupid_task.first;
            task = groupid_task.second;

            groupid_tasks_.erase(groupid_tasks_.begin() + index);
            if (!is_exclusive_task) {
              --num_queued_nonexclusive_tasks;
            }

            // We jump out of the nested for-structure here, because we have
            // found a task to process.
            break;
          }
          ++index;
        }
      }
      if (task) {
        break;
      }

      // Wait until the queue has changed (addition/removal) before re-checking
      // for new tasks to process.
      if (this->stop_ && groupid_tasks_.size() == 0u) {
        return;
      }

      this->tasks_queue_change_.wait(lock);
    }

    // We jump here if we found a task.
    CHECK(task);
    ++active_threads_;

    // Make sure the no other thread is currently working on this exclusivity
    // group.
    if (group_id != kGroupdIdNonExclusiveTask) {
      const GuardMap::iterator it_group_id_serviced =
          groupid_exclusivity_guards_.find(group_id);
      CHECK(it_group_id_serviced == groupid_exclusivity_guards_.end() ||
            it_group_id_serviced->second == false);
      it_group_id_serviced->second = true;
    }

    // Unlock the queue while we execute the task.
    lock.unlock();
    task();
    lock.lock();

    // Release the group for other threads.
    if (group_id != kGroupdIdNonExclusiveTask) {
      const GuardMap::iterator it_group_id_servied =
          groupid_exclusivity_guards_.find(group_id);
      CHECK(it_group_id_servied != groupid_exclusivity_guards_.end() &&
            it_group_id_servied->second == true);
      it_group_id_servied->second = false;
    }

    --active_threads_;

    // This is the secret to making the waitForEmptyQueue() function work.
    // After finishing a task, notify that this work is done.
    tasks_queue_change_.notify_all();
  }
}

size_t ThreadPool::numActiveThreads() const {
  std::unique_lock<std::mutex> lock(this->tasks_mutex_);
  return active_threads_;
}

void ThreadPool::waitForEmptyQueue() const {
  std::unique_lock<std::mutex> lock(this->tasks_mutex_);
  // Only exit if all tasks are complete by tracking the number of
  // active threads.
  while (active_threads_ > 0u || groupid_tasks_.size() > 0u) {
    this->tasks_queue_change_.wait(lock);
  }
}
}  // namespace aslam
