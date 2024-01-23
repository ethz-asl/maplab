#include "aslam/common/reader-writer-lock.h"

// Adapted from http://www.paulbridger.com/read_write_lock/
namespace aslam {

ReaderWriterMutex::ReaderWriterMutex()
    : pending_readers_(0u),
      num_readers_(0u),
      num_pending_writers_(0u),
      current_writer_(false),
      pending_upgrade_(false) {}

ReaderWriterMutex::~ReaderWriterMutex() {}

void ReaderWriterMutex::acquireReadLock() {
  std::unique_lock<std::mutex> lock(mutex_);
  ++pending_readers_;
  while (num_pending_writers_ != 0u || pending_upgrade_ || current_writer_) {
    cv_writer_finished_.wait(lock);
  }
  --pending_readers_;
  ++num_readers_;
}

void ReaderWriterMutex::releaseReadLock() {
  std::unique_lock<std::mutex> lock(mutex_);
  --num_readers_;
  if (num_readers_ == (pending_upgrade_ ? 1u : 0u)) {
    cv_readers_.notify_all();
  }
}

void ReaderWriterMutex::acquireWriteLock() {
  std::unique_lock<std::mutex> lock(mutex_);
  ++num_pending_writers_;
  while (num_readers_ > (pending_upgrade_ ? 1u : 0u)) {
    cv_readers_.wait(lock);
  }
  while (current_writer_ || pending_upgrade_) {
    cv_writer_finished_.wait(lock);
  }
  --num_pending_writers_;
  current_writer_ = true;
}

void ReaderWriterMutex::releaseWriteLock() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    current_writer_ = false;
  }
  cv_writer_finished_.notify_all();
}

// Attempt upgrade. If upgrade fails, relinquish read lock.
bool ReaderWriterMutex::upgradeToWriteLock() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (pending_upgrade_) {
    --num_readers_;
    if (num_readers_ == 1u) {
      cv_readers_.notify_all();
    }
    return false;
  }
  pending_upgrade_ = true;
  while (num_readers_ > 1) {
    cv_readers_.wait(lock);
  }
  pending_upgrade_ = false;
  --num_readers_;
  current_writer_ = true;
  if (num_readers_ == 0u) {
    cv_readers_.notify_all();
  }
  return true;
}

bool ReaderWriterMutex::isInUse() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (pending_readers_ > 0u || num_readers_ > 0u || num_pending_writers_ > 0u || current_writer_ ||
      pending_upgrade_) {
    // Active readers, writers, or threads are waiting for access.
    return true;
  }
  return false;
}

ScopedReadLock::ScopedReadLock(ReaderWriterMutex* rw_lock) : rw_lock_(rw_lock) {
  CHECK_NOTNULL(rw_lock_)->acquireReadLock();
}

ScopedReadLock::ScopedReadLock(ScopedReadLock&& other) : rw_lock_(other.rw_lock_) {
  other.rw_lock_ = nullptr;
}

ScopedReadLock::~ScopedReadLock() {
  if (rw_lock_ != nullptr) {
    rw_lock_->releaseReadLock();
  }
}

ScopedWriteLock::ScopedWriteLock(ReaderWriterMutex* rw_lock) : rw_lock_(rw_lock) {
  CHECK_NOTNULL(rw_lock_)->acquireWriteLock();
}

ScopedWriteLock::ScopedWriteLock(ScopedWriteLock&& other) : rw_lock_(other.rw_lock_) {
  other.rw_lock_ = nullptr;
}

ScopedWriteLock::~ScopedWriteLock() {
  if (rw_lock_ != nullptr) {
    rw_lock_->releaseWriteLock();
  }
}

}  // namespace aslam
