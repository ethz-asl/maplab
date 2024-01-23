#ifndef ASLAM_COMMON_READER_WRITER_LOCK_H_
#define ASLAM_COMMON_READER_WRITER_LOCK_H_

#include <condition_variable>
#include <mutex>

#include <glog/logging.h>

// Adapted from http://www.paulbridger.com/read_write_lock/
namespace aslam {
class ReaderWriterMutex {
 public:
  ReaderWriterMutex();
  virtual ~ReaderWriterMutex();

  virtual void acquireReadLock();
  void releaseReadLock();

  virtual void acquireWriteLock();
  virtual void releaseWriteLock();

  // Attempt upgrade. If upgrade fails, relinquish read lock.
  virtual bool upgradeToWriteLock();

  // Returns true if there are any active readers, writers, or threads that are waiting for read or
  // write access.
  bool isInUse();

 protected:
  ReaderWriterMutex(const ReaderWriterMutex&) = delete;
  ReaderWriterMutex& operator=(const ReaderWriterMutex&) = delete;

  std::mutex mutex_;
  unsigned int pending_readers_;
  unsigned int num_readers_;
  std::condition_variable cv_readers_;
  unsigned int num_pending_writers_;
  bool current_writer_;
  std::condition_variable cv_writer_finished_;
  bool pending_upgrade_;
};

class ScopedReadLock {
 public:
  explicit ScopedReadLock(ReaderWriterMutex* rw_lock);
  ScopedReadLock(ScopedReadLock&& other);
  ~ScopedReadLock();

 private:
  ScopedReadLock(const ScopedReadLock&) = delete;
  ScopedReadLock& operator=(const ScopedReadLock&) = delete;
  ReaderWriterMutex* rw_lock_;
};

class ScopedWriteLock {
 public:
  explicit ScopedWriteLock(ReaderWriterMutex* rw_lock);
  ScopedWriteLock(ScopedWriteLock&& other);
  ~ScopedWriteLock();

 private:
  ScopedWriteLock(const ScopedWriteLock&) = delete;
  ScopedWriteLock& operator=(const ScopedWriteLock&) = delete;
  ReaderWriterMutex* rw_lock_;
};

}  // namespace aslam
#endif  // ASLAM_COMMON_READER_WRITER_LOCK_H_
