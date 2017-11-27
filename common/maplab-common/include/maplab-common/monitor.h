#ifndef MAPLAB_COMMON_MONITOR_H_
#define MAPLAB_COMMON_MONITOR_H_

#include <mutex>

#include <aslam/common/reader-writer-lock.h>
#include <glog/logging.h>

namespace common {

// Recommend to use this e.g. for member variables that must be protected by a
// mutex; this will
// enforce mutex lock in order to access.
template <typename MonitoredType>
class Monitor {
 public:
  Monitor() {}

  explicit Monitor(MonitoredType object) : object_(object) {}

  explicit Monitor(const Monitor<MonitoredType>& other)
      : object_(other.object_) {}

  class WriteAccess {
   public:
    WriteAccess(MonitoredType* object, aslam::ReaderWriterMutex* mutex)
        : object_(CHECK_NOTNULL(object)), lock_(mutex) {}

    MonitoredType* operator->() {
      return object_;
    }
    MonitoredType& operator*() {
      return *object_;
    }
    MonitoredType* get() {
      return object_;
    }

   private:
    MonitoredType* object_;
    mutable aslam::ScopedWriteLock lock_;
  };

  class ReadAccess {
   public:
    ReadAccess(const MonitoredType* object, aslam::ReaderWriterMutex* mutex)
        : object_(CHECK_NOTNULL(object)), lock_(mutex) {}

    const MonitoredType* operator->() const {
      return object_;
    }
    const MonitoredType& operator*() const {
      return *object_;
    }
    const MonitoredType* get() const {
      return object_;
    }

   private:
    const MonitoredType* object_;
    mutable aslam::ScopedReadLock lock_;
  };

  WriteAccess getWriteAccess() {
    return WriteAccess(&object_, &mutex_);
  }
  WriteAccess* allocatedWriteAccess() {
    return new WriteAccess(&object_, &mutex_);
  }

  ReadAccess getReadAccess() {
    return ReadAccess(&object_, &mutex_);
  }
  ReadAccess* allocatedReadAccess() {
    return new ReadAccess(&object_, &mutex_);
  }

 private:
  MonitoredType object_;
  mutable aslam::ReaderWriterMutex mutex_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_MONITOR_H_
