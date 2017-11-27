#include "maplab-common/file-lock.h"

#include <chrono>

#include <glog/logging.h>

#include "maplab-common/conversions.h"

// Adapted from http://www.paulbridger.com/read_write_lock/
namespace common {

FileLock::FileLock(const std::string& file_name) : is_locked_(false) {
  lock_file_name_ = file_name + ".lck";
}

FileLock::~FileLock() {
  CHECK(!is_locked_);
}

void FileLock::lock() {
  using std::chrono::steady_clock;
  steady_clock::time_point start = steady_clock::now();
  while (true) {
    {
      bool status = ((file_descriptor_ = open(
                          lock_file_name_.c_str(), O_WRONLY | O_EXCL | O_CREAT,
                          0)) == -1) &&
                    errno == EEXIST;
      if (!status) {
        break;
      }
    }
    usleep(1e4);
    steady_clock::time_point end = steady_clock::now();
    using std::chrono::duration_cast;
    double time_ms =
        duration_cast<std::chrono::milliseconds>(end - start).count();
    if (time_ms > 20 * kSecondsToMilliSeconds) {
      LOG(ERROR) << "Results file lock timed out! "
                 << "Probably there was an outdated lock file present: "
                 << lock_file_name_ << ". "
                 << "The lock file has been deleted and ownership of the lock "
                    "will be forced.";
      CHECK_NE(unlink(lock_file_name_.c_str()), -1);
    }
  }
  is_locked_ = true;
  VLOG(5) << "lock file " << lock_file_name_;
}

void FileLock::unlock() {
  CHECK(is_locked_) << "Attempt to unlock when not locked.";
  VLOG(5) << "unlock file " << lock_file_name_;
  CHECK_NE(close(file_descriptor_), -1) << errno;
  unlink(lock_file_name_.c_str());
  is_locked_ = false;
}

ScopedFileLock::ScopedFileLock(const std::string& file_name) {
  file_lock_ = new FileLock(file_name);
  file_lock_->lock();
}

ScopedFileLock::~ScopedFileLock() {
  file_lock_->unlock();
}

}  // namespace common
