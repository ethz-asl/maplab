#ifndef MAPLAB_COMMON_FILE_LOCK_H_
#define MAPLAB_COMMON_FILE_LOCK_H_

#include <fstream>  // NOLINT
#include <string>
#include <sys/file.h>

namespace common {

// Note: This is not thread safe.
// Credit to the author of discovery file lock in file-discovery.cc.
class FileLock {
 public:
  explicit FileLock(const std::string& file_name);
  ~FileLock();
  void lock();
  void unlock();

 private:
  FileLock(const FileLock&) = delete;
  FileLock& operator=(const FileLock&) = delete;

  std::string lock_file_name_;
  int file_descriptor_ = -1;
  bool is_locked_;
};

class ScopedFileLock {
 public:
  explicit ScopedFileLock(const std::string& file_name);
  ~ScopedFileLock();

 private:
  explicit ScopedFileLock(const FileLock&) = delete;
  ScopedFileLock& operator=(const FileLock&) = delete;
  FileLock* file_lock_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_FILE_LOCK_H_
