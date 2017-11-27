#include "maplab-common/file-logger.h"

#include <fstream>  // NOLINT
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>

namespace common {
FileLogger::FileLogger(std::string const& filename) {
  file_handle_.reset(new std::ofstream);
  file_handle_->open(filename, std::ofstream::out | std::ofstream::trunc);

  if (!file_handle_->is_open()) {
    LOG(ERROR) << "Could not open: " << filename;
    file_handle_.reset();
    return;
  }
  file_handle_->precision(std::numeric_limits<double>::digits10);
}

FileLogger::~FileLogger() {
  if (isOpen()) {
    flushBuffer();
    closeFile();
  }
}

void FileLogger::closeFile() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  CHECK(isOpen());
  file_handle_->flush();
  file_handle_->close();
}

void FileLogger::flushBuffer() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  CHECK(isOpen());
  file_handle_->flush();
}

bool FileLogger::isOpen() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (file_handle_ == nullptr) {
    return false;
  }
  return file_handle_->is_open() && file_handle_->good();
}


const FileLogger& FileLogger::operator<<(
    std::ostream& (*object)(std::ostream&)) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  CHECK(isOpen());
  *file_handle_ << object;
  return *this;
}

const FileLogger& FileLogger::operator<<(
    std::ios_base& (*object)(std::ios_base&)) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  CHECK(isOpen());
  *file_handle_ << object;
  return *this;
}

} // namespace common
