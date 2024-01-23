#ifndef ASLAM_COMMON_READER_FIRST_READER_WRITER_LOCK_H_
#define ASLAM_COMMON_READER_FIRST_READER_WRITER_LOCK_H_

#include "aslam/common/reader-writer-lock.h"

namespace aslam {

class ReaderFirstReaderWriterMutex : public ReaderWriterMutex {
 public:
  ReaderFirstReaderWriterMutex();
  ~ReaderFirstReaderWriterMutex();

  virtual void acquireReadLock() override;

  virtual void acquireWriteLock() override;
};

}  // namespace aslam

#endif  // ASLAM_COMMON_READER_FIRST_READER_WRITER_LOCK_H_
