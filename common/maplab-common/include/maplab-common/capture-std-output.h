#ifndef MAPLAB_COMMON_CAPTURE_STD_OUTPUT_H_
#define MAPLAB_COMMON_CAPTURE_STD_OUTPUT_H_

#include <iostream>  // NOLINT
#include <mutex>
#include <ostream>  // NOLINT
#include <streambuf>
#include <string>

#include <glog/logging.h>

namespace common {
/// \class CaptureStandardOutput
/// \brief Capture the output that is streamed to std::cout in the active scope.
///        E.g. usage:
///        std::string data;
///        {
///          common::CaptureStandardOutput capture;
///          extranal_function.print(); // uses std::cout
///          data = capture.getString();
///        }
class CaptureStandardOutput {
 public:
  inline CaptureStandardOutput() {
    previous_cout_buffer_ = std::cout.rdbuf();
    std::cout.rdbuf(buffer_.rdbuf());
  }

  ~CaptureStandardOutput() {
    std::cout.rdbuf(CHECK_NOTNULL(previous_cout_buffer_));
  }

  inline std::string getString() {
    return buffer_.str();
  }

 private:
  std::stringstream buffer_;
  std::streambuf* previous_cout_buffer_;
};

/// \class RedirectStandardOutput
/// \brief Redirect from one stream to another stream.
/// E.g. common::RedirectStandardOutput<> redirect(std::cout, mycallback);
///  {
///      // Redirects std::cout to LOG(ERROR)
///      RedirectStandardOutputToGlog<> capture(std::cout, google::ERROR);
///      std::cout << "Test"
///  }
///  {
///      // Redirects std::cout to VLOG(20)
///      RedirectStandardOutputToGlog<> capture(std::cout, google::INFO, 20);
///      std::cout << "Test"
///  }
template <class ElementType = char, class Tr = std::char_traits<ElementType>>
class RedirectStreamToGlog : public std::basic_streambuf<ElementType, Tr> {
 public:
  RedirectStreamToGlog(
      std::ostream& source_stream, int log_severity,
      int vlog_level = 0)  // NOLINT
      : source_stream_(source_stream),
        log_severity_(log_severity),
        vlog_level_(vlog_level) {
    original_source_buffer_ = source_stream_.rdbuf(this);
  }

  virtual ~RedirectStreamToGlog() {
    source_stream_.rdbuf(CHECK_NOTNULL(original_source_buffer_));
    printLineBuffer();
  }

  inline std::streamsize xsputn(
      const ElementType* _Ptr, std::streamsize _Count) {
    dataCallback(_Ptr, _Count);
    return _Count;
  }

  inline typename Tr::int_type overflow(typename Tr::int_type v) {
    ElementType ch = Tr::to_char_type(v);
    dataCallback(&ch, 1);
    return Tr::not_eof(v);
  }

 private:
  inline void dataCallback(const char* ptr, std::streamsize count) {
    std::unique_lock<std::mutex> lock(m_buffer_);
    for (int idx = 0; idx < count; ++idx) {
      if (ptr[idx] == '\n') {
        printLineBuffer();
      } else {
        line_buffer_ += ptr[idx];
      }
    }
  }

  inline void printLineBuffer() {
    if (!line_buffer_.empty()) {
      if (VLOG_IS_ON(vlog_level_)) {
        google::LogMessage("stream-redirect", 0, log_severity_).stream()
            << line_buffer_;
      }
      line_buffer_.clear();
    }
  }

 private:
  std::basic_ostream<ElementType, Tr>& source_stream_;
  std::streambuf* original_source_buffer_;
  std::string line_buffer_;
  std::mutex m_buffer_;
  int log_severity_;
  int vlog_level_;
};

}  // namespace common
#endif  // MAPLAB_COMMON_CAPTURE_STD_OUTPUT_H_
