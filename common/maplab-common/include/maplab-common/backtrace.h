#ifndef MAPLAB_COMMON_BACKTRACE_H_
#define MAPLAB_COMMON_BACKTRACE_H_

#include <cxxabi.h>
#include <execinfo.h>
#include <memory>
#include <sstream>  // NOLINT
#include <string>

namespace common {

// Adapted from http://panthema.net/2008/0901-stacktrace-demangled/ .
inline std::string backtrace() {
  constexpr size_t kBacktraceLimit = 64;
  void* array[kBacktraceLimit];
  size_t size;

  size = ::backtrace(array, kBacktraceLimit);
  // Causes
  // http://valgrind.org/docs/manual/mc-manual.html#opt.show-mismatched-frees ,
  // but that is ok.
  std::unique_ptr<char* []> symbollist(backtrace_symbols(array, size));

  if (size == 0u) {
    return "  Backtrace empty, possibly corrupt!";
  }

  std::stringstream result;
  for (size_t i = 1u; i < size; i++) {
    char *begin_name = 0, *begin_offset = 0, *end_offset = 0;

    // Find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char* p = symbollist[i]; *p; ++p) {
      if (*p == '(') {
        begin_name = p;
      } else if (*p == '+') {
        begin_offset = p;
      } else if (*p == ')' && begin_offset) {
        end_offset = p;
        break;
      }
    }

    if (begin_name && begin_offset && end_offset && begin_name < begin_offset) {
      *begin_name++ = '\0';
      *begin_offset++ = '\0';
      *end_offset = '\0';

      // Mangled name is now in [begin_name, begin_offset) and caller
      // offset in [begin_offset, end_offset).
      int status;
      // Causes
      // http://valgrind.org/docs/manual/mc-manual.html#opt.show-mismatched-frees
      // but that is ok.
      std::unique_ptr<char> demangled(
          abi::__cxa_demangle(begin_name, nullptr, nullptr, &status));
      if (status == 0) {
        result << "  " << symbollist[i] << " : " << demangled.get() << "+"
               << begin_offset << "\n";
      } else {
        // Demangling failed. Output function name as a C function with
        // no arguments.
        result << "  " << symbollist[i] << " : " << begin_name << "+"
               << begin_offset << "\n";
      }
    } else {
      // Couldn't parse the line? Print the whole line.
      result << "  " << symbollist[i] << "\n";
    }
  }

  return result.str();
}

}  // namespace common

#endif  // MAPLAB_COMMON_BACKTRACE_H_
