#ifndef MAPLAB_COMMON_GLOG_HELPERS_H_
#define MAPLAB_COMMON_GLOG_HELPERS_H_

#include <stdlib.h>

#include <glog/logging.h>

namespace common {

// This function can be registered as a failure handler
// with glog and will reset the console to non-raw mode
// before issuing the SIGABRT signal with:
//   google::InstallFailureFunction(
//     &common::glogFailFunctionWithConsoleRecovery);
inline void glogFailFunctionWithConsoleRecovery() {
  // Set the console back to non-raw mode.
  int retval = system("stty sane");
  ++retval;  // Avoid unused warning.
  abort();
}

namespace glog_helpers {
// Useful for initializer lists.
template <typename Type>
inline Type checkNotZero(const Type in) {
  CHECK_NE(in, 0);
  return in;
}

}  // namespace glog_helpers
}  // namespace common
#endif  // MAPLAB_COMMON_GLOG_HELPERS_H_
