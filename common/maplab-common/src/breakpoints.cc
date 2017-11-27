#include "maplab-common/breakpoints.h"

#include <cstdio>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_int32(breakpoint_level, 0, "Breakpoint level.");

namespace breakpoints {

void BreakpointWithLevel(std::string message, int32_t level) {
  if (FLAGS_breakpoint_level < level)
    return;
  LOG(WARNING) << "Break point: " << message;
  getchar();
}

void BreakpointUnconditional(std::string message) {
  LOG(WARNING) << "Break point: " << message;
  getchar();
}

}  // namespace breakpoints
