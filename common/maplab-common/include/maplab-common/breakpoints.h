#ifndef MAPLAB_COMMON_BREAKPOINTS_H_
#define MAPLAB_COMMON_BREAKPOINTS_H_

#include <string>

namespace breakpoints {
/// Breakpoint that is active depending on the set breakpoint level.
void BreakpointWithLevel(std::string message, int32_t level);
/// Unconditional breakpoint.
void BreakpointUnconditional(std::string message);
}  // namespace breakpoints

#endif  // MAPLAB_COMMON_BREAKPOINTS_H_
