#ifndef MAPLAB_COMMON_THREADING_HELPERS_H_
#define MAPLAB_COMMON_THREADING_HELPERS_H_

#include <cstddef>

namespace common {

// Return the number of concurrent threads supported by the hardware.
size_t getNumHardwareThreads();

}  // namespace common

#endif  // MAPLAB_COMMON_THREADING_HELPERS_H_
