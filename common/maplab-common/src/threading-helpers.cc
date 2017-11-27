#include "maplab-common/threading-helpers.h"

#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_uint64(
    num_hardware_threads, 0u,
    "Number of hardware threads to announce. (0: autodetect)");

namespace common {
namespace internal {

constexpr size_t kDefaultNumHardwareThreads = 4u;

size_t getNumHardwareThreadsImpl() {
  // Just use the user-provided count if set.
  if (FLAGS_num_hardware_threads > 0) {
    return FLAGS_num_hardware_threads;
  }

  const size_t num_detected_threads = std::thread::hardware_concurrency();

  // Fallback to default or user-provided value if the detection failed.
  if (num_detected_threads == 0) {
    static bool warned_once = false;
    if (!warned_once) {
      LOG(WARNING) << "Could not detect the number of hardware threads. "
                   << "Using default of " << kDefaultNumHardwareThreads
                   << ". This can be overridden using the flag "
                   << "num_hardware_threads.";
      warned_once = true;
    }
    return kDefaultNumHardwareThreads;
  }
  return num_detected_threads;
}
}  // namespace internal

size_t getNumHardwareThreads() {
  static size_t cached_num_threads = internal::getNumHardwareThreadsImpl();
  return cached_num_threads;
}

}  // namespace common
