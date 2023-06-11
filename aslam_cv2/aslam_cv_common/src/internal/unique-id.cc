#include "aslam/common/internal/unique-id.h"

#include <atomic>
#include <chrono>

namespace aslam {
namespace internal {
void generateUnique128BitHash(uint64_t hash[2]) {
  static std::atomic<uint64_t> counter;
  hash[0] =
      aslam::internal::UniqueIdHashSeed::instance().seed() ^
      std::hash<uint64_t>()(
          std::chrono::high_resolution_clock::now().time_since_epoch().count());
  // Increment must happen here, otherwise the sampling is not atomic.
  hash[1] = std::hash<uint64_t>()(++counter);
}
}  // namespace internal
}  // namespace common
