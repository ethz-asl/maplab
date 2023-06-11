#ifndef ASLAM_COMMON_INTERNAL_UNIQUE_ID_H_  // NOLINT
#define ASLAM_COMMON_INTERNAL_UNIQUE_ID_H_  // NOLINT

#include <atomic>
#include <string>

#include "aslam/common/id.pb.h"

namespace aslam {
namespace internal {
class UniqueIdHashSeed {
 public:
  class Key {
    Key() {}
  };

  UniqueIdHashSeed() : seed_(31u) {}
  static UniqueIdHashSeed& instance() {
    static UniqueIdHashSeed instance;
    return instance;
  }

  void saltSeed(const Key&, uint64_t salt) {
    seed_ ^= salt;
  }

  uint64_t seed() const {
    return seed_;
  }

 private:
  std::atomic<uint64_t> seed_;
};

void generateUnique128BitHash(uint64_t hash[2]);
}  // namespace internal
}  // namespace aslam

#endif  // ASLAM_COMMON_INTERNAL_UNIQUE_ID_H_  NOLINT
