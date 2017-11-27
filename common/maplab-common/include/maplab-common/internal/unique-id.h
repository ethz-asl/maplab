#ifndef MAPLAB_COMMON_INTERNAL_UNIQUE_ID_H_  // NOLINT
#define MAPLAB_COMMON_INTERNAL_UNIQUE_ID_H_  // NOLINT

#include <atomic>
#include <string>

#include "maplab-common/id.pb.h"

namespace common {
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

  void saltSeed(const Key&, size_t salt) {
    seed_ ^= salt;
  }

  size_t seed() const {
    return seed_;
  }

 private:
  std::atomic<size_t> seed_;
};

void generateUnique128BitHash(uint64_t hash[2]);
}  // namespace internal
}  // namespace common

#endif  // MAPLAB_COMMON_INTERNAL_UNIQUE_ID_H_  NOLINT
