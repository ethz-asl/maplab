#ifndef MAPLAB_COMMON_BIDIRECTIONAL_MAP_H_
#define MAPLAB_COMMON_BIDIRECTIONAL_MAP_H_
#include <unordered_map>

#include <gtest/gtest_prod.h>

#include "maplab-common/accessors.h"

namespace common {
template <typename TypeLeft, typename TypeRight>
class BidirectionalMap {
 public:
  friend class BidirectionalMapTest;
  static_assert(!std::is_pointer<TypeLeft>::value, "Pointer not supported.");
  static_assert(!std::is_pointer<TypeRight>::value, "Pointer not supported.");

  // Returns false if an element is already in the map and leaves the map
  // unmodified.
  bool insert(const TypeLeft& left, const TypeRight& right) {
    const auto left_it_success = left_to_right_.emplace(left, nullptr);
    if (!left_it_success.second) {
      return false;
    }
    const TypeLeft* left_ptr = &left_it_success.first->first;
    const auto right_it_success = right_to_left_.emplace(right, left_ptr);
    if (!right_it_success.second) {
      // Remove the added element again. It should exist.
      left_to_right_.erase(left_it_success.first);
      return false;
    }
    left_it_success.first->second = &right_it_success.first->first;
    return true;
  }

  // All getters return a nullptr if key is not available.
  const TypeLeft* getLeft(const TypeRight& right) const {
    const TypeLeft* const* result = common::getValuePtr(right_to_left_, right);
    if (result == nullptr) {
      return nullptr;
    }
    return *result;
  }
  const TypeRight* getRight(const TypeLeft& left) const {
    const TypeRight* const* result = common::getValuePtr(left_to_right_, left);
    if (result == nullptr) {
      return nullptr;
    }
    return *result;
  }

 private:
  std::unordered_map<TypeLeft, const TypeRight*> left_to_right_;
  std::unordered_map<TypeRight, const TypeLeft*> right_to_left_;
};
}  // namespace common
#endif  // MAPLAB_COMMON_BIDIRECTIONAL_MAP_H_
