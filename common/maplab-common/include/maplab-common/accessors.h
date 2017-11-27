#ifndef MAPLAB_COMMON_ACCESSORS_H_
#define MAPLAB_COMMON_ACCESSORS_H_

#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <aslam/common/memory.h>
#include <glog/logging.h>

/// Returns a const reference to the value for the specified key in an
/// std::unordered_map.
/// Fails hard if the keys does not exist.
namespace common {
template <typename KeyType, typename ValueType, template <typename> class Hash,
          template <typename> class Comparator,
          template <typename> class Allocator>
inline const ValueType& getChecked(
    const std::unordered_map<
        KeyType, ValueType, Hash<KeyType>, Comparator<KeyType>,
        Allocator<std::pair<const KeyType, ValueType>>>& map,
    const KeyType& key) {
  typename std::unordered_map<
      KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
      Allocator<std::pair<const KeyType, ValueType>>>::const_iterator it =
      map.find(key);
  CHECK(it != map.end());
  return it->second;
}

/// Returns a non-const reference to the value for the specified key in an
/// std::unordered_map. Fails hard if the keys does not exist.
template <typename KeyType, typename ValueType, template <typename> class Hash,
          template <typename> class Comparator,
          template <typename> class Allocator>
inline ValueType& getChecked(
    std::unordered_map<KeyType, ValueType, Hash<KeyType>, Comparator<KeyType>,
                       Allocator<std::pair<const KeyType, ValueType>>>&
        map,  // NOLINT
    const KeyType& key) {
  typename std::unordered_map<
      KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
      Allocator<std::pair<const KeyType, ValueType>>>::iterator it =
      map.find(key);
  CHECK(it != map.end());
  return it->second;
}

/// Returns the value at key or the specified default value
/// if the key doesn't exist in the map.
template <typename KeyType, typename ValueType, template <typename> class Hash,
          template <typename> class Comparator,
          template <typename> class Allocator>
inline const ValueType& getValueOrDefault(
    std::unordered_map<KeyType, ValueType, Hash<KeyType>, Comparator<KeyType>,
                       Allocator<std::pair<const KeyType, ValueType>>>&
        map,  // NOLINT
    const KeyType& key,
    const ValueType& default_value) {
  typename std::unordered_map<
      KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
      Allocator<std::pair<const KeyType, ValueType>>>::const_iterator it =
      map.find(key);
  if (it == map.end()) {
    return default_value;
  }
  return it->second;
}

/// Returns a non-const pointer to the value for the specified key in an
/// std::unordered_map. Returns nullptr if the key doesn't exists.
template <typename KeyType, typename ValueType, template <typename> class Hash,
          template <typename> class Comparator,
          template <typename> class Allocator>
inline ValueType* getValuePtr(
    std::unordered_map<KeyType, ValueType, Hash<KeyType>, Comparator<KeyType>,
                       Allocator<std::pair<const KeyType, ValueType>>>&
        map,  // NOLINT
    const KeyType& key) {
  typename std::unordered_map<
      KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
      Allocator<std::pair<const KeyType, ValueType>>>::iterator it =
      map.find(key);
  if (it == map.end()) {
    return nullptr;
  }
  return &it->second;
}

/// Returns a const pointer to the value for the specified key in an
/// std::unordered_map. Returns nullptr if the key doesn't exists.
template <typename KeyType, typename ValueType, template <typename> class Hash,
          template <typename> class Comparator,
          template <typename> class Allocator>
inline const ValueType* getValuePtr(
    const std::unordered_map<
        KeyType, ValueType, Hash<KeyType>, Comparator<KeyType>,
        Allocator<std::pair<const KeyType, ValueType>>>& map,
    const KeyType& key) {
  typename std::unordered_map<
      KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
      Allocator<std::pair<const KeyType, ValueType>>>::const_iterator it =
      map.find(key);
  if (it == map.end()) {
    return nullptr;
  }
  return &it->second;
}

template <typename ValueType, template <typename> class Allocator>
bool containsValue(
    const std::vector<ValueType, Allocator<ValueType>>& vec,
    const ValueType& value) {
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}

}  // namespace common

#endif  // MAPLAB_COMMON_ACCESSORS_H_
