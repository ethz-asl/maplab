#ifndef MAPLAB_COMMON_PROTO_HELPERS_H_
#define MAPLAB_COMMON_PROTO_HELPERS_H_
#include <algorithm>

#include <google/protobuf/stubs/common.h>

template <typename T>
bool operator==(
    const ::google::protobuf::RepeatedField<T>& lhs,
    const ::google::protobuf::RepeatedField<T>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

#endif  // MAPLAB_COMMON_PROTO_HELPERS_H_
